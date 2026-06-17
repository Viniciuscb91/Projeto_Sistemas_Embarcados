#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

#define LED_STATUS_PIN    2
#define JOYSTICK_X_CH     ADC_CHANNEL_4 
#define JOYSTICK_Y_CH     ADC_CHANNEL_3 
#define JOYSTICK_SEL_PIN  6

#define SERVO_X_PIN       18
#define SERVO_Y_PIN       19

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 20
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define MPU9250_ADDR      0x68

typedef struct {
    int x_raw;
    int y_raw;
    int btn_pressed;
} joystick_data_t;

QueueHandle_t xJoystickQueue;

int g_joy_x = 2048, g_joy_y = 2048, g_btn = 0; 
int g_servo_x = 307, g_servo_y = 307;          
float g_pitch = 0.0, g_roll = 0.0;             
float g_acc_x_g = 0.0, g_acc_y_g = 0.0, g_acc_z_g = 0.0;

adc_oneshot_unit_handle_t adc_handle;

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t mpu9250_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

esp_err_t mpu9250_read_regs(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
}

void mpu9250_init(void) {
    mpu9250_write_reg(0x6B, 0x00); 
}

void vTaskJoystick(void *pvParameters) {
    joystick_data_t data; 
    while(1) { 
        int sum_x = 0, sum_y = 0;
        const int samples = 4;

        for(int i = 0; i < samples; i++) {
            int temp_x, temp_y;
            adc_oneshot_read(adc_handle, JOYSTICK_X_CH, &temp_x);
            adc_oneshot_read(adc_handle, JOYSTICK_Y_CH, &temp_y);
            sum_x += temp_x;
            sum_y += temp_y;
            vTaskDelay(pdMS_TO_TICKS(2)); 
        }

        data.x_raw = sum_x / samples;
        data.y_raw = sum_y / samples;
        data.btn_pressed = !gpio_get_level(JOYSTICK_SEL_PIN);
        
        g_joy_x = data.x_raw;
        g_joy_y = data.y_raw;
        g_btn = data.btn_pressed;
        
        xQueueOverwrite(xJoystickQueue, &data);
        vTaskDelay(pdMS_TO_TICKS(30)); 
    }
}

void vTaskServos(void *pvParameters) {
    joystick_data_t rx; 
    float s_x = 307.0, s_y = 307.0; 
    
    const int CENTRO_ADC = 2048;    
    const int ZONA_MORTA = 150;      
    const float ALPHA = 0.06
    ;        

    while(1) {
        if(xQueueReceive(xJoystickQueue, &rx, portMAX_DELAY)) {
            int process_x = rx.x_raw;
            int process_y = rx.y_raw;

            if (abs(process_x - CENTRO_ADC) < ZONA_MORTA) process_x = CENTRO_ADC;
            if (abs(process_y - CENTRO_ADC) < ZONA_MORTA) process_y = CENTRO_ADC;

            int tx = 102 + ((process_x * (512 - 102)) / 4095);
            int ty = 102 + ((process_y * (512 - 102)) / 4095);
            
            s_x = (s_x * (1.0 - ALPHA)) + (tx * ALPHA);
            s_y = (s_y * (1.0 - ALPHA)) + (ty * ALPHA);
            
            g_servo_x = (int)s_x;
            g_servo_y = (int)s_y;
            
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, g_servo_x);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g_servo_y);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        }
    }
}

void vTaskMPU9250(void *pvParameters) {
    uint8_t raw_data[14];
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y;
    float acc_pitch, acc_roll, gyro_x_dps, gyro_y_dps;
    const float dt = 0.020; 

    while(1) {
        if (mpu9250_read_regs(0x3B, raw_data, 14) == ESP_OK) {
            accel_x = (raw_data[0] << 8) | raw_data[1];
            accel_y = (raw_data[2] << 8) | raw_data[3];
            accel_z = (raw_data[4] << 8) | raw_data[5];
            gyro_x  = (raw_data[8] << 8) | raw_data[9];
            gyro_y  = (raw_data[10] << 8) | raw_data[11];

            gyro_x_dps = (float)gyro_x / 131.0;
            gyro_y_dps = (float)gyro_y / 131.0;

            g_acc_x_g = (float)accel_x / 16384.0;
            g_acc_y_g = (float)accel_y / 16384.0;
            g_acc_z_g = (float)accel_z / 16384.0;

            acc_pitch = atan2((float)-accel_x, sqrt((float)accel_y * accel_y + (float)accel_z * accel_z)) * 180.0 / M_PI;
            acc_roll  = atan2((float)accel_y, (float)accel_z) * 180.0 / M_PI;

            g_pitch = 0.96 * (g_pitch + gyro_x_dps * dt) + 0.04 * acc_pitch;
            g_roll  = 0.96 * (g_roll + gyro_y_dps * dt) + 0.04 * acc_roll;

      //     printf("{\"pitch\": %.2f, \"roll\": %.2f}\n", g_pitch, g_roll);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vTaskConsole(void *pvParameters) {
    while(1) {
        printf("\n========================================\n");
        printf("JOYSTICK : X=%d | Y=%d | BOTAO=%d\n", g_joy_x, g_joy_y, g_btn);
        printf("SERVO X  : %d Duty | PITCH = %.1f graus\n", g_servo_x, g_pitch);
        printf("SERVO Y  : %d Duty | ROLL  = %.1f graus\n\n", g_servo_y, g_roll);
        printf("IMU ACCEL (g): X: %.2f | Y: %.2f | Z: %.2f\n", g_acc_x_g, g_acc_y_g, g_acc_z_g);
        printf("=====================================================\n");
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

void app_main() {
    printf("INICIALIZANDO HARDWARE...\n");
    
    gpio_reset_pin(LED_STATUS_PIN); 
    gpio_set_direction(LED_STATUS_PIN, GPIO_MODE_OUTPUT); 
    gpio_set_level(LED_STATUS_PIN, 0); 
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOYSTICK_SEL_PIN), 
        .mode = GPIO_MODE_INPUT,                    
        .pull_up_en = GPIO_PULLUP_ENABLE,           
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      
        .intr_type = GPIO_INTR_DISABLE              
    };
    gpio_config(&io_conf); 
    
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT, 
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12, 
        .atten = ADC_ATTEN_DB_12,    
    };
    adc_oneshot_config_channel(adc_handle, JOYSTICK_X_CH, &config);
    adc_oneshot_config_channel(adc_handle, JOYSTICK_Y_CH, &config);
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,  
        .timer_num = LEDC_TIMER_0,          
        .duty_resolution = LEDC_TIMER_12_BIT, 
        .freq_hz = 50,                        
        .clk_cfg = LEDC_AUTO_CLK            
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_ch_x = { 
        .speed_mode = LEDC_LOW_SPEED_MODE, 
        .channel = LEDC_CHANNEL_0,       
        .timer_sel = LEDC_TIMER_0, 
        .intr_type = LEDC_INTR_DISABLE, 
        .gpio_num = SERVO_X_PIN, 
        .duty = 307, 
        .hpoint = 0 
    };
    ledc_channel_config(&ledc_ch_x);
    
    ledc_channel_config_t ledc_ch_y = { 
        .speed_mode = LEDC_LOW_SPEED_MODE, 
        .channel = LEDC_CHANNEL_1,       
        .timer_sel = LEDC_TIMER_0, 
        .intr_type = LEDC_INTR_DISABLE, 
        .gpio_num = SERVO_Y_PIN, 
        .duty = 307, 
        .hpoint = 0 
    };
    ledc_channel_config(&ledc_ch_y);
    
    if (i2c_master_init() == ESP_OK) {
        mpu9250_init();
    }

    gpio_set_level(LED_STATUS_PIN, 1);
    printf("HARDWARE PRONTO!\n");
    
    xJoystickQueue = xQueueCreate(1, sizeof(joystick_data_t));
    
    if (xJoystickQueue != NULL) {
        xTaskCreate(vTaskJoystick, "T_Joy", 2048, NULL, 3, NULL); 
        xTaskCreate(vTaskServos,   "T_Srv", 2048, NULL, 2, NULL); 
        xTaskCreate(vTaskMPU9250,  "T_MPU", 3072, NULL, 2, NULL); 
        xTaskCreate(vTaskConsole,  "T_Log", 3072, NULL, 1, NULL); 
    }
}