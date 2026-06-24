/****************************************************************=================
 * IFPB - INSTITUTO FEDERAL DE EDUCAÇÃO, CIÊNCIA E TECNOLOGIA DA PARAÍBA
 * CAMPUS CAMPINA GRANDE - DEPARTAMENTO DE ENGENHARIA / SISTEMAS EMBARCADOS
 * * DISCIPLINA: Sistemas Embarcados II               SEMESTRE LETIVO: 2026.1
 * DOCENTE:    Alexandre Sales Vasconcelos
 * DISCENTES:  Andreza Santos, Lavoisier Ramos, Nivaldo Neto, Vinícius Barbosa
 * * PROJETO:    Mesa Labirinto Interativa com Gêmeo Digital (Grafana)
 * HARDWARE:   ESP32-S3-WROOM-1U
 * * DESCRIÇÃO DO SISTEMA:
 * Este firmware implementa o controle concorrente de uma mesa labirinto utilizando
 * o RTOS (FreeRTOS) integrado ao ecossistema da Espressif (ESP-IDF). O sistema:
 * 1. Amostra e filtra leituras analógicas de um Joystick de 2 eixos (ADC1).
 * 2. Controla de forma proporcional dois servomotores SG90 utilizando o periférico
 * LEDC (PWM) com algoritmo de suavização por média móvel exponencial (filtro alfa).
 * 3. Comunica-se com um sensor inercial MPU6050 (módulo JY62) via barramento I2C para
 * ler dados brutos de aceleração e rotação, aplicando um Filtro Complementar
 * para extrair os ângulos reais de inclinação (Pitch e Roll).
 * 4. Transmite a telemetria via console Serial para alimentação do Gêmeo Digital.
 *********************************************************************************/

#include <stdio.h>                     // Biblioteca padrão de entrada/saída (printf).
#include <stdlib.h>                    // Biblioteca padrão C para utilitários (função abs).
#include <math.h>                      // Biblioteca matemática para cálculo inercial (atan2, sqrt, M_PI).
#include "freertos/FreeRTOS.h"         // Definições básicas do kernel do FreeRTOS.
#include "freertos/task.h"             // API de gerenciamento de tarefas/tasks concorrentes.
#include "freertos/queue.h"            // API de filas para comunicação segura entre tasks.
#include "driver/gpio.h"               // Driver de controle das portas de entrada/saída digitais (GPIOs).
#include "esp_adc/adc_oneshot.h"       // Novo driver da Espressif para leitura analógica estável (ADC).
#include "driver/ledc.h"               // Driver do periférico LED Control, usado para gerar sinais PWM para servos.
#include "driver/i2c.h"                // Driver de comunicação do barramento síncrono I2C.

/* === DEFINIÇÃO DE PINAGEM CORRIGIDA PARA ESP32-S3 === */
#define LED_STATUS_PIN     2           // GPIO 2: Saída para o LED indicador de inicialização concluída.
#define JOYSTICK_X_CH      ADC_CHANNEL_3 // GPIO 4: Canal analógico ADC1_CH3 para ler o Eixo X do Joystick.
#define JOYSTICK_Y_CH      ADC_CHANNEL_4 // GPIO 5: Canal analógico ADC1_CH4 para ler o Eixo Y do Joystick.
#define JOYSTICK_SEL_PIN   6           // GPIO 6: Entrada digital com Pull-Up para o botão interno do Joystick.

#define SERVO_X_PIN        18          // GPIO 18: Saída PWM do periférico LEDC para controle do Servo X.
#define SERVO_Y_PIN        17          // GPIO 17: Saída PWM do periférico LEDC para controle do Servo Y.

#define I2C_MASTER_SDA_IO  8           // GPIO 8: Linha de dados (SDA) do barramento I2C0 nativo do S3.
#define I2C_MASTER_SCL_IO  9           // GPIO 9: Linha de clock (SCL) do barramento I2C0 nativo do S3.
#define I2C_MASTER_NUM     I2C_NUM_0   // Define o uso do bloco de hardware interno I2C número 0 do chip.
#define I2C_MASTER_FREQ_HZ 400000      // Frequência de operação do I2C a 400kHz (Modo Fast Mode).
#define MPU9250_ADDR       0x68        // Endereço físico padrão de barramento do chip sensor MPU6050/9250.

// Estrutura de dados que empacota as leituras do joystick para envio via fila (Queue)
typedef struct {
    int x_raw;                         // Armazena a leitura digitalizada do eixo X (0 a 4095).
    int y_raw;                         // Armazena a leitura digitalizada do eixo Y (0 a 4095).
    int btn_pressed;                   // Armazena o estado lógico do botão (0=Solto, 1=Pressionado).
} joystick_data_t;

QueueHandle_t xJoystickQueue;          // Variável manipuladora (handle) da fila de comunicação do joystick.

// Variáveis globais para compartilhamento de status com a task de exibição no console
int g_joy_x = 2048, g_joy_y = 2048, g_btn = 0; // Armazenam estados do joystick (iniciam no centro).
int g_servo_x = 307, g_servo_y = 307;          // Valores de Duty Cycle iniciais dos Servos (equivalente a 90°).
float g_pitch = 0.0, g_roll = 0.0;             // Ângulos finais filtrados de orientação da mesa labirinto.
float g_acc_x_g = 0.0, g_acc_y_g = 0.0, g_acc_z_g = 0.0; // Acelerações lineares convertidas em força G.

adc_oneshot_unit_handle_t adc_handle;  // Objeto manipulador para a unidade ADC1 configurada em modo One-Shot.

// Função que configura os parâmetros elétricos e inicializa o barramento mestre I2C
esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // Configura o ESP32-S3 como mestre da comunicação.
        .sda_io_num = I2C_MASTER_SDA_IO,       // Vincula o pino físico de dados SDA.
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Ativa resistores de pull-up internos para estabilizar a linha.
        .scl_io_num = I2C_MASTER_SCL_IO,       // Vincula o pino físico de clock SCL.
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Ativa resistores de pull-up internos para o clock.
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Define a velocidade de transmissão de dados.
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);   // Aplica as estruturas de configuração no bloco I2C_NUM_0.
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); // Aloca os recursos de software do driver no sistema.
}

// Função para escrever um único byte de dados em um registrador específico do MPU6050
esp_err_t mpu9250_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};   // Vetor contendo o endereço destino e o valor a ser gravado.
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100)); // Executa a escrita I2C com timeout de 100ms.
}

// Função para ler um bloco consecutivo de registradores mapeados no hardware do MPU6050
esp_err_t mpu9250_read_regs(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(100)); // Envia o registrador de partida e lê "len" bytes em sequência.
}

// Inicialização de nível de registro para o sensor inercial
void mpu9250_init(void) {
    mpu9250_write_reg(0x6B, 0x00); // Escreve 0 no registrador PWR_MGMT_1 para tirar o MPU do modo de hibernação (Sleep Mode).
}

// Task encarregada por amostrar, filtrar e despachar as informações do Joystick
void vTaskJoystick(void *pvParameters) {
    joystick_data_t data;              // Instancia uma estrutura local para organizar os dados amostrados.
    while(1) {                         // Loop infinito/operacional da tarefa do FreeRTOS.
        int sum_x = 0, sum_y = 0;      // Acumuladores locais para o cálculo da média aritmética.
        const int samples = 4;         // Define a amostragem por oversampling de 4 leituras por ciclo.

        for(int i = 0; i < samples; i++) {
            int temp_x, temp_y;
            adc_oneshot_read(adc_handle, JOYSTICK_X_CH, &temp_x); // Captura o valor instantâneo do canal X.
            adc_oneshot_read(adc_handle, JOYSTICK_Y_CH, &temp_y); // Captura o valor instantâneo do canal Y.
            sum_x += temp_x;           // Acumula o valor lido do eixo X.
            sum_y += temp_y;           // Acumula o valor lido do eixo Y.
            vTaskDelay(pdMS_TO_TICKS(2)); // Bloqueia por 2 milissegundos para espaçar as amostras do conversor.
        }

        data.x_raw = sum_x / samples;  // Divide a soma total obtendo o valor médio filtrado para X.
        data.y_raw = sum_y / samples;  // Divide a soma total obtendo o valor médio filtrado para Y.
        data.btn_pressed = !gpio_get_level(JOYSTICK_SEL_PIN); // Lê o botão físico (Inverte o nível lógico pois atua em nível baixo / Pull-Up).
        
        g_joy_x = data.x_raw;          // Atualiza a variável de escopo global com o valor de X estável.
        g_joy_y = data.y_raw;          // Atualiza a variável de escopo global com o valor de Y estável.
        g_btn = data.btn_pressed;      // Atualiza a variável de escopo global com o estado atual do botão.
        
        xQueueOverwrite(xJoystickQueue, &data); // Sobrescreve a estrutura na fila com os dados novos, sem travar a execução.
        vTaskDelay(pdMS_TO_TICKS(30)); // Aguarda 30 milissegundos antes do início do próximo ciclo de varredura.
    }
}

// Task responsável pelo tratamento matemático e acionamento dinâmico dos servos
void vTaskServos(void *pvParameters) {
    joystick_data_t rx;                // Cria estrutura local para receber o pacote vindo da fila.
    float s_x = 307.0, s_y = 307.0;    // Inicializa valores internos de controle em ponto flutuante para alta precisão.
    
    const int CENTRO_ADC = 2048;       // Ponto central ideal do conversor de 12 bits (4095 / 2).
    const int ZONA_MORTA = 150;        // Margem de tolerância central para ignorar variações mecânicas em repouso.
    const float ALPHA = 0.06;          // Fator ponderador do filtro de média móvel exponencial (menor = mais suave).

    while(1) {
        // Aguarda indefinidamente (portMAX_DELAY) até que um novo dado seja injetado na fila pela TaskJoystick
        if(xQueueReceive(xJoystickQueue, &rx, portMAX_DELAY)) {
            int process_x = rx.x_raw;  // Copia o valor bruto de X recebido da fila.
            int process_y = rx.y_raw;  // Copia o valor bruto de Y recebido da fila.

            // Aplica filtro de Zona Morta: se estiver dentro do raio central, força o valor para o centro estável.
            if (abs(process_x - CENTRO_ADC) < ZONA_MORTA) process_x = CENTRO_ADC;
            if (abs(process_y - CENTRO_ADC) < ZONA_MORTA) process_y = CENTRO_ADC;

            // Mapeamento linear: Converte a escala do ADC (0-4095) para a escala de largura de pulso do Servo (102 a 512 do timer)
            int tx = 102 + ((process_x * (512 - 102)) / 4095); // Equivalente à faixa de 1ms a 2ms de pulso do servo.
            int ty = 102 + ((process_y * (512 - 102)) / 4095); // Converte de forma otimizada usando aritmética inteira.
            
            // Aplicação do Filtro Passa-Baixas Exponencial (EMA) para suavizar movimentos bruscos da mesa
            s_x = (s_x * (1.0 - ALPHA)) + (tx * ALPHA); // Combina a posição anterior com o novo alvo proporcionalmente.
            s_y = (s_y * (1.0 - ALPHA)) + (ty * ALPHA); // Reduz solavancos e vibrações estruturais na maquete.
            
            g_servo_x = (int)s_x;      // Converte o resultado estabilizado float de X de volta para inteiro.
            g_servo_y = (int)s_y;      // Converte o resultado estabilizado float de Y de volta para inteiro.
            
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, g_servo_x); // Altera o registrador interno de largura de pulso do canal 0.
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);       // Aplica imediatamente a nova configuração de hardware no Servo X.
            
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g_servo_y); // Altera o registrador interno de largura de pulso do canal 1.
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);       // Aplica imediatamente a nova configuração de hardware no Servo Y.
        }
    }
}

// Task de sensoriamento inercial e fusão de dados sensoriais (Filtro Complementar)
void vTaskMPU9250(void *pvParameters) {
    uint8_t raw_data[14];              // Buffer local para leitura sequencial dos 14 registros de dados do sensor.
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y; // Variáveis de 16 bits com sinal para acomodar inteiros em complemento de dois.
    float acc_pitch, acc_roll, gyro_x_dps, gyro_y_dps; // Variáveis auxiliares de ponto flutuante para a física do sistema.
    const float dt = 0.020;            // Tempo discreto de amostragem constante da tarefa (20ms = 0.02s).

    while(1) {
        // Realiza a leitura atômica via I2C começando no registrador 0x3B (ACCEL_XOUT_H)
        if (mpu9250_read_regs(0x3B, raw_data, 14) == ESP_OK) {
            // Reconstrói as leituras de 16 bits juntando o byte mais significativo (High) e menos significativo (Low)
            accel_x = (raw_data[0] << 8) | raw_data[1];   // Aceleração no eixo X do encapsulamento.
            accel_y = (raw_data[2] << 8) | raw_data[3];   // Aceleração no eixo Y do encapsulamento.
            accel_z = (raw_data[4] << 8) | raw_data[5];   // Aceleração no eixo Z do encapsulamento.
            gyro_x  = (raw_data[8] << 8) | raw_data[9];   // Velocidade angular ao redor do eixo X.
            gyro_y  = (raw_data[10] << 8) | raw_data[11]; // Velocidade angular ao redor do eixo Y.

            // Escalonamento do Giroscópio: Converte o valor bruto para Graus por Segundo (dps) usando fundo de escala de +/-250dps
            gyro_x_dps = (float)gyro_x / 131.0;
            gyro_y_dps = (float)gyro_y / 131.0;

            // Escalonamento do Acelerômetro: Converte leituras brutas em unidades de aceleração gravitacional (g) usando escala de +/-2g
            g_acc_x_g = (float)accel_x / 16384.0;
            g_acc_y_g = (float)accel_y / 16384.0;
            g_acc_z_g = (float)accel_z / 16384.0;

            // Cálculo dos ângulos de inclinação puramente geométricos através dos vetores estáticos da gravidade
            acc_pitch = atan2((float)-accel_x, sqrt((float)accel_y * accel_y + (float)accel_z * accel_z)) * 180.0 / M_PI; // Trigonometria do arco tangente para Pitch.
            acc_roll  = atan2((float)accel_y, (float)accel_z) * 180.0 / M_PI;                                            // Trigonometria do arco tangente para Roll.

            // Algoritmo de Filtro Complementar: Protege o sistema contra ruído de vibração e desvio integrativo (Drift)
            g_pitch = 0.96 * (g_pitch + gyro_x_dps * dt) + 0.04 * acc_pitch; // Confia 96% na integração estável do Giroscópio e 4% no Acelerômetro.
            g_roll  = 0.96 * (g_roll + gyro_y_dps * dt) + 0.04 * acc_roll;   // Garante precisão rápida e imunidade a ruídos mecânicos dos servos.

            // Linha comentada de telemetria JSON para acoplamento do Grafana
            // printf("{\"pitch\": %.2f, \"roll\": %.2f}\n", g_pitch, g_roll);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Força a suspensão da tarefa por 20ms garantindo o sincronismo da taxa discreta "dt".
    }
}

// Task de depuração e geração de logs no formato humano para análise de engenharia
void vTaskConsole(void *pvParameters) {
    while(1) {
        printf("\n========================================\n");
        printf("JOYSTICK : X=%d | Y=%d | BOTAO=%d\n", g_joy_x, g_joy_y, g_btn); // Exibe estados absolutos da interface de controle.
        printf("SERVO X  : %d Duty | PITCH = %.1f graus\n", g_servo_x, g_pitch); // Exibe os níveis de atuação versus inclinação real de Pitch.
        printf("SERVO Y  : %d Duty | ROLL  = %.1f graus\n\n", g_servo_y, g_roll); // Exibe os níveis de atuação versus inclinação real de Roll.
        printf("IMU ACCEL (g): X: %.2f | Y: %.2f | Z: %.2f\n", g_acc_x_g, g_acc_y_g, g_acc_z_g); // Exibe a força estática/dinâmica lida nos eixos lineares.
        printf("=====================================================\n");
        vTaskDelay(pdMS_TO_TICKS(500)); // Suspende a tarefa por 500 milissegundos para não saturar a transmissão serial do console.
    }
}

// Ponto de entrada padrão do sistema operacional do microcontrolador
void app_main() {
    printf("INICIALIZANDO HARDWARE...\n"); // Log básico de inicialização precoce de hardware.
    
    /* Configuração inicial básica do LED de Status */
    gpio_reset_pin(LED_STATUS_PIN);                       // Limpa qualquer configuração residual de inicialização elétrica no pino 2.
    gpio_set_direction(LED_STATUS_PIN, GPIO_MODE_OUTPUT); // Define eletronicamente a GPIO 2 como uma saída digital.
    gpio_set_level(LED_STATUS_PIN, 0);                    // Mantém o LED desligado (Nível Lógico Baixo) durante os testes de periféricos.
    
    /* Configuração via máscara da porta do Botão do Joystick */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOYSTICK_SEL_PIN),       // Associa o pino físico GPIO 6 na máscara de configuração do driver.
        .mode = GPIO_MODE_INPUT,                          // Declara a GPIO 6 operando em regime de entrada digital.
        .pull_up_en = GPIO_PULLUP_ENABLE,                  // Aciona o resistor pull-up interno para fixar o nível alto estável (3.3V).
        .pull_down_en = GPIO_PULLDOWN_DISABLE,            // Desativa explicitamente o resistor de pull-down.
        .intr_type = GPIO_INTR_DISABLE                    // Sinaliza que este pino não vai disparar eventos de interrupção de hardware.
    };
    gpio_config(&io_conf);                                // Consolida e grava as configurações nos registros do microcontrolador.
    
    /* Inicialização do subsistema de Conversão Analógica-Digital (ADC) */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,                            // Configura o uso exclusivo do bloco integrado de hardware ADC1.
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,              // Associa o clock padrão interno automático fornecido pelo silício.
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);      // Aloca em memória a estrutura de gerenciamento da nova unidade analógica.
    
    /* Configuração detalhada do canal de resolução dos eixos analógicos */
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,                      // Define amostragem em resolução fina de 12 bits (Faixa de leitura de 0 a 4095).
        .atten = ADC_ATTEN_DB_12,                         // Define a atenuação de 12dB para permitir leitura na faixa de tensão plena de 0 a 3.3V.
    };
    adc_oneshot_config_channel(adc_handle, JOYSTICK_X_CH, &config); // Grava a configuração no canal correspondente ao Eixo X (GPIO 4).
    adc_oneshot_config_channel(adc_handle, JOYSTICK_Y_CH, &config); // Grava a configuração no canal correspondente ao Eixo Y (GPIO 5).
    
    /* Configuração do Timer do periférico gerador de modulação por largura de pulso (LEDC / PWM) */
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,                // Define o modo de operação de baixa velocidade para controle estável de servos.
        .timer_num = LEDC_TIMER_0,                        // Associa as configurações ao Bloco de Hardware do Temporizador Número 0.
        .duty_resolution = LEDC_TIMER_12_BIT,             // Determina resolução fina de 12 bits (Modula a largura do pulso em valores de 0 a 4095).
        .freq_hz = 50,                                    // Configura a frequência padrão mundial para servomotores analógicos em 50Hz (período estável de 20ms).
        .clk_cfg = LEDC_AUTO_CLK                          // Define seleção de clock automática gerenciada pelo clock do barramento do sistema.
    };
    ledc_timer_config(&ledc_timer);                       // Realiza a gravação física dos registradores do temporizador PWM.
    
    /* Parametrização estrutural e alocação eletrônica do canal do Servo X */
    ledc_channel_config_t ledc_ch_x = { 
        .speed_mode = LEDC_LOW_SPEED_MODE,                // Modos de velocidade síncronos com o timer do motor.
        .channel = LEDC_CHANNEL_0,                        // Associa a estrutura de software ao canal lógico de controle 0.
        .timer_sel = LEDC_TIMER_0,                        // Vincula este canal ao temporizador 0 configurado previamente em 50Hz.
        .intr_type = LEDC_INTR_DISABLE,                   // Sinaliza que modificações não gerarão rotinas de interrupção ISR.
        .gpio_num = SERVO_X_PIN,                          // Define a GPIO 18 como saída física direta dos pulsos do Servo X.
        .duty = 307,                                      // Inicia o motor posicionado em ponto neutro central (90° -> 1.5ms de pulso ou ~7.5% de 4095).
        .hpoint = 0                                       // Define ponto de partida da contagem do ciclo de trabalho zerado.
    };
    ledc_channel_config(&ledc_ch_x);                      // Ativa o hardware do canal PWM do Servo X.
    
    /* Parametrização estrutural e alocação eletrônica do canal do Servo Y */
    ledc_channel_config_t ledc_ch_y = { 
        .speed_mode = LEDC_LOW_SPEED_MODE,                // Modos de velocidade síncronos com o timer do motor.
        .channel = LEDC_CHANNEL_1,                        // Associa a estrutura de software ao canal lógico de controle 1.
        .timer_sel = LEDC_TIMER_0,                        // Vincula este canal ao mesmo temporizador 0 de 50Hz.
        .intr_type = LEDC_INTR_DISABLE,                   // Sinaliza que modificações não gerarão rotinas de interrupção ISR.
        .gpio_num = SERVO_Y_PIN,                          // Define a GPIO 17 como saída física direta dos pulsos do Servo Y.
        .duty = 307,                                      // Inicia o motor posicionado em ponto neutro central (90° -> 1.5ms de pulso ou ~7.5% de 4095).
        .hpoint = 0                                       // Define ponto de partida da contagem do ciclo de trabalho zerado.
    };
    ledc_channel_config(&ledc_ch_y);                      // Ativa o hardware do canal PWM do Servo Y.
    
    /* Teste condicional e inicialização lógica do Sensor via I2C */
    if (i2c_master_init() == ESP_OK) {                     // Tenta configurar os recursos de hardware do mestre I2C.
        mpu9250_init();                                   // Se o barramento foi aberto com sucesso, executa a rotina de inicialização do sensor MPU6050.
    }

    gpio_set_level(LED_STATUS_PIN, 1);                    // Altera o estado lógico da GPIO 2 para nível Alto, acendendo o LED de Inicialização Pronta.
    printf("HARDWARE PRONTO!\n");                         // Envia string de controle sinalizando fim de inicializações de registradores.
    
    // Inicializa a fila de controle (Queue) para reter 1 pacote por vez, agindo como barreira de dados estável
    xJoystickQueue = xQueueCreate(1, sizeof(joystick_data_t));
    
    // Valida se a alocação da fila em memória estática/dinâmica do kernel do RTOS obteve sucesso
    if (xJoystickQueue != NULL) {
        // Dispara concorrentemente no escalonador (scheduler) as quatro tarefas do sistema distribuindo prioridades
        xTaskCreate(vTaskJoystick, "T_Joy", 2048, NULL, 3, NULL); // Prioridade 3 (Mais alta): Amostragem rápida e sem atrasos do controle humano.
        xTaskCreate(vTaskServos,   "T_Srv", 2048, NULL, 2, NULL); // Prioridade 2 (Média): Resposta dinâmica e suavização mecânica dos motores.
        xTaskCreate(vTaskMPU9250,  "T_MPU", 3072, NULL, 2, NULL); // Prioridade 2 (Média): Monitoramento e cálculo de estabilização inercial por trigonometria.
        xTaskCreate(vTaskConsole,  "T_Log", 3072, NULL, 1, NULL); // Prioridade 1 (Mais baixa): Transmissão serial assíncrona humana de depuração.
    }
}