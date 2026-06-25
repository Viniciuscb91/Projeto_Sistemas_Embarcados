idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES driver esp_adc freertos) # Força o CMake a buscar o FreeRTOS