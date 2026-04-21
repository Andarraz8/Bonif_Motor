#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

// --- Definición de Pines ---
#define PIN_DIG1 4
#define PIN_DIG2 16
#define PIN_DIG3 17

#define SEG_A 1
#define SEG_B 23
#define SEG_C 19
#define SEG_D 21
#define SEG_E 3
#define SEG_F 22
#define SEG_G 18

#define INDICADOR_ROJO  33
#define INDICADOR_VERDE 32
#define SALIDA_IZQ      12
#define SALIDA_DER      27

#define BTN_IZQUIERDO   13
#define BTN_DERECHO     14

#define CANAL_ADC       ADC_CHANNEL_6
#define PIN_PWM         26

// --- Constantes y Variables Globales ---
#define PERIODO_MUESTREO_US 20000
#define TIEMPO_DEBOUNCE_US  200000
#define PWM_MAX_DUTY        4095 

static volatile int digitos_display[3] = {0, 0, 0};
static volatile int sentido_giro = 0; 

static volatile int64_t ultimo_pulso_izq = 0;
static volatile int64_t ultimo_pulso_der = 0;

const uint8_t mapa_numeros[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

void actualizar_segmentos(uint8_t patron) {
    gpio_set_level(SEG_A, !((patron >> 0) & 1));
    gpio_set_level(SEG_B, !((patron >> 1) & 1));
    gpio_set_level(SEG_C, !((patron >> 2) & 1));
    gpio_set_level(SEG_D, !((patron >> 3) & 1));
    gpio_set_level(SEG_E, !((patron >> 4) & 1));
    gpio_set_level(SEG_F, !((patron >> 5) & 1));
    gpio_set_level(SEG_G, !((patron >> 6) & 1));
}

void refrescar_panel(int d1, int d2, int d3) {
    int pines_digitos[] = {PIN_DIG1, PIN_DIG2, PIN_DIG3};
    int valores[] = {d1, d2, d3};

    for (int i = 0; i < 3; i++) {
        gpio_set_level(PIN_DIG1, 1);
        gpio_set_level(PIN_DIG2, 1);
        gpio_set_level(PIN_DIG3, 1);

        actualizar_segmentos(mapa_numeros[valores[i]]);
        
        gpio_set_level(pines_digitos[i], 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// --- Manejadores de Interrupción (ISR) ---

static void IRAM_ATTR isr_boton_izquierdo(void *arg) {
    int64_t tiempo_actual = esp_timer_get_time();
    if (tiempo_actual - ultimo_pulso_izq > TIEMPO_DEBOUNCE_US) {
        sentido_giro = 1;
        ultimo_pulso_izq = tiempo_actual;
    }
}

static void IRAM_ATTR isr_boton_derecho(void *arg) {
    int64_t tiempo_actual = esp_timer_get_time();
    if (tiempo_actual - ultimo_pulso_der > TIEMPO_DEBOUNCE_US) {
        sentido_giro = 2;
        ultimo_pulso_der = tiempo_actual;
    }
}

// --- Configuración de Periféricos ---

void configurar_sistema() {
    uint64_t mascara_salidas = (
        (1ULL << SEG_A) | (1ULL << SEG_B) | (1ULL << SEG_C) | (1ULL << SEG_D) |
        (1ULL << SEG_E) | (1ULL << SEG_F) | (1ULL << SEG_G) |
        (1ULL << PIN_DIG1) | (1ULL << PIN_DIG2) | (1ULL << PIN_DIG3) |
        (1ULL << INDICADOR_ROJO) | (1ULL << INDICADOR_VERDE) |
        (1ULL << SALIDA_IZQ) | (1ULL << SALIDA_DER)
    );

    gpio_config_t cfg_salidas = {
        .pin_bit_mask = mascara_salidas,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg_salidas);

    gpio_config_t cfg_entradas = {
        .pin_bit_mask = (1ULL << BTN_IZQUIERDO | 1ULL << BTN_DERECHO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&cfg_entradas);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_IZQUIERDO, isr_boton_izquierdo, NULL);
    gpio_isr_handler_add(BTN_DERECHO, isr_boton_derecho, NULL);

    timer_config_t cfg_timer = {
        .divider = 80, .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE, .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = false,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &cfg_timer);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void app_main() {
    configurar_sistema();

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_cfg_adc = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_cfg_adc, &adc_handle);

    adc_oneshot_chan_cfg_t cfg_canal_adc = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc_handle, CANAL_ADC, &cfg_canal_adc);

    ledc_timer_config_t timer_pwm = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_pwm);

    ledc_channel_config_t canal_pwm = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_PWM, .duty = PWM_MAX_DUTY, .hpoint = 0 
    };
    ledc_channel_config(&canal_pwm);

    gpio_set_level(INDICADOR_ROJO, 1);
    gpio_set_level(SALIDA_IZQ, 1);

    int lectura_cruda;
    uint64_t cuenta_timer = 0;

    while (1) {
        refrescar_panel(digitos_display[0], digitos_display[1], digitos_display[2]);

        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &cuenta_timer);

        if (cuenta_timer >= PERIODO_MUESTREO_US) {
            adc_oneshot_read(adc_handle, CANAL_ADC, &lectura_cruda);
            
            int porcentaje = (lectura_cruda * 100) / 4095;
            
            digitos_display[0] = porcentaje / 100;
            digitos_display[1] = (porcentaje / 10) % 10;
            digitos_display[2] = porcentaje % 10;

            int duty_invertido = PWM_MAX_DUTY - ((porcentaje * PWM_MAX_DUTY) / 100);
            
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_invertido);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

            timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
        }

        if (sentido_giro != 0) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_MAX_DUTY);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(500));

            if (sentido_giro == 1) { 
                gpio_set_level(INDICADOR_ROJO, 1);
                gpio_set_level(INDICADOR_VERDE, 0);
                gpio_set_level(SALIDA_IZQ, 1);
                gpio_set_level(SALIDA_DER, 0);
            } 
            else if (sentido_giro == 2) { 
                gpio_set_level(INDICADOR_ROJO, 0);
                gpio_set_level(INDICADOR_VERDE, 1);
                gpio_set_level(SALIDA_IZQ, 0);
                gpio_set_level(SALIDA_DER, 1);
            }
            sentido_giro = 0; 
        }
    }
}
