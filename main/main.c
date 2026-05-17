#include <stdio.h>
#include "driver/gpio.h"
#include "soc/clk_tree_defs.h"
#include "freertos/FreeRTOS.h"

#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"

static const char *TAG = "sugi";

#define PIN_STEP_IN1 19
#define PIN_STEP_IN2 21
#define PIN_STEP_IN3 22
#define PIN_STEP_IN4 23

void setup_gpio()
{

    gpio_config_t motor_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = (1ULL << PIN_STEP_IN1) | (1ULL << PIN_STEP_IN2) | (1ULL << PIN_STEP_IN3) | (1ULL << PIN_STEP_IN4),

    };
    gpio_config(&motor_conf);
}

void motorFullStep()
{
    static uint8_t state = 0;
    switch (state)
    {
    case 0:
        gpio_set_level(PIN_STEP_IN1, 1);
        gpio_set_level(PIN_STEP_IN2, 0);
        gpio_set_level(PIN_STEP_IN3, 0);
        gpio_set_level(PIN_STEP_IN4, 0);

        state = 1;
        break;
    case 1:

        gpio_set_level(PIN_STEP_IN1, 0);
        gpio_set_level(PIN_STEP_IN2, 1);
        gpio_set_level(PIN_STEP_IN3, 0);
        gpio_set_level(PIN_STEP_IN4, 0);

        state = 2;
        break;
    case 2:

        gpio_set_level(PIN_STEP_IN1, 0);
        gpio_set_level(PIN_STEP_IN2, 0);
        gpio_set_level(PIN_STEP_IN3, 1);
        gpio_set_level(PIN_STEP_IN4, 0);

        state = 3;
        break;
    case 3:

        gpio_set_level(PIN_STEP_IN1, 0);
        gpio_set_level(PIN_STEP_IN2, 0);
        gpio_set_level(PIN_STEP_IN3, 0);
        gpio_set_level(PIN_STEP_IN4, 1);

        state = 0;
        break;
    default:
        gpio_set_level(PIN_STEP_IN1, 0);
        gpio_set_level(PIN_STEP_IN2, 0);
        gpio_set_level(PIN_STEP_IN3, 0);
        gpio_set_level(PIN_STEP_IN4, 0);
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}
void taskStepMotor()
{

    xTaskCreate()

    for (int i = 0; i < 2048; i++)
    {
        motorFullStep();
        // ESP_LOGI(TAG, "That one has happend at %d", i);
    }
    gpio_set_level(PIN_STEP_IN1, 0);
    gpio_set_level(PIN_STEP_IN2, 0);
    gpio_set_level(PIN_STEP_IN3, 0);
    gpio_set_level(PIN_STEP_IN4, 0);
}
void app_main(void)
{
    setup_gpio();
    for (int i = 0; i < 2048; i++)
    {
        motorFullStep();
        // ESP_LOGI(TAG, "That one has happend at %d", i);
    }
    gpio_set_level(PIN_STEP_IN1, 0);
    gpio_set_level(PIN_STEP_IN2, 0);
    gpio_set_level(PIN_STEP_IN3, 0);
    gpio_set_level(PIN_STEP_IN4, 0);
}
