#include <stdio.h>
#include "driver/gpio.h"
#include "soc/clk_tree_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_intr_alloc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "dht.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "sugi";

#define PIN_FAN_A 23
#define PIN_FAN_B 22
#define PWM_FREQ_HZ 20000
#define PWM_RESOLUTION_HZ 20000000
#define PWM_PERIOD_TICKS (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)

#define PIN_BUZZER 21
#define PIN_DHT11 4
#define PIN_RELAY 5
#define PIN_MOISTER_SENSOR 34
#define MOISTER_AO_CHANNEL ADC_CHANNEL_6

#define PIN_PWM_MOTOR 32
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE -90
#define SERVO_MAX_DEGREE 90

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD 20000

static mcpwm_cmpr_handle_t comp_motor = NULL;
static mcpwm_oper_handle_t oper_motor = NULL;
static mcpwm_gen_handle_t gen_motor = NULL;

mcpwm_gen_handle_t fan_generator = NULL;
mcpwm_cmpr_handle_t fan_comparator = NULL;
mcpwm_oper_handle_t fan_oper = NULL;
mcpwm_timer_handle_t fan_timer = NULL;

adc_oneshot_unit_handle_t adc_moist_handle;

const int dry_value = 4095;
const int wet_value = 1480;

typedef struct
{
    float temperature;
    int moisture;
} SystemState_t;

SystemState_t sys_state = {0.0, 0};
SemaphoreHandle_t state_mutex = NULL;

float target_temp = 25.0;
int target_moisture = 10;
float temp_tolerance = 2.0;

int map_value(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline uint32_t angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void moveServo(int angle)
{
    mcpwm_comparator_set_compare_value(comp_motor, angle_to_compare(angle));
}

void setFanSpeed(int n)
{
    mcpwm_comparator_set_compare_value(fan_comparator, n * 10);
}

void setup_gpio()
{
    gpio_config_t motor_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = (1ULL << PIN_RELAY) | (1ULL << PIN_BUZZER),
    };
    gpio_config(&motor_conf);
    gpio_set_level(PIN_RELAY, 0);
    gpio_set_level(PIN_BUZZER, 0);
}

void servo_conf()
{
    mcpwm_timer_config_t conf_timer_motor = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_timer_handle_t timer_motor = NULL;
    ESP_ERROR_CHECK(mcpwm_new_timer(&conf_timer_motor, &timer_motor));

    mcpwm_operator_config_t conf_oper = {
        .group_id = 0,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&conf_oper, &oper_motor));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_motor, timer_motor));

    mcpwm_comparator_config_t conf_comparato = {
        .intr_priority = 0,
        .flags.update_cmp_on_tez = 1,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_motor, &conf_comparato, &comp_motor));

    mcpwm_generator_config_t conf_gen = {
        .gen_gpio_num = PIN_PWM_MOTOR,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_motor, &conf_gen, &gen_motor));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comp_motor, angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_motor, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_motor, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp_motor, MCPWM_GEN_ACTION_LOW)));

    mcpwm_timer_enable(timer_motor);
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_motor, MCPWM_TIMER_START_NO_STOP));
}

void soil_conf()
{
    adc_oneshot_unit_init_cfg_t moist_init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&moist_init_config, &adc_moist_handle));

    adc_oneshot_chan_cfg_t moister_adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_moist_handle, MOISTER_AO_CHANNEL, &moister_adc_config));
}

void fan_setup()
{
    gpio_reset_pin(PIN_FAN_B);
    gpio_set_direction(PIN_FAN_B, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_FAN_B, 0);

    mcpwm_timer_config_t timer_config = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks = PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &fan_timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 1,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &fan_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(fan_oper, fan_timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(fan_oper, &comparator_config, &fan_comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = PIN_FAN_A,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(fan_oper, &generator_config, &fan_generator));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(fan_generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(fan_generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, fan_comparator, MCPWM_GEN_ACTION_LOW)));

    mcpwm_timer_enable(fan_timer);
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(fan_timer, MCPWM_TIMER_START_NO_STOP));
}

void sensor_task(void *pvParameters)
{
    while (1)
    {
        int adc_raw = 0;
        adc_oneshot_read(adc_moist_handle, MOISTER_AO_CHANNEL, &adc_raw);
        int temp_moist = map_value(adc_raw, dry_value, wet_value, 0, 100);
        if (temp_moist < 0)
            temp_moist = 0;
        if (temp_moist > 100)
            temp_moist = 100;

        float temp_t = 0.0, temp_h = 0.0;
        esp_err_t result = dht_read_float_data(DHT_TYPE_DHT11, PIN_DHT11, &temp_h, &temp_t);

        if (result == ESP_OK && xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            sys_state.temperature = temp_t;
            sys_state.moisture = temp_moist;
            xSemaphoreGive(state_mutex);

            ESP_LOGI(TAG, "SENSORS | Temp: %.1fC, Soil: %d%%", temp_t, temp_moist);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void alarm_task(void *pvParameters)
{
    bool is_alarming = false;

    while (1)
    {
        float current_t = 0;
        if (xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            current_t = sys_state.temperature;
            xSemaphoreGive(state_mutex);
        }

        if (current_t != 0.0)
        {
            bool temp_is_bad = (current_t > (target_temp + temp_tolerance)) ||
                               (current_t < (target_temp - 0.1));

            if (temp_is_bad && !is_alarming)
            {
                gpio_set_level(PIN_BUZZER, 1);
                ESP_LOGW(TAG, "ALARM | Temp is %.1fC (Outside safe zone)! BUZZER ON.", current_t);
                is_alarming = true;
            }
            else if (!temp_is_bad && is_alarming)
            {
                gpio_set_level(PIN_BUZZER, 0);
                ESP_LOGI(TAG, "ALARM | Temp stabilized at %.1fC. BUZZER OFF.", current_t);
                is_alarming = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void pid_fan_task(void *pvParameters)
{
    float Kp = 15.0;
    float Ki = 1.5;
    float Kd = 5.0;

    float integral = 0;
    float prev_error = 0;
    float dt = 2.0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2000);

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        float current_t = 0;
        if (xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            current_t = sys_state.temperature;
            xSemaphoreGive(state_mutex);
        }

        float error = current_t - target_temp;

        if (error <= 0)
        {
            setFanSpeed(0);
            integral = 0;
            prev_error = 0;
            continue;
        }

        float P = Kp * error;

        integral += error * dt;
        if (integral > 100)
            integral = 100;

        float I = Ki * integral;
        float D = Kd * ((error - prev_error) / dt);
        float output = P + I + D;

        int speed_percent = (int)output;
        if (speed_percent > 100)
            speed_percent = 100;
        if (speed_percent < 0)
            speed_percent = 0;

        setFanSpeed(speed_percent);
        prev_error = error;

        ESP_LOGI(TAG, "PID CONTROL | Error: +%.1fC -> Output Fan Speed: %d%%", error, speed_percent);
    }
}

void watering_task(void *pvParameters)
{
    bool is_watering = false;

    while (1)
    {
        int current_m = 0;
        if (xSemaphoreTake(state_mutex, portMAX_DELAY))
        {
            current_m = sys_state.moisture;
            xSemaphoreGive(state_mutex);
        }

        if (current_m < target_moisture)
        {
            if (!is_watering)
            {
                moveServo(90);
                // gpio_set_level(PIN_RELAY, 1);
                ESP_LOGI(TAG, "WATER TASK | Moisture low (%d%%). Valve OPEN, Relay ON.", current_m);
                is_watering = true;
            }
        }
        else
        {
            if (is_watering)
            {
                moveServo(0);
                gpio_set_level(PIN_RELAY, 0);
                ESP_LOGI(TAG, "WATER TASK | Moisture sufficient (%d%%). Valve CLOSED, Relay OFF.", current_m);
                is_watering = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "--- System Booting ---");

    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create Mutex. Halting.");
        return;
    }

    setup_gpio();
    soil_conf();
    servo_conf();
    fan_setup();
    ESP_LOGI(TAG, "Hardware Initialized.");

    xTaskCreate(sensor_task, "Sensor_Task", 4096, NULL, 5, NULL);
    xTaskCreate(pid_fan_task, "PID_Task", 4096, NULL, 4, NULL);
    xTaskCreate(watering_task, "Water_Task", 2048, NULL, 4, NULL);
    xTaskCreate(alarm_task, "Alarm_Task", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "Operating System Tasks Running. Main function sleeping.");

    vTaskSuspend(NULL);
}