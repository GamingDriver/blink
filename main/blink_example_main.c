/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/ledc.h"

static const char *TAG = "example";


/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_2

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

#define TAG "MOTOR_CONTROL"

#define MOTOR1_PIN1 20
#define MOTOR1_PIN2 21
#define ENABLE1_PIN 14

#define PWM_FREQUENCY 30000
#define PWM_CHANNEL   LEDC_CHANNEL_0
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define INITIAL_DUTY_CYCLE 200

static int duty_cycle = INITIAL_DUTY_CYCLE;

void configure_pwm() {
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENABLE1_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void configure_gpio() {
    // Configure GPIO pins
    gpio_pad_select_gpio(MOTOR1_PIN1);
    gpio_set_direction(MOTOR1_PIN1, GPIO_MODE_OUTPUT);
    
    gpio_pad_select_gpio(MOTOR1_PIN2);
    gpio_set_direction(MOTOR1_PIN2, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(ENABLE1_PIN);
    gpio_set_direction(ENABLE1_PIN, GPIO_MODE_OUTPUT);
}



static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void app_main(void)
{
    configure_pwm();
    configure_gpio();

    ESP_LOGI(TAG, "Testing DC Motor...");

    while (true) {
        // Move the DC motor forward at maximum speed
        ESP_LOGI(TAG, "Moving Forward");
        gpio_set_level(MOTOR1_PIN1, 0);
        gpio_set_level(MOTOR1_PIN2, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Stop the DC motor
        ESP_LOGI(TAG, "Motor stopped");
        gpio_set_level(MOTOR1_PIN1, 0);
        gpio_set_level(MOTOR1_PIN2, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        duty_cycle = INITIAL_DUTY_CYCLE;
    }
    /*
    int led_state_user_input;
    configure_led();
    //uint16_t n = 125;
    while(true){
        scanf("%d", &led_state_user_input);
        if (led_state_user_input == 1 && s_led_state == 0){
            s_led_state = 1;
            blink_led();
            ESP_LOGI(TAG, "Turning the LED ON");
        }
        else if(led_state_user_input == 0 && s_led_state == 1){
            s_led_state = 0;
            blink_led();
            ESP_LOGI(TAG, "Turning the LED OFF");
        }
    }
    */
        /*
    while(true) {
        ESP_LOGI(TAG, "Turning the LED %s! Period = %d", s_led_state == true ? "ON" : "OFF",n*2);
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(n / portTICK_PERIOD_MS);
    }
    */
/*
    
    bool x = false;
    configure_led();

    while (true) {
        if (n >= 1000 || x == true) {
            x = true;
            if (x == true && n > 125) {
                n -= 25;
            }
            else
                x = false;
        }   
        else {
            n += 25;
            x = false;
        }
        ESP_LOGI(TAG, "Turning the LED %s! Period = %d", s_led_state == true ? "ON" : "OFF",n*2);
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(n / portTICK_PERIOD_MS);
        //my_sum(n);
    }
*/
}

//global s


uint16_t my_sum(uint16_t n)
{
    uint16_t s = 0;
    for (uint16_t i = 1; i <= 10; i++) //for i in range(1,10,1)
    {
        s = s + i;
    }
    return s;
}