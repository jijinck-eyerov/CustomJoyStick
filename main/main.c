/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "joystick_buttons.h"
#include "driver/adc.h"

#define HID_JOYSTICK_TAG "HID_JOYSTICK"

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "example";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[]= {
    TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD))};

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "EYEROV",             // 1: Manufacturer
    "EYEROV CUSTOM JOYSTICK",      // 2: Product
    "052024",              // 3: Serials, should use chip ID
    "HID interface",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

/********* Application ***************/

typedef enum {
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_t;

#define DISTANCE_MAX        125
#define DELTA_SCALAR        5

static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
{
    static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
    static uint32_t distance = 0;

    // Calculate next delta
    if (cur_dir == MOUSE_DIR_RIGHT) {
        *delta_x_ret = DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_DOWN) {
        *delta_x_ret = 0;
        *delta_y_ret = DELTA_SCALAR;
    } else if (cur_dir == MOUSE_DIR_LEFT) {
        *delta_x_ret = -DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_UP) {
        *delta_x_ret = 0;
        *delta_y_ret = -DELTA_SCALAR;
    }

    // Update cumulative distance for current direction
    distance += DELTA_SCALAR;
    // Check if we need to change direction
    if (distance >= DISTANCE_MAX) {
        distance = 0;
        cur_dir++;
        if (cur_dir == MOUSE_DIR_MAX) {
            cur_dir = 0;
        }
    }
}

static void app_send_hid_demo(void)
{

    // Mouse output: Move mouse cursor in square trajectory
    ESP_LOGI(TAG, "Sending Mouse report");
    int8_t delta_x;
    int8_t delta_y;
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // for (int i = 0; i < (DISTANCE_MAX / DELTA_SCALAR) * 4; i++) {
        // Get the next x and y delta in the draw square pattern
        mouse_draw_square_next_delta(&delta_x, &delta_y);
        tud_hid_gamepad_report(HID_ITF_PROTOCOL_KEYBOARD, 100, 200, 300, 2, 2, 2, 3, 0xffffffff);
        vTaskDelay(pdMS_TO_TICKS(200));
    // }
}


static int readJoystickChannel(adc1_channel_t channel)
{
    uint32_t adc_value=0, sample_count=5;
    adc1_config_width(ADC_WIDTH_BIT_12);                  //Range 0-1023
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
    // return (uint8_t)(adc1_get_raw(channel) >> 2);         //Read analog and shift to 0-255
    for (int i =0;i<sample_count; i++)
    {
        adc_value+= adc1_get_raw(channel);
    }

    return ((int)(adc_value/sample_count));         //Read analog and shift to 0-255

}

static void read_joystick_task(void *pvParameter)
{
    int8_t js1x,js1y,js2x,js2y,js3x,js3y;

    uint64_t current_sum;

    uint16_t buttons = 0;
    uint32_t last_sum = 0;
    uint16_t last_buttons = 0;

    joystick_buttons_event_t ev;
    QueueHandle_t joystick_buttons_events = joystick_buttons_init();

    while(true) {
        ESP_LOGD(HID_JOYSTICK_TAG, "wait for joystick_buttons_events");

        if (xQueueReceive(joystick_buttons_events, &ev, 50/portTICK_PERIOD_MS)) {
            // ESP_LOGI(HID_JOYSTICK_TAG, "joystick_buttons_events %d", ev.state);
            buttons = ev.state;
        }

        js1x = (int8_t)((readJoystickChannel(ADC1_CHANNEL_3)/16)-90)/5;
        js1y = (int8_t)((readJoystickChannel(ADC1_CHANNEL_4)/16)-90)/5;
        js2x = (int8_t)((readJoystickChannel(ADC1_CHANNEL_5)/16)-90)/5;
        js2y = (int8_t)((readJoystickChannel(ADC1_CHANNEL_6)/16)-90)/5;
        js3x = (int8_t)((readJoystickChannel(ADC1_CHANNEL_7)/16)-90)/5;
        js3y = (int8_t)((readJoystickChannel(ADC1_CHANNEL_8)/16)-90)/5;
        
        js1x *= 5;
        js1y *= 5;
        js2x *= 5;
        js2y *= 5;
        js3x *= 5;
        js3y *= 5;
        
        // very simple checksum :)
        current_sum = (js3x<<40) + (js3y<<32) +(js2x<<24) + (js2y<<16) + (js1x<<8) + js1y;

        ESP_LOGD(HID_JOYSTICK_TAG, "last_sum %ld", last_sum);
        
        // only transmit if something changed
        if (current_sum != last_sum || last_buttons != buttons) {
            ESP_LOGD(HID_JOYSTICK_TAG, "send buttons %d JS1 X=%d Y=%d JS2 X=%d Y=%d JS3 X=%d Y=%d", buttons, js1x, js1y, js2x, js2y, js3x, js3y);

            tud_hid_gamepad_report(HID_ITF_PROTOCOL_KEYBOARD, js1x, js1y, js2x, js2y, js3x, js3y, 0, buttons);
            
            // esp_hidd_send_joystick_value(hid_conn_id, buttons, js1x, js1y, js2x, js2y);
        }

        // used to detect state changes which trigger sending a packet
        last_sum = current_sum;
        last_buttons = buttons;
    }

}

static esp_err_t read_joystick_init(void)
{

    xTaskCreate(read_joystick_task, "read_joystick_task", 4096, NULL, 4, NULL);

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret;


    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    static bool send_hid_data = true;
    while (send_hid_data) {
        if (tud_mounted()) {
            ESP_LOGE(HID_JOYSTICK_TAG, "%s joystick mounted\n", __func__);

            send_hid_data = false;
                // app_send_hid_demo();
            if((ret = read_joystick_init()) != ESP_OK) {
                ESP_LOGE(HID_JOYSTICK_TAG, "%s init read joystick failed\n", __func__);
            }
    
            // send_hid_data = !gpio_get_level(APP_BUTTON);
        }
        ESP_LOGE(HID_JOYSTICK_TAG, "%s joystick mount failed\n", __func__);
        vTaskDelay(pdMS_TO_TICKS(100));
    }    
    while (1) {
        ESP_LOGI(HID_JOYSTICK_TAG, "Looping");
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}
