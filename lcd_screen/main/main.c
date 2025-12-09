#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ili9341.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gptimer.h"
#include "lvgl.h"

/** 
 * Author: Clair Weir
 * Created: 25.05.2025
 * Last Update: 02.06.2025
 * Controller: ESP32-S3
 * LCD Screen: 2"4 Display, ILI9341
 * 
 * About: Display Text on the LCD screen
 * Connect Screen to ESP32-S3:
 *      - GND to G
 *      - 5V to 5V
 *      - 3.3V and LCD_RD to 3.3V
 *      - LCD_RST to pin 4
 *      - LCD_CS to pin 5
 *      - LCD_RS to pin 6
 *      - LCD_WR to pin 7
 *      - LCD_D0 to pin 13
 *      - LCD_D1 to pin 14
 *      - LCD_D2 to pin 3
 *      - LCD_D3 to pin 46
 *      - LCD_D4 to pin 9
 *      - LCD_D5 to pin 10
 *      - LCD_D6 to pin 11
 *      - LCD_D7 to pin 12
*/

// define the pins
#define LCD_D0 13
#define LCD_D1 14
#define LCD_D2 3
#define LCD_D3 46
#define LCD_D4 9
#define LCD_D5 10
#define LCD_D6 11
#define LCD_D7 12
#define LCD_WR 7
#define LCD_CS 5
#define LCD_RS 6
#define LCD_RST 4

// define screen dimensions and pixel clock freq
#define LCD_H_RES 240
#define LCD_V_RES 320
#define PIXEL_CLOCK_HZ 5000000

// define buffer
static lv_draw_buf_t draw_buf;

// gptimer callback, needed to keep lvgls internal timing
static bool gptimer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    // tells lvgl that 1ms has passed
    lv_tick_inc(1);
    return true;
}

// callback: sends pixeldata to display
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
   // converts pixelmap to lv_color format
    lv_color_t *color_p = (lv_color_t *)px_map;

    // get the panel from the user data
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t) lv_display_get_user_data(disp);

    // sends the pixel data to diplay
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    
    // we are done with drawing
    lv_disp_flush_ready(disp);
}

void app_main(void)
{
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;

   // sets up the i80 bus interface, sets up pins
    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = LCD_RS,
        .wr_gpio_num = LCD_WR,
        .data_gpio_nums = {
            LCD_D0, LCD_D1, LCD_D2, LCD_D3,
            LCD_D4, LCD_D5, LCD_D6, LCD_D7
        },
        .bus_width = 8,
        .max_transfer_bytes = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
        .clk_src = LCD_CLK_SRC_DEFAULT,
    };
    esp_lcd_new_i80_bus(&bus_config, &i80_bus);

    // sets up io (communication) with display
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_CS,
        .pclk_hz = PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_cmd_level = 0,
            .dc_data_level = 1,
            .dc_idle_level = 0,
            .dc_dummy_level = 0
        },
        .flags = {
            .swap_color_bytes = true
        },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle);

    // sets up panel driver
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };

    // set up reset and initialise
    esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    // mirror the display vertically
    esp_lcd_panel_mirror(panel_handle, true, false);

    // turn display on
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // initialise lvgl
    lv_init();

    // set up the framebuffer
    lv_color_format_t color_format = LV_COLOR_FORMAT_NATIVE;
    size_t buf_size = LCD_H_RES * LCD_V_RES * sizeof(lv_color_t); 
    lv_color_t *buf = (lv_color_t *)heap_caps_calloc(LCD_H_RES * LCD_V_RES, sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf);   

    lv_draw_buf_init(&draw_buf,
                 LCD_H_RES,
                 LCD_V_RES,
                 color_format,
                 LCD_H_RES,    
                 buf,
                 buf_size);


    // set up lvgl display driver
    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_draw_buffers(disp, &draw_buf, NULL);
    lv_display_set_flush_cb(disp, lvgl_flush_cb); 
    lv_display_set_user_data(disp, panel_handle);      

    // setup gptimer
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t gptimer_conf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000
    };
    gptimer_new_timer(&gptimer_conf, &gptimer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_on_alarm_cb
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000,  
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);

    // create label with backgroundcolor and text
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);
    lv_label_set_text(label, "Hello World!");
    lv_obj_set_style_text_color(label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    
    while (true) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
