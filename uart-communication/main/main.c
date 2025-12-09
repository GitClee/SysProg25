#include <driver/uart.h>
#include <string.h>
#include "led_strip.h"

/** 
 * Author: Clair Weir
 * Created: 31.03.2025
 * Last Update: 07.04.2025
 * Controller: ESP32-S3
 * 
 * About: Send and Receive Data with UART Communication. Receive data using Interrupts.
*/

#define BLINK_PIN 38

static intr_handle_t handle_console;
static led_strip_handle_t led_strip;

// function prototypes
void receive_data(void);
void send_data(void);
void blink(void);
void uart_event_task(void *pvParameters);

int app_main(void){
    // set up led strip 
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_PIN,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);

    receive_data();
    return 0;
}

void receive_data(void){
    // use UART 0 for the UART to USB Connection use UART 2 for PIN Connection
    // const uart_port_t uart_num = UART_NUM_2;
    const uart_port_t uart_num = UART_NUM_0;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Connect with pins 4 and 5 and gnd
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 18, 19));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));

    xTaskCreate(uart_event_task, "uart_event_task", 2048, (void *)uart_queue, 12, NULL);
   
}

void uart_event_task(void *pvParameters){
    QueueSetHandle_t uart_queue = (QueueSetHandle_t)pvParameters;
    uart_event_t event;
    uint8_t data[128];

    for (;;) {
        if(xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){
            switch (event.type) {
                case UART_DATA: {
                    blink();
                    int len = uart_read_bytes(UART_NUM_0, data, event.size, 100);
                    if (len > 0){
                        data[len] = '\0';
                        printf("%s\n", data);
                    }
                    break;
                }
                default:
                    // Andere Ereignisse ausgeben
                    printf("UART Event: %d\n", event.type);
                    break;
            }
        }
    }
}


void send_data(void){
    // use UART 0 for the UART to USB Connection use UART 2 for PIN Connection
    // const uart_port_t uart_num = UART_NUM_2;
    const uart_port_t uart_num = UART_NUM_0;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Connect with pins 4 and 5 and gnd
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 18, 19));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));

    // Send data over UART.

    while(1){
        char* test_str = "Hello World!";

        uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
   
}

void blink(void){
    led_strip_set_pixel(led_strip, 0, 16, 16, 16);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(500));
    led_strip_clear(led_strip);
}





