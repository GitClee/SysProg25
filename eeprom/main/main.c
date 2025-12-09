#include <stdio.h>

// lib for i2c
#include "driver/i2c.h"

/** 
 * Author: Clair Weir
 * Created: 28.04.2025
 * Last Update: 04.05.2025
 * Controller: ESP32-S3
 * EEPROM: 24C08WP
 * 
 * About: Use I2C to write data to the EEPROM and read it back
 * Connect EEPROM to ESP32-S3:
 *              - VCC (PIN 8) to 3.3V
 *              - VSS, A0, A1, A2, WP (PIN 1, 2, 3, 4, 7) to GND
 *              - SDA (PIN 5) to PIN 5
 *              - SCL (PIN 6) to PIN 4
*/


// i2c 
#define I2C_SCL 4
#define I2C_SDA 5
#define EEPROM_ADDR 0x50
#define FREQ 400000
#define TIMEOUT_MS 1000


// function prototypes
static void init_i2c();
static void write_byte(uint8_t mem_addr, uint8_t data);
static void read_byte(uint8_t mem_addr, uint8_t *data);


// data to be written to the eeprom
const char *data = "Hello World. Hello World. Hello World. Hello World. Hello World.";
const uint8_t full_mem_addr = 0x00;

void app_main(void)
{
    init_i2c();
    uint8_t *byte = data;
    uint8_t addr = full_mem_addr;

    // write the data
    while(*byte != '\0'){
        write_byte(addr, *byte);
        byte++;
        addr++;
    }
    write_byte(addr, *byte);

    printf("Finished writing.\n");
    // read the data
    uint8_t read_data[100] = {0};
    byte = read_data;
    addr = full_mem_addr;
    for(int i = 0; i < 65; i++){
        read_byte(addr, byte);
        byte++;
        addr++;
    }
    printf("%s\n", read_data);

}

// initialise i2c
void init_i2c(void){
     // configure i2c
     i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = FREQ,
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

// write a byte to a specific address
void write_byte(uint8_t mem_addr, uint8_t data){


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mem_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// reads bytes from the eeprom
void read_byte(uint8_t mem_addr, uint8_t *data){
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mem_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (EEPROM_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}



