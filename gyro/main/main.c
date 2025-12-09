#include <stdio.h>
#include "driver/gptimer.h"

// lib for i2c
#include "driver/i2c.h"

/** 
 * Author: Clair Weir
 * Created: 26.04.2025
 * Last Update: 28.04.2025
 * Controller: ESP32-S3
 * Sensor: MPU6050
 * 
 * About:   1. Use I2C to read the values of the MPU6050, 
 *          2. Write a blocking delay funtion only using (gp)timer
 * Connect MPU to ESP32-S3:
 *              - VCC to 3.3V
 *              - GND to GND
 *              - SDA to PIN 5
 *              - SCL to PIN 4
*/

static gptimer_handle_t gptimer = NULL;

// i2c 
#define I2C_SCL 4
#define I2C_SDA 5
#define MPU_ADDR 0x68
#define FREQ 400000
#define TIMEOUT_MS 1000


// function prototypes, informs the compiler about the functions
static void setupTimer();
static uint64_t current_time();
static void blocking_delay();
static void init_i2c();
static void wakeup_mpu();
static void read_data();


void app_main(void)
{
    init_i2c();
    wakeup_mpu();
    while(true){
        read_data();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}

// Exercise 1

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

// wake mpu from sleep mode
void wakeup_mpu(){
    uint8_t wake_cmd[2] = {0x6B, 0x00};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, wake_cmd, sizeof(wake_cmd), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}



// read the data from mpu
void read_data(){
    //starting address for data, data stored over 6 registers
    uint8_t reg = 0x3B;

    uint8_t data[14];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    int16_t accel_x = (data[0] << 8) | data[1];
    int16_t accel_y = (data[2] << 8) | data[3];
    int16_t accel_z = (data[4] << 8) | data[5];

    float real_accel_x = ((accel_x - 2600) / 16384.0)*9.80665;
    float real_accel_y = ((accel_y + 17350) / 16384.0)*9.80665;
    float real_accel_z = ((accel_z + 500) / 16384.0)*9.80665;
 
    int16_t temp = (data[6] << 8) | data[7];
    float temp_real = temp /340.0 +36.53;

    int16_t gyro_x = (data[8] << 8) | data[9];
    int16_t gyro_y = (data[10] <<8) | data[11];
    int16_t gyro_z = (data[12] << 8) | data[13];

    float real_gyro_x = ((gyro_x - 5100) / 131.0);
    float real_gyro_y = ((gyro_y + 3500)  / 131.0);
    float real_gyro_z = ((gyro_z + 1100) / 131.0);

    printf("ACC X:%.2fm/s², Y:%.2fm/s², Z:%.2fm/s²     ", real_accel_x, real_accel_y, real_accel_z);
    printf("Temp: %.2f°C     ", temp_real);
    printf("GYRO X:%.2f°/s, Y:%.2f°/s, Z:%.2f°/s\n", real_gyro_x, real_gyro_y, real_gyro_z);

}


// Excercise 2

// blocking delay function, only uses gptimer.h
void blocking_delay(int ms){

    if(gptimer == NULL){
        setupTimer();
    }

    uint64_t start = current_time();
    uint64_t delay = ms * 1000;

    while((current_time() - start) < delay){
        printf("...\n");
    }

}

// function that creates and initalizes the timer
static void setupTimer() {

    // timer configuration, runs at 1MHZ
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };

    gptimer_new_timer(&timer_config, &gptimer);

    gptimer_enable(gptimer);

    gptimer_start(gptimer);
}

// get the current time of gptimer
uint64_t current_time(){
    uint64_t time = 0;
    gptimer_get_raw_count(gptimer, &time);
    return time;
}


