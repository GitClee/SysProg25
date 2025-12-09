#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"

/** 
 * Author: Clair Weir
 * Created: 24.03.2025
 * Last Update: 31.03.2025
 * Controller: ESP32-S3
 * 
 * About: Exercise Sheet 1 Exercise 4a
 * Play a tone every 5 sec
*/

// cable on pin 1 and gnd
#define speakerPin 1

// define the notes in hz 
const float C = 261.63;

// the signal set on the gpio pin, oscillates between true and false
volatile bool value = false;

// function prototypes, informs the compiler about the functions
static gptimer_handle_t setupTimer();
static void setAlarm(float hz, gptimer_handle_t gptimer);
static void playNote(gptimer_handle_t gptimer, float note, int length);
static void playNoteTask(void *param);

// Interrupt function, sets the gpio pin to either true or false 
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    value = !value;
    gpio_set_level(speakerPin, value);
    return true;
}

// Interrupt function, play tone 
static bool IRAM_ATTR timer_alarm_cb_2(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gptimer_handle_t gptimer = (gptimer_handle_t) user_data; 
    xTaskCreate(playNoteTask, "playNoteTask", 4096, gptimer, 1, NULL);
    return true;
}

void app_main(void) {

    // setup the gpio pin
    gpio_reset_pin(speakerPin);
    gpio_set_direction(speakerPin, GPIO_MODE_OUTPUT);


    // setup the timer
    gptimer_handle_t gptimer = setupTimer();

}

// function that creates and initalizes the timers
static gptimer_handle_t setupTimer() {

    gptimer_handle_t gptimer = NULL;
    gptimer_handle_t playTimer = NULL;

    // timer configuration, runs at 1MHZ
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };

    gptimer_new_timer(&timer_config, &gptimer);
    gptimer_new_timer(&timer_config, &playTimer);

    // set the event callback (interrupt function)
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_cb,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_event_callbacks_t cbs2 = {
        .on_alarm = timer_alarm_cb_2,
    };
    gptimer_register_event_callbacks(playTimer, &cbs2, (void*) gptimer);

    gptimer_enable(gptimer);
    gptimer_enable(playTimer);

    // set up alarm to trigger interrupt at 5sec
    gptimer_alarm_config_t alarm_config_new = {
        .alarm_count = 5000000,
        .reload_count = 0, 
        .flags.auto_reload_on_alarm = true, 
    };

    gptimer_set_alarm_action(playTimer, &alarm_config_new);

    // start timer
    gptimer_start(playTimer);

    return gptimer;
}


// set up the alarm
static void setAlarm(float hz, gptimer_handle_t gptimer) {

    // set up alarm to trigger interrupt at 1000000/hz
    gptimer_alarm_config_t alarm_config_new = {
        .alarm_count = 1000000/hz,
        .reload_count = 0, 
        .flags.auto_reload_on_alarm = true, 
    };

    gptimer_set_alarm_action(gptimer, &alarm_config_new);

    // start timer
    gptimer_start(gptimer);
    
}

// create task, so interrupt can execute
static void playNoteTask(void *param) {
    gptimer_handle_t gptimer = (gptimer_handle_t) param;
    playNote(gptimer, C, 500);
    vTaskDelete(NULL); 
}

// play a note with a certain length
static void playNote(gptimer_handle_t gptimer, float note, int length){
    setAlarm(note, gptimer);

    vTaskDelay(pdMS_TO_TICKS(length));
        
    // stop note 
    gptimer_stop(gptimer);

    // wait for 10 ms
    vTaskDelay(pdMS_TO_TICKS(10));
    
}