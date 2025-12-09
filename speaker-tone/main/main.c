#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"

/** 
 * Author: Clair Weir
 * Created: 17.03.2025
 * Last Update: 31.03.2025
 * Controller: ESP32-S3
 * 
 * About: Uses the ESP32-S3 and a speaker to play a song using timer and interrupt functions.
 * Plug in the speaker on pin 1 and Ground (G)
*/

// cable on pin 1 and gnd
#define speakerPin 1

// define the notes in hz 
const float E = 329.63;
const float F = 349.23;
const float G = 392;
const float D = 293.66;
const float C = 261.63;

// the signal set on the gpio pin, oscillates between true and false
volatile bool value = false;

// function prototypes, informs the compiler about the functions
static gptimer_handle_t setupTimer();
static void setAlarm(float hz, gptimer_handle_t gptimer);
static void playNote(gptimer_handle_t gptimer, float note, int length);

// Interrupt function, sets the gpio pin to either true or false 
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    value = !value;
    gpio_set_level(speakerPin, value);
    return true;
}

void app_main(void) {

    // setup the gpio pin
    gpio_reset_pin(speakerPin);
    gpio_set_direction(speakerPin, GPIO_MODE_OUTPUT);

    // Song: E E F G G F E D C C D E E D D

    // setup the timer
    gptimer_handle_t gptimer = setupTimer();


    // Play song, one note after the other
    playNote(gptimer, E, 500);

    playNote(gptimer, E, 500);

    playNote(gptimer, F, 500);

    playNote(gptimer, G, 500);

    playNote(gptimer, G, 500);

    playNote(gptimer, F, 500);

    playNote(gptimer, E, 500);

    playNote(gptimer, D, 500);

    playNote(gptimer, C, 500);
    
    playNote(gptimer, C, 500);

    playNote(gptimer, D, 500);

    playNote(gptimer, E, 500);

    playNote(gptimer, E, 750);

    playNote(gptimer, D, 250);

    playNote(gptimer, D, 500);

}

// function that creates and initalizes the timer
static gptimer_handle_t setupTimer() {

    gptimer_handle_t gptimer = NULL;

    // timer configuration, runs at 1MHZ
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };

    gptimer_new_timer(&timer_config, &gptimer);

    // set the event callback (interrupt function)
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_cb,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_enable(gptimer);

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

// play a note with a certain length
static void playNote(gptimer_handle_t gptimer, float note, int length){

    setAlarm(note, gptimer);

    vTaskDelay(pdMS_TO_TICKS(length));
    
    // stop note 
    gptimer_stop(gptimer);

    // wait for 10 ms
    vTaskDelay(pdMS_TO_TICKS(10));
}