#include <stdio.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"

/** 
 * Author: Clair Weir
 * Created: 10.06.2025
 * Last Update: 10.06.2025
 * Controller: ESP32-S3
 * Ultrasonic Sensor: RCW-0001
 * Speaker: LSM-50F-8
 * 
 * About: Estimate the distance to object using the ultrasonic sensor
 * & play a tone on the speaker indicating the distance to the object.
 * 
 * Connect Ultrasonic Sensor: 
 *      Vcc to 3.3V
 *      Gnd to G
 *      Trig to pin 4
 *      Echo to pin 5
 * Connect Speaker:
 *      + to pin 6
 *      - to G
*/

// Input and Output pins
#define trigger_pin 4 // ultrasonic sensor input
#define echo_pin 5 // ultrasonic sensor output
#define speaker_pin 6 // speaker input
#define vel 345.5f // air spreading velocity at 23°C

static gptimer_handle_t counter; // used for checking the length of the ultrasonic signal
static gptimer_handle_t tone_timer; // used for making the speaker tone

// the signal set on the gpio pin, oscillates between true and false
bool value = false;
bool tone_timer_running = false;

// function prototypes, informs the compiler about the functions
static gptimer_handle_t setupTimer();
static float runUltrasonic();
static void setupAlarm(gptimer_handle_t gptimer);
static void run_cont();

// Interrupt function, sets the gpio pin to either true or false 
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    value = !value;
    gpio_set_level(speaker_pin, value);
    return true;
}

void app_main(void){

    // setup the pins
    gpio_reset_pin(trigger_pin);
    gpio_reset_pin(echo_pin);
    gpio_reset_pin(speaker_pin);
     
    gpio_set_direction(trigger_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(echo_pin, GPIO_MODE_INPUT);
    gpio_set_direction(speaker_pin, GPIO_MODE_OUTPUT);

    // pull trigger low on startup
    gpio_set_level(trigger_pin, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // setup the timers
    counter = setupTimer();
    gptimer_enable(counter);

    tone_timer = setupTimer();
    setupAlarm(tone_timer);

    //continously read values
    run_cont();

}

// run continously
// note: implementation could be nicer (maybe with another timer?) 
static void run_cont(){
    while(true){
        // get distance
        float dist = runUltrasonic();
        // if dist <= 30 cm start tone
        if(dist <= 30){
            // if dist < 5 cm -> continous sound
            if(dist < 5){
                // start tone only if it is not running already
                if(!tone_timer_running){
                    gptimer_start(tone_timer);
                    tone_timer_running = true;
                }
            }else{
                // calc the delay in ms
                int length = dist*(50/3);
                // if tone is already running: stop
                if(tone_timer_running){
                    gptimer_stop(tone_timer);
                    tone_timer_running = false;
                }

                //play tone with calculated break
                gptimer_start(tone_timer);
                vTaskDelay(pdMS_TO_TICKS(length));
                gptimer_stop(tone_timer);
            }
        }else{
            // if distance to big and tone is playing: stop
            if(tone_timer_running){
                gptimer_stop(tone_timer);
                tone_timer_running = false;
            }
        }
    }
}

static float runUltrasonic(){

    // send 10us signal to trigger to start
    gpio_set_level(trigger_pin, true);
    esp_rom_delay_us(10);
    gpio_set_level(trigger_pin, false);

    // wait for signal on echo
    while(!gpio_get_level(echo_pin)){}

    // when resp is true start timer
    gptimer_set_raw_count(counter, 0);
    gptimer_start(counter);

    // wait until signal is finished
    while(gpio_get_level(echo_pin)){}

    // get time of signal
    uint64_t time_raw;
    gptimer_get_raw_count(counter, &time_raw);
    gptimer_stop(counter);

    float time_s = time_raw/1000000.0f; //get time in secs

    // calculate the distance in cm
    float range = ((time_s * vel)/2.0f)*100.0f;

    printf("Distance is %fcm\n", range);

    vTaskDelay(pdMS_TO_TICKS(100));

    return range;
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

    return gptimer;
}

static void setupAlarm(gptimer_handle_t gptimer){

    // set the event callback (interrupt function)
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_alarm_cb,
    };

    gptimer_register_event_callbacks(gptimer, &cbs, NULL);
    // set up alarm to trigger interrupt 
    gptimer_alarm_config_t alarm_config_new = {
        .alarm_count = 3822, // determines the note: 1000000/(hz of note)
        .reload_count = 0, 
        .flags.auto_reload_on_alarm = true, 
    };

    gptimer_set_alarm_action(gptimer, &alarm_config_new);

    gptimer_enable(gptimer);
}
