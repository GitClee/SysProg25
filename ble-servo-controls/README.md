# SERVO Controls using a BLE GATT SERVER

- [About](#1-reset-phase)
- [Servo Controlls](#servo-controls)
- [Bluetooth](#bluetooth)
- [GATT Server](#gatt-server)
- [Setup guide](#setup-guide)
- [Using the Serial Bluetooth Terminal](#using-the-serial-bluetooth-terminal)
- [Links](#links)


## About


This repository implements a BLE GATT Server on a ESP32-S3 that can receive messages over Bluetooth and then turns the arm of a servo accordingly. In this document I will go over relevant information about the servo and bluetooth and explain details about GATT Servers. Finally there will be a setup guide for the programm and the Serial Bluetooth Terminal App used to communicate with the ESP32.


## Servo Controls


### Basics

- Servo MC-410
- three connections: VCC (red), GND (brown) and signal (orange) (might differ based on your servo)
- contol the servo position with PWM (Pulse Wave Modulation)
- PWM: length of signal pulse (20ms period) determines position of arm

![PWM Signal](/images/pwm_signal.png)

### Execution

- idea: use two timers to create the rectangle function (timer_20ms and timer_signal)
- timer_20ms runs for 20ms and then sets signal to high and activates the timer_signal
- timer signal runs for certain time (depending on direction) and then sets signal to low
-> rectangle signal 

- let the 20ms timer run for a few ms to give the servo time to turn

```cmake
// Interrupt function, turns signal off
static bool IRAM_ATTR timer_alarm_off(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gpio_set_level(SIGNALPIN, false);
    return true;
}
```

```cmake
// Interrupt function, turns signal on and starts the timer_signal
static bool IRAM_ATTR timer_alarm_on(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gpio_set_level(SIGNALPIN, true);
    resetSignalAlarm(signal_length_ticks);

    return true;
}
```


## Bluetooth

- two forms: Classic Bluetooth and Bluetooth Low Energy (BLE)
- Classic Bluetooth supports higher data-rates
- BLE uses less electricity and has less latency
- Classic can only connect Peer-To-Peer
- BLE also support multiple connections

- The ESP32-S3 only supports BLE
- We'll use NimBLE, a BLE stack and we'll implement a GATT Server


## GATT Server


### Basics

#### GATT: Generic Attribute Profile

- defines the way tow BLE devices transfer data back and forth
- needs a dedicated connection between devices, only works Peer-To-Peer
- ESP32 functions as server, awaits request form client (phone)
- Transactions based on Profiles, Services and Characteristics

![GATT Transactions](/images/gatt_transactions.png)

- Profile is a collection of Services
- Services and Characteristics have a unique UUID
- Services group Characteristics
- Characteristics determine interactions (read, write, ...)

#### GAP: Generic Access Profile

- controls the connections and the advertising in Bluetooth
- makes device "visible" and handles connections
- peripheral (ESP32) will transmit advertising package in an interval

#### Execution

- define gatt service and characteristics, add read and write functions 
- we wont use the read function, but it is necessary to declare -> we'll leave it empty

```cmake
// struct that defines the services of our gatt server
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &UART_SERVICE_UUID.u,
    .characteristics = (struct ble_gatt_chr_def[]){
        {.uuid = &UART_CHAR_UUID_RX.u, 
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .access_cb = ble_get_data,
        .val_handle = &rx_handle},
        {.uuid = &UART_CHAR_UUID_TX.u,
          .flags = BLE_GATT_CHR_F_NOTIFY,
          .val_handle = &tx_handle,
          .access_cb = device_read},

        {0}}},
    {0}};
```

```cmake
// ble service function, receives data from the client
static int ble_get_data(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg){
    char value = ctxt->om->om_data[0];
    if(value == 'l'){
        moveServo(1000);
    }else if (value == 'r'){
        moveServo(2000);
    }else if(value == 'm'){
        moveServo(1500);
    }
    return 0;
}

```

- define advertisement and the gap event handler

```cmake
// give ble devices information about device and services
static void ble_app_advertise(void){
    // define information about this device
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // define connection and discovery types
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; 
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; 
    
    // start advertising
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}
```

```cmake
// triggered by gap events 
static int ble_gap_event(struct ble_gap_event *event, void *arg){
    switch (event->type){

        // device attempts to connect
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0){
                // if connection failed, start advertising
                ble_app_advertise();
            }
            break;
        
        // advertising complete
        case BLE_GAP_EVENT_ADV_COMPLETE:
            //start advertising again
            ble_app_advertise();
            break;
        default:
            break;
    }
    return 0;
}
```

- initalise and start

```cmake
void app_main(void) {

    // set up GPIO Pin
    gpio_set_direction(SIGNALPIN, GPIO_MODE_OUTPUT);

    // setup timers for the servo function
    setupTimers();

    // setup nimble gatt server
    nvs_flash_init();
    nimble_port_init();
    ble_svc_gap_device_name_set("ESP32 Servo Controlls");
    ble_svc_gap_init(); 
    ble_svc_gatt_init(); 
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task); 

}
```

## Setup guide


You'll need:
- ESP32-S3
- Servo (MC-410 or similar)
- Serial Bluetooth Terminal on your smart phone

1. Connect ESP32-S3 to the Servo:
    - VCC (red cable) to 3.3V
    - GND (brown cable) to GND
    - Signal (orange cable) to PIN 4 

**Cable colors might vary depending on your servo!**

2. Settings:
    - Enable Bluetooth and NimBLE in your SDK Config Editor (click gear icon the ESP-IDF menu bar)
    
    ![Enable Bluetooth](/images/enable_bt.png)

    - Make sure REQUIRES driver bt nvs_flash has been added to the CMakesLists in main

3. Build and Flash the Project

4. The ESP32 is ready!


## Using the Serial Bluetooth Terminal


1. Download **Serial Bluetooth Terminal** to your smartphone and open the app

2. Tap the Menu Icon (≡) and then tap on *Devices*

3. Switch to the ***Bluetooth LE tab*** and tap ***SCAN***

4. Tap and hold ***ESP32 Servo Controls*** until a menu pops up

5. Tap on ***Edit***

6. Select ***Custom***, click on ***Service UUID*** and pick the second option

7. Now tap on ***Read characteristic UUID*** and ***Write characteristic UUID*** and pick the only option

![PWM Signal](/images/sbt_setup.jpg)

8. Tap on the check mark and tap on ***ESP32 Servo Contols***
(if the connection fails try reseting the ESP3)

9. Now you can send messages to the ESP32
Sending **l**, **m** and **r** will move the servo arm to the left middle and right respectively


## Links

- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/ble/index.html
- https://www.youtube.com/watch?v=EIo5aZ3c89Q
- https://learn.adafruit.com/introduction-to-bluetooth-low-energy/gap
- https://learn.adafruit.com/introduction-to-bluetooth-low-energy/gatt