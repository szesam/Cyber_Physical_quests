

// Standard C library
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// GPIO library
#include "driver/gpio.h"

// RTOS tasks library
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Timer library
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

// i2c library
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_log.h"

// Console IO library
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_vfs_dev.h"

//SERVO library
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//files to include
#include "alphafonttable.h"

#include "driver/adc.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif




// Timer definition
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value


//ADC definitions
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   50          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_5;     //GPIO33 if ADC1, GPIO14 if ADC2 - channel for encoder reading
static const adc_bits_width_t width = ADC_WIDTH_BIT_10; //10bit width for ez conversion. 
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

//global defintions
int count = 0; //used for counting the number of black-white transitions in encoder
bool one_pulse = true; //used for determining whether or not to increment count in adc_reading


// Timer define
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval is 0.1s
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload


////////////////////////////// Getting speed methods and configurations //////////////////////////////////////////////////////////
//global flag for saying when timer interrumps every 1 second 
int time_flag = 0;

//GLOBAL COUNTER FOR REV SPIKES 
int spikes = 0;

///////////// ADC READING /////////////////////////
// #if CONFIG_IDF_TARGET_ESP32
// static esp_adc_cal_characteristics_t *adc_chars;
// static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
// #elif CONFIG_IDF_TARGET_ESP32S2BETA
// static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
// #endif 
// static const adc_atten_t atten = ADC_ATTEN_11db;
// static const adc_unit_t unit = ADC_UNIT_1;

#if CONFIG_IDF_TARGET_ESP32
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
#endif

void adc_init(){
    #if CONFIG_IDF_TARGET_ESP32
        //Check if Two Point or Vref are burned into eFuse
        check_efuse();
    #endif

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    #if CONFIG_IDF_TARGET_ESP32
        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);
    #endif
}

//encoder wheel speed
void encoder_adc(void* arg)
{
    //Continuously sample ADC1
    while (1) 
    {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        // printf("adc_reading: %d\n", adc_reading); //adc_reading is 1023 when black, below that when white.
        if (adc_reading < 1000 && one_pulse == true)
        {
            //only read one count each transition from black to white. (read at falling edge)
            count++;
            one_pulse = false;
        }
        if (adc_reading >= 1000) one_pulse = true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

float speed;
void encoder_getting_wheel_speed(void *arg)
{
    int time_one_rev, timenow = 0, time_taken;
    // float speed;
    // timenow = esp_timer_get_time();//get time in microseconds since boot
    while(1)
    {
        if (count == 7)
        {
            // one rev
            time_one_rev = esp_timer_get_time();
            // time taken to go one rev
            time_taken = time_one_rev - timenow;
            // find speed here:
            speed = (1.0/(time_taken/1000000.0))*(0.2136);
            printf("speed of wheel: %.2f\n",speed);
            //set old time to timenow
            timenow = time_one_rev;
            //reset count
            count = 0;
        }
    vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}

// void adc_reading(void){
//    #if CONFIG_IDF_TARGET_ESP32
//         //Check if Two Point or Vref are burned into eFuse
//         check_efuse();
//     #endif

//     //Configure ADC
//     if (unit == ADC_UNIT_1) {
//         adc1_config_width(ADC_WIDTH_BIT_12);
//         adc1_config_channel_atten(channel, atten);
//     } else {
//         adc2_config_channel_atten((adc2_channel_t)channel, atten);
//     }

//     #if CONFIG_IDF_TARGET_ESP32
//         //Characterize ADC
//         adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//         esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
//         print_char_val_type(val_type);
//     #endif


//     //Continuously sample ADC1
//     uint32_t previous_reading = 0;
//     while (1) {
//         uint32_t adc_reading = 0;
        
//         //Multisampling
//         for (int i = 0; i < NO_OF_SAMPLES; i++) {
//             if (unit == ADC_UNIT_1) {
//                 adc_reading += adc1_get_raw((adc1_channel_t)channel);
//             } else {
//                 int raw;
//                 adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
//                 adc_reading += raw;
//             }
//         }
//         adc_reading /= NO_OF_SAMPLES;
//         #if CONFIG_IDF_TARGET_ESP32
//             //Convert adc_reading to voltage in mV
//             uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
//             uint32_t previous_voltage_mv = esp_adc_cal_raw_to_voltage(previous_reading, adc_chars);

//             //if voltage goes above 1000mv then from white to black spike
//             //need to check that voltage is changing (moving) or same voltage as last reading (stopped)
//             if((voltage_mv > 1000) && (previous_voltage_mv != voltage_mv)){
//                 spikes = spikes + 1;
//             }
//             previous_reading = adc_reading;

//             //print value to console 
//             //printf("Voltage: %dmV\n", voltage_mv);
//         #elif CONFIG_IDF_TARGET_ESP32S2BETA
//                 printf("ADC%d CH%d Raw: %d\t\n", unit, channel, adc_reading);
//         #endif

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }   
// }

////////////////// TIMER //////////////////////////////////
// Define timer interrupt handler
void IRAM_ATTR timer_isr()
{
    // Clear interrupt
    TIMERG0.int_clr_timers.t0 = 1;

    // Indicate timer has fired and one second has passed 
    time_flag = 1;

    //reenable alarm 
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
}

void time_init(void){
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr,(void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}


////////////CALCULATING SPEED//////////////////////////
// float speed = 0; //this is used in alpha display task 
// void get_speed_task(void){
//     //if timer interrumps calculate speed with total number of rotations from pcnt value 
//     while(1){
//         if(time_flag == 1){
//             //reset time flag
//             time_flag = 0;

//             //calculate speed and print to console
//             //RPM X 60 (s/m) X wheel circumference (m)

//             //rev per sec, there are 7 spikes in 1 rev
//             float rps = spikes/7;
//             //float rpm = rps * 60;

//             speed = rps * 0.62; // m/s
//             printf("Speed: %.2f m/s\n", speed);

//             //reenable spikes count
//             spikes = 0;
//         }
//         vTaskDelay(10);
//     }
// }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* i2c display methods */
// initiate i2c
static void i2c_example_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_EXAMPLE_MASTER_RX_BUF_DISABLE, I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Turn on oscillator for alpha display
static int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
static int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
static int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main tasks of the program 

//turn on alphanumeric display
void i2c_task() 
{
    int ret;
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Write to characters to buffer
    uint16_t displaybuffer[8];

    // Display start speed of 0 m/s
    displaybuffer[3] = alphafonttable['0'];
    displaybuffer[2] = alphafonttable['0'];
    displaybuffer[1] = alphafonttable['0'];
    displaybuffer[0] = alphafonttable['0'];
    
    // Continually writes the same command
    while (1) {
        int speed2 = (int)(speed*100.0);
        
        displaybuffer[0] = alphafonttable[ ((speed2/1000) % 10)+ '0'];
        displaybuffer[2] = alphafonttable[ ((speed2/100) % 10)+ '0'];
        displaybuffer[2] = alphafonttable[ ((speed2/10) % 10) + '0'];
        displaybuffer[3] = alphafonttable[ (speed2 % 10)+ '0'];

        
        // Send commands characters to display over I2C
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i=0; i<8; i++) {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd4);

       vTaskDelay(100);
    }

}

void app_main(void) {

    //initiliaze adc 
    adc_init();
    //initialize i2c
    i2c_example_master_init();
    i2c_scanner();
    //INITILAIZE TIMER 
    time_init();

    // Create tasks
    // xTaskCreate(adc_reading,"adc_reading", 4096, NULL, 5, NULL);
    // xTaskCreate(get_speed_task,"get_speed_task", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_adc,"encoder_adc", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_getting_wheel_speed,"encoder_getting_wheel_speed", 4096, NULL, 5, NULL);

    // Create tasks
    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);

}
