//Carmen Hurtado Code adapted from exmaple in class github repo 

//STD C library
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include <math.h>

//RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

//GPIO task
#include "driver/gpio.h"

//esp timer
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"


//timer definitions
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   0.1   // 100 ms 
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

//mcpwm library
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//ADC task
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES_ULTRA   10         //Multisampling
#define NO_OF_SAMPLES_IR   50                //Multisampling
#define NO_OF_SAMPLES   50                //Multisampling




#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

/////////////////////////////FUNCTIONS AND CONFIG FOR STEERING SERVO////////////////////////////////////////
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 27);    //Steering servo
}

void servo_init(void){
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

}

//calculate degree
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void steering_servo_task(void *arg)
{
    uint32_t angle, count;
    while (1) {
        for (count = 0; count < SERVO_MAX_DEGREE; count++) {
            printf("Angle of rotation: %d\n", count);
            angle = servo_per_degree_init(count);
            printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}


//Steer Right 
void SteerRight(){
    uint32_t angle, count;
    for (count = 90; count > 0; count--) {
        //printf("Angle of rotation: %d\n", count);
        angle = servo_per_degree_init(count);
        //printf("pulse width: %dus\n", angle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
        vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    }
}

//Steer Left 
void SteerLeft(){
    uint32_t angle, count;
    for (count = 90; count < 180; count++) {
        //printf("Angle of rotation: %d\n", count);
        angle = servo_per_degree_init(count);
        //printf("pulse width: %dus\n", angle);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
        vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    }
}


/////////////////////////////FUNCTIONS AND CONFIG FOR IR SENSOR////////////////////////////////////////
// init atten variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//IR left
static const adc_channel_t channel3 = ADC_CHANNEL_5;     //GPIO33 width=10
//IR right
static const adc_channel_t channel4 = ADC_CHANNEL_3;     //GPIO39

// adc init 
static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
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
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
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
void adc_init()
{
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        // adc1_config_channel_atten(channel1, atten);
        // adc1_config_channel_atten(channel2, atten);
        adc1_config_channel_atten(channel3, atten);
        adc1_config_channel_atten(channel4, atten);
    } else {
        // adc2_config_channel_atten((adc2_channel_t)channel1, atten);
        // adc2_config_channel_atten((adc2_channel_t)channel2, atten);
        adc2_config_channel_atten((adc2_channel_t)channel3, atten);
        adc2_config_channel_atten((adc2_channel_t)channel4, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

//helper function for find_distance_ir
float range_finder(int voltage)
{
    float inverse_distance = 0.0, c, m;
    if (voltage <=2000 && voltage > 400)
    {
        c = -0.8583691;
        m = 60085.83691;
        inverse_distance = ((float)voltage - c)/m;
    }
    else if (voltage <= 2500 && voltage > 2000)
    {
        c = 903.293413;
        m = 32934.13174;
        inverse_distance = ((float)voltage - c)/m;
    }
    else if (voltage <= 2750 && voltage > 2500)
    {
        c = 1950.119976;
        m = 11997.60048;
        inverse_distance = ((float)voltage - c)/m;
    }
    else if (voltage > 2750)
    {
        return 150;
    }
    else if (voltage <=400)
    {
        return 15;
    }
    return 1.0/inverse_distance;
}

//function for left IR 
float find_distance_ir_left()
{
    float adc_reading = 0.0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES_IR; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel3);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel3, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
        // vTaskDelay(pdMS_TO_TICKS(40)); //sense every 40ms, 50 samples -> 2s intervals
    }
    adc_reading /= NO_OF_SAMPLES_IR;
    //Convert adc_reading to voltage in mV
    float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    // uint32_t distance = 5 * adc_reading;
    float distance = range_finder(voltage);
    return distance/100.0;
}

//function for right IR
float find_distance_ir_right()
{
    float adc_reading = 0.0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES_IR; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel4);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel4, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
        // vTaskDelay(pdMS_TO_TICKS(40)); //sense every 40ms, 50 samples -> 2s intervals
    }
    adc_reading /= NO_OF_SAMPLES_IR;
    //Convert adc_reading to voltage in mV
    float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    // uint32_t distance = 5 * adc_reading;
    float distance = range_finder(voltage);
    return distance/100.0;
}

void one_cycle_read()
{
    //header for csv/nodejs
    printf("IR left (cm), IR Right\n");
    while(1)
    {
        
        float ir_left = find_distance_ir_left();
        float ir_right = find_distance_ir_right();
        printf("%.2f, %.2f\n", ir_left, ir_right);

        if(ir_left <= 0.25){
            SteerRight();
        }
        if(ir_right <= 0.25){
            SteerLeft();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void){

  //Initialize
  servo_init();
  adc_init();

  //timer init
  //periodic_timer_init();


  // Create task to poll ADXL343
  xTaskCreate(one_cycle_read,"one_cycle_read", 4096, NULL, 5, NULL);
}
