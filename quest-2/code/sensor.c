/*Carmen Hurtado and Samuel Sze 03-05-2021 
EC444 Quest 2: Tactile Internet
Code adapted from ESP-IDF example for ADC*/

//Standard C library
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//RTOS library
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//GPIO library
#include "driver/gpio.h"
//ADC library
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES_ULTRA   10         //Multisampling
#define NO_OF_SAMPLES_IR   50                //Multisampling
#define NO_OF_SAMPLES   50                //Multisampling

// init atten variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//battery
static const adc_channel_t channel1 = ADC_CHANNEL_4;     //GPIO32 
//thermistor
static const adc_channel_t channel2 = ADC_CHANNEL_6;     //GPIO34
//ultrasonic
static const adc_channel_t channel3 = ADC_CHANNEL_5;     //GPIO33 width=10
//rangefinder
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
void init()
{
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel1, atten);
        adc1_config_channel_atten(channel2, atten);
        adc1_config_channel_atten(channel3, atten);
        adc1_config_channel_atten(channel4, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel1, atten);
        adc2_config_channel_atten((adc2_channel_t)channel2, atten);
        adc2_config_channel_atten((adc2_channel_t)channel3, atten);
        adc2_config_channel_atten((adc2_channel_t)channel4, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}


//task functions from one_cycle_read
float find_voltage()
{
    float adc_reading = 0.0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel1);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel1, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES; // find adc_reading average
    //Convert adc_reading to voltage in mV
    float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage;
}
float find_temperature()
{
    float beta = 3435.0, R = 10000.0, adc_reading = 0.0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel2);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES; // find adc_reading average
    //Convert adc_reading to voltage in mV
    float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    // find resistance over thermistor
    float R0 = 10000 * (3300/voltage-1);
    // convert resistance in thermistor to temperature in celsius.
    float one_over_T = 1.0/298.15 + 1/beta * log(R/R0);
    return 1/one_over_T - 273.15;
}
float find_distance_ultrasonic()
{
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES_ULTRA; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel3);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel3, ADC_WIDTH_BIT_10, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES_ULTRA;
    //calculating the voltage scaling factor 
    float input_voltage_v = 3.3;
    //value from data specification sheet 
    float volts_per_inch = 512;
    float scaling_factor_mv = (input_voltage_v/volts_per_inch)*1000;

    //Convert adc_reading to voltage in mV
    float voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    //calculate distance in inches
    float distance_in = voltage_mv/scaling_factor_mv;
    //change units to meters
    float distance_m = distance_in * 0.0254;
    return distance_m;
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
float find_distance_ir()
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

//task function from app main
void one_cycle_read()
{
    //header for csv/nodejs
    printf("Battery voltage (mV), temperature (C), ultrasonic distance (cm), IR distance (cm)\n");
    while(1)
    {
        float battery_voltage = find_voltage();
        float thermistor = find_temperature();
        float ultrasonic = find_distance_ultrasonic();
        float ir = find_distance_ir();
        printf("%.2f, %.2f, %.2f, %.2f\n", battery_voltage, thermistor, ultrasonic, ir);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    //configure each adc
    init();
    //one cycle involves displaying data from each sensor once and printing onto console
    xTaskCreate(one_cycle_read, "one_cycle_read", 4096, NULL, 6, NULL); 
}