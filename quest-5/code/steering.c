/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define DRIVE_MIN_PULSEWIDTH 900 //Minimum pulse width in microsecond
#define DRIVE_MAX_PULSEWIDTH 1900 //Maximum pulse width in microsecond
#define DRIVE_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define STEERING_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define STEERING_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define STEERING_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12);    //Set GPIO 12 as PWM0A, to which drive wheels are connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 13);    //Set GPIO 19 as PWM0A, to which steering servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t drive_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (DRIVE_MIN_PULSEWIDTH + (((DRIVE_MAX_PULSEWIDTH - DRIVE_MIN_PULSEWIDTH) * (degree_of_rotation)) / (DRIVE_MAX_DEGREE)));
    return cal_pulsewidth;
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t steering_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (STEERING_MIN_PULSEWIDTH + (((STEERING_MAX_PULSEWIDTH - STEERING_MIN_PULSEWIDTH) * (degree_of_rotation)) / (STEERING_MAX_DEGREE)));
    return cal_pulsewidth;
}

void pwm_init() {
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

//void calibrateESC() {
//    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
//    printf("waited 3 seconds \n");
//    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds - backwards
//    printf("Configured HIGH signal 2100 \n");
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds - forwards
//    printf("Configured LOW signal 700 \n");
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
//    printf("Configured NEUTRAL signal 1400 \n");
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
//    printf("Reset to NEUTRAL value \n");
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
//}

/**
 * @brief Configure MCPWM module
 */
//void drive_control(void *arg)
//{
//    uint32_t angle, count;
//
//    //while (1) {
//        //for (count = 0; count < DRIVE_MAX_DEGREE; count++) {
//        //count = 180;
//        //    printf("Angle of rotation: %d\n", count);
//        //    angle = drive_per_degree_init(count);
//        //    printf("pulse width: %dus\n", angle);
//    for (count = 1400; count > 1200; count -= 5) {
//        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
//        printf("Count: %d\n",count);
//        vTaskDelay(100/portTICK_RATE_MS);
//    }
//    printf("\n-----------------------------------------------");
//
//    for (count = 1200; count < 1600; count += 5) {
//        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
//        printf("Count: %d\n",count);
//        vTaskDelay(100/portTICK_RATE_MS);
//    }
//    printf("\n-----------------------------------------------");
//
//    for (count = 1600; count >= 1400; count -= 5) {
//        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
//        printf("Count: %d\n",count);
//        vTaskDelay(100/portTICK_RATE_MS);
//    }
//    printf("\n-----------------------------------------------");
//
//    //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);
//    //vTaskDelay(1000/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
//    vTaskDelay(100/portTICK_RATE_MS);
//
//
//        //}
//    //}
//    vTaskDelete(NULL);
//}

/**
 * @brief Configure MCPWM module
 */
void straight(){
    uint32_t angle, count;
                count = 90;
                angle = steering_per_degree_init(count);
               
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                //vTaskDelay(100/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
            }
//
//}
//
void left(){
                uint32_t angle, count;
                count = 180;
                angle = steering_per_degree_init(count);
               
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                //vTaskDelay(100/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
            }

void right(){
                uint32_t angle, count;
                count = 0;
                angle = steering_per_degree_init(count);
               
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
                //vTaskDelay(100/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
            }


void steering_control(void *arg)
{
    uint32_t angle, count;

    while (1) {

//        int left = 0;
//        int right = 0;
//        int straight = 90;
//
        left();
        vTaskDelay(100);
        right();
        vTaskDelay(100);
        straight();
        vTaskDelay(100);
//        angle = steering_per_degree_init(180);
//        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
//        vTaskDelay(3000/portTICK_RATE_MS);
//        angle = steering_per_degree_init(0);
//        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
//        vTaskDelay(3000/portTICK_RATE_MS);
//        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, right);
//        vTaskDelay(300/portTICK_RATE_MS);
//
//            for (count = 0; count < STEERING_MAX_DEGREE; count += 90) {
//                //count = 90;
//                printf("Angle of rotation: %d\n", count);
//                angle = steering_per_degree_init(count);
//                printf("pulse width: %dus\n", angle);
//                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
//                vTaskDelay(2000/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//            }


    }
}

void app_main(void)
{

    pwm_init();
//    calibrateESC();
//    while(1){
//
//        uint32_t count;
//    for (count = 1400; count > 1200; count -= 5) {
//        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
//        printf("Count: %d\n",count);
//        vTaskDelay(100/portTICK_RATE_MS);
//    }
//
//        //vTaskDelay(2000);
//        for (count = 1200; count < 1600; count += 5) {
//            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
//            printf("Count: %d\n",count);
//            vTaskDelay(100/portTICK_RATE_MS);
//        }
//    }


//    uint32_t angle, count;
//
//    while (1) {
//
//        for (count = 0; count < STEERING_MAX_DEGREE; count++) {
//            //count = 90;
//            printf("Angle of rotation: %d\n", count);
//            angle = steering_per_degree_init(count);
//            printf("pulse width: %dus\n", angle);
//            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
//            vTaskDelay(100/portTICK_RATE_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//        }
//
//        vTaskDelay(100/portTICK_RATE_MS);
//    }
    printf("Testing servo motor.......\n");
    xTaskCreate(steering_control, "steering_control", 4096, NULL, 5, NULL);
    //xTaskCreate(drive_control, "drive_control", 4096, NULL, 4, NULL);

}


