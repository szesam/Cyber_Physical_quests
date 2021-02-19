// Fishtank.c - ESP32 script that combines 14-segment Adafruit display, timer and servo motor to feed fish at fixed time interval. 
// Interval given by user through initial console IO.

/* Carmen Hurtado and Samuel Sze Quest 1 for EC444 Spring 2021
Code adapted from examples of the class github repo */ 

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

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2300//Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

//UART DEFINITIONS 
#define EX_UART_NUM UART_NUM_0
#define ECHO_TEST_TXD  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RXD  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

uint16_t user_time = 0;
uint16_t time_counter = 0;
uint16_t i2c_flag = 1;
int servo_flag = 0;

/* Timer methods */
// A simple structure to pass timer "events" to main task
typedef struct {
    int flag;    
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {
    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;
    
    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers(this case the timer reaches zero), we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

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

/* Main init method */
static void init() {
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
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);

    //initialize servo
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 26);   
    //initial mcpwm configuration
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    //call the initialize i2c method 
    i2c_example_master_init();

}

/*Servo methods*/
// calculate pulse width 
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


/*UART IO  methods*/
//init for uart IO 
void uart_init(){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0); 
}

//Console Io to get user input time
void get_time(){

    int hours;
    int minutes;

    // Configure a temporary buffer for the incoming data
    uint8_t *data_hr = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *data_mn = (uint8_t *) malloc(BUF_SIZE);
    while(1){
        printf("Enter hours desired for fish feeding: \n ");

        //array of chars for storing user input 
        char hr_in[4] = {};
        //boolean value to know if user is done typing 
        int got_hr = 0;
        int position_hr = 0;
        //keep reading characters(nums) until pressed enter, assumption: length(string) < 4
        //while we haven't got hour
        while (got_hr == 0){ 
            int len_hr = uart_read_bytes(EX_UART_NUM, data_hr, BUF_SIZE, 20 / portTICK_RATE_MS);
            if (len_hr > 0){
                //if user presses enter key
                if (*data_hr == '\r'){
                    // set got hr to true and stop loop
                    got_hr = 1;
                }
                //else add character to string of chars to form output
                else{
                    hr_in[position_hr] = *data_hr;
                    position_hr = position_hr + 1;
                }
            }
        }
        
        hours = atoi(hr_in);  
        printf("%d\n", hours);


        printf("Enter minutes desired for fish feeding: \n");
        //array of chars for storing user input 
        char mn_in[4] = {};
        //boolean value to know if user is done typing 
        int got_mn = 0;
        int position_mn = 0;
        //keep reading characters(nums) until pressed enter, assumption: length(string) < 4
        //while we haven't got minutes
        while (got_mn == 0){ 
            int len_mn = uart_read_bytes(EX_UART_NUM, data_mn, BUF_SIZE, 20 / portTICK_RATE_MS);

            if (len_mn > 0){
                //if user presses enter key
                if (*data_mn == '\r'){
                    // set got mn to true and stop loop
                    got_mn = 1;
                }
                //else add character to string of chars to form output
                else{
                    mn_in[position_mn] = *data_mn;
                    position_mn = position_mn + 1;
                }
            }
        }
        minutes = atoi(mn_in); 
        printf("%d\n", minutes);

        //initialize the total time counter depending on user input 
        user_time = (uint16_t)(hours * 3600 + minutes * 60);
        printf("%d\n",user_time);
        time_counter = user_time;
        return; 

    }   
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main tasks of the program 

// The timer task 
void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        timer_event_t evt;
        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        if (evt.flag == 1 && time_counter != 0)
        {
            //decrement timer as time goes by 
            time_counter--;
            //print updated time 
            printf("interrupted: time_counter: %d, user_time = %d\n", time_counter, user_time);
        }
        if (time_counter == 0)
        {
            //if timer reaches zero turn on flag for servo to start spinning and toggle flag for displaying "FEED"
            servo_flag = 1;
            i2c_flag = 0;
            printf("stopped interrupt: time_counter = %d, user_time = %d, servo_flag = %d\n", time_counter, user_time,servo_flag);
        }
        vTaskDelay(100);
    }
}

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
    
    // Continually writes the same command
    while (1) {
        //if timer reached zero display command that feeding is on 
        if (i2c_flag == 0) 
        {
            // Display feed 
            displaybuffer[3] = alphafonttable['D'];
            displaybuffer[2] = alphafonttable['E'];
            displaybuffer[1] = alphafonttable['E'];
            displaybuffer[0] = alphafonttable['F'];
            continue;
        }
        if (i2c_flag == 1)
        {
            //calculkate current time to display 
            int h = time_counter/3600;
            int m = (time_counter - (3600*h))/60;
            int s = (time_counter - (3600*h) - (m*60));
            
            if((m >= 0) && (h>=0)){
                displaybuffer[3] = alphafonttable[m%10 + '0'];
                displaybuffer[2] = alphafonttable[((m/10)%10) + '0'];
                displaybuffer[1] = alphafonttable[h%10 + '0'];
                displaybuffer[0] = alphafonttable[((h/10)%10) + '0'];
            }
            //if there are only seconds left show seconds instead of hours and minutes 
            if((m <= 0) && (h <= 0)){
                displaybuffer[3] = alphafonttable[s%10 + '0'];
                displaybuffer[2] = alphafonttable[((s/10)%10) + '0'];
                displaybuffer[1] = alphafonttable['0'];
                displaybuffer[0] = alphafonttable['0'];
            }
            printf("Remaining time: %d hours, %d minutes, %d seconds\n", h,m,s);

        }
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

// servo task
void servo_task(void *arg)
{
    uint32_t angle, count;
    while (1) {
        //once servo gets activated 
        if (servo_flag == 1)
        {
            mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
            //rotate left and right 180 degrees 3 times to dispense food 
            for(int i = 1; i < 4; i++){
                for (count = 0; count < SERVO_MAX_DEGREE; count++) {
                angle = servo_per_degree_init(count);
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
                //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
                vTaskDelay(3);     
                }
                for (count = SERVO_MAX_DEGREE - 1; count > 0 ; count--) {
                    // printf("Angle of rotation: %d\n", count);
                    angle = servo_per_degree_init(count);
                    // printf("pulse width: %dus\n", angle);
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
                    //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
                    vTaskDelay(3);     
                }
            }
            //reset servo flag to start the countdwon again 
            servo_flag = 0;
            i2c_flag = 1;
            time_counter = user_time;
            printf("did it reach here? time_counter = %d, usertime = %d \n",time_counter, user_time);
            mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        }
        vTaskDelay(100);
    }
}

void app_main(void) {
    // Create a FIFO queue for timer-based
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Console IO to get user inputted time 
    // initialize uart driver for console io
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    get_time();
    // Initiate alarm using timer API, initialize i2c, init servo
    init();
    // Create task to handle timer-based events
    xTaskCreate(timer_evt_task, "timer_evt_task", 4096, NULL, 6, NULL);
    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);
    xTaskCreate(servo_task, "servo_task", 4096, NULL, 5, NULL);
}
