// standard library
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include <math.h>

//mcpwm library
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//adc library for encoder
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

//esp timer
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"

//i2c adxl
#include "driver/i2c.h"
#include "./ADXL343.h"

//i2c PID
// freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// timer
#include "driver/timer.h"


////////////////////////////////////////////////////////////////////////////////
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

// ADXL343 addresses
#define slave_addr_adxl                         ADXL343_ADDRESS // 0x53
// LIDAR addresses
#define SLAVE_ADDR                         0x62 //7-bit slave address with default value
#define REGISTER_READ                      0X00 // register to write to initiate ranging
#define MEASURE_VALUE                      0x04 // Value to initiate ranging
#define HIGH_LOW                           0x8f //for multi-byte read

// Timer define
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval is 0.1s
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 700 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

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
//global variables for PID
int dt_complete = 0; // for timer interrupt
int dt = 1; //delta time for PID (0.1s)
float setpoint = 50.0; // 50 cm
float previous_error = 0.0; //previous error for PID
float integral = 0.0; // integral term 
float derivative; // derivative term
// Distance detected from LIDAR
float distance;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// timer init
// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {
    // Prepare basic event data, aka set flag
    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;
    // indicate timer has fired
    dt_complete  = 1;
    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
    // Send the event data back to the main program task
}
void timer_test_init()
{
// timer initialization
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
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// i2c init
static void i2c_master_init(){
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
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
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
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//ADC init 
static void check_efuse(void)
{
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// MCPWM init
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 27);    //Set GPIO 27 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 12);    //Set GPIO 12 as PWM0B, to ESC is connected
}

void calibrateESC() {
    printf("Crawler on in 3seconds\n");       
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
    printf("Crawler neutral\n"); 
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1200); // NEUTRAL signal in microseconds
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    printf("Crawler neutral now, calibration done\n");
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID_adxl(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( slave_addr_adxl << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( slave_addr_adxl << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register (single byte write)
void writeRegister_adxl(uint8_t reg, uint8_t data) {
  // create i2c communication init
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);	// 1. Start (Master write start)
  i2c_master_write_byte(cmd, ( slave_addr_adxl << 1 ) | WRITE_BIT, ACK_CHECK_EN); // (Master write slave add + write bit)
  // wait for salve to ack
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // (Master write register address)
  // wait for slave to ack
  i2c_master_write_byte(cmd, data, ACK_CHECK_DIS);// master write data 
  // wait for slave to ack
  i2c_master_stop(cmd); // 11. Stop
  // i2c communication done and delete
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // no return here since data is written by master onto slave
}

// Read register (single byte read)
uint8_t readRegister_adxl(uint8_t reg) {
  uint8_t data;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // 1. Start
  i2c_master_start(cmd);
  // Master write slave address + write bit)
  i2c_master_write_byte(cmd, ( slave_addr_adxl << 1 ) | WRITE_BIT, ACK_CHECK_EN); 
  // Master write register address
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); 
  // Start master
  i2c_master_start(cmd);
  // Master write slave address + read bit
  i2c_master_write_byte(cmd, ( slave_addr_adxl << 1 ) | READ_BIT, ACK_CHECK_EN);  
  // Master read in slave ack and data
  i2c_master_read_byte(cmd, &data , ACK_CHECK_DIS);
  // Master nacks and stops.
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); // This starts the I2C communication
  i2c_cmd_link_delete(cmd);
  return data;
}

// read 16 bits (2 bytes) (multiple-byte read)
int16_t read16_adxl(uint8_t reg) {
  int16_t two_byte_data;
  uint8_t second_data;
  // Read X0, Y0 or Z0
  uint8_t first_data = readRegister_adxl(reg);
  // read X1, y1 or Z1
  second_data = readRegister_adxl(reg+1);
  // combine two 8 bits by left shifting second data.
  // then bitwise OR (if either data has 1, then two_byte_data has 1)
  two_byte_data = (second_data << 8 | first_data);
  return two_byte_data;
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister_adxl(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F; //0x0000 1111
  format |= range; //bits that are set of 1 in range will be set to one in format
  // bits that are set to 0 in range will be unchanged in format.

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;
  /* Write the register back to the IC */
  writeRegister_adxl(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister_adxl(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister_adxl(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// LIDAR V3 Functions ///////////////////////////////////////////////////////////

// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start command
    i2c_master_start(cmd);
    //slave address followed by write bit
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //register pointer sent
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //data sent
    i2c_master_write_byte(cmd, data, ACK_CHECK_DIS);
    //stop command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

// Read register
uint16_t readRegister(uint8_t reg) {
    int ret;
    uint8_t value;
    uint8_t value2;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    //start command
    i2c_master_start(cmd);
    //sensor address, write, ack
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //register address ack
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //stop
    i2c_master_stop(cmd);
    //stop condition 
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);

    //start command for cmd2
    i2c_master_start(cmd2);
    //sensor address, read, ack
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    //data out high byte #1
    i2c_master_read_byte(cmd2, &value, ACK_CHECK_DIS);
    //data out low byte #2
    i2c_master_read_byte(cmd2, &value2, ACK_CHECK_DIS);
    //stop
    i2c_master_stop(cmd2);
    //ret is device ID
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    //delete two i2c comm packets
    i2c_cmd_link_delete(cmd2);
    i2c_cmd_link_delete(cmd);
    // printf("value: %d, value2: %d \n", value, value2);
    return (uint16_t)(value<<8|value2);
}

////////////////////////////////////////////////////////////////////////////////
// Master init
void master_init()
{
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

    //3. i2c init
    // Function to initiate i2c -- note the MSB declaration!
    i2c_master_init();
    i2c_scanner();
    // Init ADXL ////////////////////////////////////////////////
    uint8_t deviceID;
    getDeviceID_adxl(&deviceID);
    // Disable interrupts
    writeRegister_adxl(ADXL343_REG_INT_ENABLE, 0);
    // Set range
    setRange(ADXL343_RANGE_2_G);
    // Enable measurements
    writeRegister_adxl(ADXL343_REG_POWER_CTL, 0x08);

    //4. ADC init
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //timer init
    timer_test_init();
}
////////////////////////////////////////////////////////////////////////////////
// Below are tasks
////////////////////////////////////////////////////////////////////////////////

// Driving servo: takes input from webpage and drive the buggy - need to add in code
void driving_servo(void *arg)
{
    // takes
    int count_speed;
    while(1)
    {
        //code goes here.
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}
////////////////////////////////////////////////////////////////////////////////

// void steering_servo(void *arg)
// {
//    int count, angle;
//     while (1) {
//         for (count = 0; count < SERVO_MAX_DEGREE; count++) {
//             angle = servo_per_degree_init(count);
//             mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
//             vTaskDelay(100 / portTICK_PERIOD_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//         }
//         for (count = SERVO_MAX_DEGREE; count > 0; count--) {
//             angle = servo_per_degree_init(count);
//             mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
//             vTaskDelay(100 / portTICK_PERIOD_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
//         }
//         vTaskDelay(1000 / portTICK_PERIOD_MS); 
//     }
// }

////////////////////////////////////////////////////////////////////////////////
//adxl wheel speed
////////////////////////////////////////////////////////////////////////////////
// ADXL343 Functions ///////////////////////////////////////////////////////////

float calcRoll(float x, float y, float z){
  float roll = atan2(y, z) * 57.3;
  return roll;
}
float calcPitch(float x, float y, float z){
  float pitch = atan2((-x), sqrt(y * y + z * z)) * 57.3;
  return pitch;
}
void getAccel(float * xp, float *yp, float *zp) {
  *xp = read16_adxl(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = read16_adxl(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = read16_adxl(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// Task to continuously poll acceleration and calculate roll and pitch
void adxl343() {
// Check for ADXL343
  uint8_t deviceID;
  getDeviceID_adxl(&deviceID);
  if (deviceID == 0xE5) {
    printf("\n>> Found ADAXL343\n");
  }
  // Disable interrupts
  writeRegister_adxl(ADXL343_REG_INT_ENABLE, 0);
    // Enable measurements
  writeRegister_adxl(ADXL343_REG_POWER_CTL, 0x08);

  printf("\n>> Polling ADAXL343\n");
  while (1) {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

////////////////////////////////////////////////////////////////////////////////
// LIDAR 
////////////////////////////////////////////////////////////////////////////////
static void test_lidar() {
  printf("\n>> Polling Lidar\n");
  while (1) {
    uint8_t reg = REGISTER_READ;
    uint8_t data = MEASURE_VALUE;
    writeRegister(reg,data);
    // continously read register 0x01 until first bit (LSB) goes 0
    int compare = 1;
    while(compare)
    {
      uint8_t reading = readRegister(0x01);
      compare = reading&(1<<7);
      // printf("Reading: %d\n", reading);
      vTaskDelay(5);
    }
    distance = (float)(readRegister(HIGH_LOW));
    printf("Distance: %f\n", distance);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

////////////////////////////////////////////////////////////////////////////////
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

void encoder_getting_wheel_speed(void *arg)
{
    int time_one_rev, timenow = 0, time_taken;
    float speed;
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
////////////////////////////////////////////////////////////////////////////////


void app_main(void)
{
    master_init();
    calibrateESC();

    printf("Testing RC car.......\n");
    // xTaskCreate(steering_servo, "steering_servo", 4096, NULL, 5, NULL);
    xTaskCreate(driving_servo,"driving_servo", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_adc,"encoder_adc", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_getting_wheel_speed,"encoder_getting_wheel_speed", 4096, NULL, 5, NULL);
    xTaskCreate(adxl343,"adxl343_speed",4096,NULL,5,NULL);
    xTaskCreate(test_lidar,"test_lidar", 4096, NULL, 5, NULL);
}
