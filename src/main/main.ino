/**
 * @name: Rocket ESP-32 main file (main.ino)
 * 
 * @authors: Pedro Andrade
 * Universidade do Minho
 * 2024
 * 
 * @brief: Defines the running finite state machine for the three phases: 
 * idle (setup + verification), ready (reading + saving data), done (save file, sleep)
 * 
 * @pinout: 
 * BMI160 <-> I2C
 * SD     <-> VSPI
 * SG     <-> GPIO4
*/

#include "DFRobot_BMI160.h"
#include "SDCard.h"
#include <math.h>

#define PI 3.14
#define G 9.81

#define PARACHUTED_TAG_NUMBER 49855

#define MAX_DC 255

// LED PWM parameters
#define LED_PIN     2
#define LED_CH      0
#define LED_RES     8
#define LED_FREQ    4        // freq_min

// SG92R PWM parameters
#define SG92R_PIN   4
#define SG92R_CH    8
#define SG92R_RES   8
#define SG92R_FREQ  50

// BMI160 imu parameters
#define BMI_I2C_ADDR 0x69
#define GYRO_SENS 65.6
#define GYRO_THRESH 0.05
#define ACC_SENS 2048.0
#define ACC_THRESH 0.05

// SD Card parameters
#define FILE_NAME MOUNT_POINT"/data.csv"

// state parameters
#define SAFE_IDLE_TIME_PRE_S 10
#define SAFE_IDLE_TIME_POST_S 120
#define BLINK_PERIOD_S 4
#define STOP_SAVING_AFTER_PARACHUTED_S 20
#define SAFE_DOWN_VEL_S 3
#define N_OFFSET_SAMPLES 500
#define N_SAMPLES_WITHOUT_CHANGE 40
#define ALPHA 0.8

// define FSM states
enum State {
  IDLE,       // FULL
  READY,      // WAVE
  DONE        // SHORT BLINK
} currentState = IDLE;

// create tag for logging
static const char *TAG = "main";

DFRobot_BMI160 bmi160;

unsigned long last_millis = millis();
uint32_t timestamp[2] = {0};
int16_t accelGyro[6] = {0};
float offset[6] = {0};
float acc[3] = {0};
float gyro[3] = {0};
float gx_e = 0.0, gy_e = 0.0, gz_e = 0.0, ax_e = 0.0, ay_e = 0.0, az_e = 0.0;
float last_vy = 0, ay = 0, vy = 0, py = 0;
float pitch = 0, yaw = 0;
bool first = 1;
int counter = 0;
unsigned long timestep = millis(), last_timestep = millis(), parachuted = 0, t = 0;







void setup()
{
  ESP_LOGI(TAG, "Starting in IDLE state");

  // configure servo PWM functionalitites
  ledcSetup(SG92R_CH, SG92R_FREQ, SG92R_RES);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(SG92R_PIN, SG92R_CH);

  // set PWM dutycycle [0-255]
  ledcWrite(SG92R_CH, MAX_DC * 0.025);

  // configure LED PWM functionalitites
  ledcSetup(LED_CH, LED_FREQ, LED_RES);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(LED_PIN, LED_CH);

  // set PWM dutycycle
  ledcWrite(LED_CH, MAX_DC); // always ON
  
  // init sd card
  sdcard_init();

  delay(100);

  // open data file
  if(!sdcard_openFile(FILE_NAME))
  {
    ESP_LOGE("ERR_SD_OPEN", "SD file open failed");
    ledcWrite(LED_CH, 0);
    while(1);
  }

  // write header
  sdcard_writeFile("Timestamp [ms], Yaw_Z [º], Pitch_X [º], Acc_Y [m/s^2], Vel_Y [m/s], Pos_Y [m]\n");

  delay(100);

  // init the BMI160 hardware, check DFRobot_BMI160::setSensConf method for mode operation settings
  if (bmi160.softReset() != BMI160_OK)
  {
    ESP_LOGE("ERR_BMI160_SR", "Soft reset failed");
    ledcWrite(LED_CH, 0);
    while(1); // return or enable error flag
  }

  delay(100);
  
  // set and init the BMI160 I2C address
  if (bmi160.I2cInit(BMI_I2C_ADDR) != BMI160_OK)
  {
    ESP_LOGE("ERR_BMI160_I2C", "I2C init failed");
    ledcWrite(LED_CH, 0);
    while(1);
  }

  ESP_LOGI(TAG, "Device initialized");

  Serial.begin(115200);
}









void loop()
{
  // handle state transitions
  switch (currentState)
  {
    // IDLE -> READY
    case IDLE: 
      if((millis() - last_millis >= SAFE_IDLE_TIME_PRE_S * 1000) && !first)
      {
        last_millis = millis();
        currentState = READY;
        first = 1;
        // placed here to avoid repetition in READY loop (to increase sampling frequency)
        ledcWrite(LED_CH, MAX_DC * 0.5); // 50% blink
        ESP_LOGI(TAG, "Switching to READY state");
      }
      break;

    // READY -> DONE
    case READY: 
      if((millis() - parachuted >= STOP_SAVING_AFTER_PARACHUTED_S * 1000) && parachuted)
      {
        last_millis = millis();
        currentState = DONE;
        ESP_LOGI(TAG, "Switching to DONE state");
      }
      break;
    
    /*
    case DONE:
      break;
    */
  }

  // perform actions based on the current state
  switch (currentState)
  {
    case IDLE:
    {
      if(first)
      {
        first = 0;

        delay(SAFE_IDLE_TIME_POST_S * 1000);

        for (int j = 0; j < N_OFFSET_SAMPLES; j++)
        {
          if(!bmi160.getAccelGyroData(accelGyro))
          {
            for (int i = 0; i < 3; i++)
              offset[i] += accelGyro[i]/GYRO_SENS;
            
            for(int i = 3; i < 6; i++)
              offset[i] += accelGyro[i]/ACC_SENS;
          }
        }    

        for(int i = 0; i < 6; i++)
        {
          offset[i] /= N_OFFSET_SAMPLES;
          //Serial.println(offset[i]);
        }    
      }

      break;
    }
      
    
    case READY:
    {
      timestep = millis();
      float dt = (timestep - last_timestep) * 0.001;

      // get both accel and gyro data from bmi160
      // parameter accelGyro is the pointer to store the data
      if(!bmi160.getAccelGyroData(accelGyro, timestamp))
      {
        // calibrate bias error
        for (int i = 0; i < 3; i++)
        {
          gyro[i] = accelGyro[i]/GYRO_SENS - offset[i];
          acc[i] = accelGyro[i+3]/ACC_SENS - offset[i+3];
        } 

        // reduce mean error
        if (acc[0] <= ACC_THRESH && acc[0] >= -ACC_THRESH)
          acc[0] = 0.0;
        if (acc[1] <= ACC_THRESH && acc[1] >= -ACC_THRESH)
          acc[1] = 0.0;
        if (acc[2] <= ACC_THRESH && acc[2] >= -ACC_THRESH)
          acc[2] = 0.0;
        if (gyro[0] <= GYRO_THRESH && gyro[0] >= -GYRO_THRESH)
          gyro[0] = 0.0;
        if (gyro[1] <= GYRO_THRESH && gyro[1] >= -GYRO_THRESH)
          gyro[1] = 0.0;
        if (gyro[2] <= GYRO_THRESH && gyro[2] >= -GYRO_THRESH)
          gyro[2] = 0.0;
        

        // low-pass filter on acc + gyro measurements
        gx_e = (1-ALPHA)*(-gyro[0]) + ALPHA*gx_e;
        gy_e = (1-ALPHA)*(-gyro[1]) + ALPHA*gy_e;
        gz_e = (1-ALPHA)*(-gyro[2]) + ALPHA*gz_e;
        ax_e = (1-ALPHA)*(-acc[0]) + ALPHA*ax_e;
        ay_e = (1-ALPHA)*(-acc[1]) + ALPHA*ay_e;
        az_e = (1-ALPHA)*(-acc[2]) + ALPHA*az_e;

        // update rotation
        pitch += dt * gx_e;
        yaw += dt * gz_e;

        // update vertical motion
        ay = ax_e * sin(yaw * PI / 180) * cos(pitch * PI / 180) 
           + ay_e * cos(yaw * PI / 180) * cos(pitch * PI / 180) 
           + az_e * sin(pitch * PI / 180);

        vy += dt * ay * G;
        py += vy * dt + 0.5 * pow(dt, 2) * ay * G;

        // detect descending movement and activate parachute
        if(vy < -0.001)
        {
          Serial.println(vy);
          if(first)
          {
            first = 0;
            t = millis();
          }
          else
            if((millis() - t >= SAFE_DOWN_VEL_S * 1000) && !parachuted)
            {
              parachuted = millis();
              sdcard_writeFile(parachuted, 
                               PARACHUTED_TAG_NUMBER, 
                               PARACHUTED_TAG_NUMBER, 
                               PARACHUTED_TAG_NUMBER, 
                               PARACHUTED_TAG_NUMBER, 
                               py);
              
              for(float dc = MAX_DC * 0.025; dc < MAX_DC * 0.125; dc++)
              {
                ledcWrite(SG92R_CH, dc);
                delay(20);
              }
            }
        }
        else
        {
          first = 1;
        }
        
        // reset velocity if it's constant over a long period of time (reset sensor drift)
        if(last_vy == vy)
        {
          if(++counter >= N_SAMPLES_WITHOUT_CHANGE)
          {
            vy = pitch = yaw = 0.0;
            counter = 0;
          }
        }
        else counter = 0;

        // write data in sd card
        sdcard_writeFile(millis(), yaw, pitch, ay, vy, py);
      }

      last_timestep = timestep;
      last_vy = vy;
      break;
    }
    case DONE: 
    {
      ledcWrite(LED_CH, 0);

      // close file and end sd comms
      if(parachuted)
      {
        sdcard_closeFile();
        //sdcard_end();
        parachuted = 0;
      }
        
      // short blink
      if (millis() - last_millis >= BLINK_PERIOD_S * 1000)
      {
        last_millis = millis();
        ledcWrite(LED_CH, MAX_DC);
        delay(500);
      }

      break;
    }
  }
}
