#ifndef PCA9685_H
#define PCA9685_H


#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>


#include "ros/ros.h"
#include "jetracer/jetRacerCar.h"

#include <linux/i2c-dev.h>
#include <smbus_functions.h>

#define PCA9685_MODE1_REG 0X00
#define PCA9685_MODE2_REG 0X01
#define PCA9685_SUBADR_REG 0X02
#define PCA9685_ALLCALLADR_REG 0X05
#define PCA9685_CHANNEL0_REG 0x06
#define PCA9685_CHANNEL1_REG 0x0A
#define PCA9685_CHANNEL2_REG 0x0E
#define PCA9685_CHANNEL3_REG 0x12
#define PCA9685_CHANNEL4_REG 0x16
#define PCA9685_CHANNEL5_REG 0x1A
#define PCA9685_CHANNEL6_REG 0x1E
#define PCA9685_CHANNEL7_REG 0x22
#define PCA9685_CHANNEL8_REG 0x26
#define PCA9685_CHANNEL9_REG 0x2A
#define PCA9685_CHANNEL10_REG 0x2E
#define PCA9685_CHANNEL11_REG 0x32
#define PCA9685_CHANNEL12_REG 0x36
#define PCA9685_CHANNEL13_REG 0x3A
#define PCA9685_CHANNEL14_REG 0x3E
#define PCA9685_CHANNEL15_REG 0x42
#define PCA9685_CHANNEL_ALL_REG 0xFA
#define PCA9685_PRESCALE_REG 0xFE
#define PCA9685_TESTMODE_REG 0xFF

class PCA9685Driver{

  public:
    PCA9685Driver(ros::NodeHandle &node);
    void setPWM_DutyCycle(const jetracer::jetRacerCar::ConstPtr& msg);
    void Stop();
  private:        
    int   i2cAddress;
    int   pwmFrequency;
    ros::NodeHandle node;
    int throttleChannel;
    int steerChannel;

    float steerAngleMin;
    float steerAngleMax;
    int steerMinDC;
    int steerMaxDC;
    int steerPWMGain;
    int steerPWMOffset;
    
    float throttleReverseMax;
    float throttleForwardMax;
    int throttleMaxReverseDC;
    int throttleMaxFwrdDC;
    int throttlePWMGain;
    int throttlePWMOffset;
    
    bool sendData(uint8_t channel, uint16_t dutyCycle);
    int  angleToPWM(float steerAngle);
    int  throttleToPWM(float throttle);
    bool Init();
    bool Reset();
    int file;
};


#endif
