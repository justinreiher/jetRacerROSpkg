#include "PCA9685.h"
#include "ros/ros.h"
#include <cmath>

PCA9685Driver::PCA9685Driver(ros::NodeHandle &node)
{
  
  this->node = node; //store the car ROS node pointer
	
  //get the PCA9685driver parameters
  node.getParam("/racerParam/i2cPWMDriverAddress", this->i2cAddress);
  node.getParam("/racerParam/pwmFrequency", this->pwmFrequency);
  node.getParam("/racerParam/steerChannel", this->steerChannel);
  node.getParam("/racerParam/throttleChannel", this->throttleChannel);

  //get the hard limits for steering
  node.getParam("/racerParam/steerAngleMin", this->steerAngleMin);
  node.getParam("/racerParam/steerAngleMax", this->steerAngleMax);
  node.getParam("/racerParam/dcSteerMin", this->steerMinDC);
  node.getParam("/racerParam/dcSteerMax", this->steerMaxDC);

  //get the hard limits for throttle
  node.getParam("/racerParam/throttleReverseMax", this->throttleReverseMax);
  node.getParam("/racerParam/throttleForwardMax", this->throttleForwardMax);
  node.getParam("/racerParam/dcThrottleFwrdFull", this->throttleMaxFwrdDC);
  node.getParam("/racerParam/dcThrottleReverseFull", this->throttleMaxReverseDC);
  
  //read back and display the loaded in parameters - useful to double check the parameters have been read in properly.
  ROS_INFO("PCA9685 PWM Driver i2c Address: %#2x", this->i2cAddress);
  ROS_INFO("PWM Driver Frequency: %d Hz", this->pwmFrequency);
  ROS_INFO("Steering Channel: %d", this->steerChannel);
  ROS_INFO("Throttle Channel: %d", this->throttleChannel);
  
  ROS_INFO("jetRacer Steering Limits:");
  ROS_INFO("Min Steer Angle: %f", this->steerAngleMin);
  ROS_INFO("Max Steer Angle: %f", this->steerAngleMax);
  ROS_INFO("Min Duty Cycle Steer Limit: %d", this->steerMinDC);
  ROS_INFO("Max Duty Cycle Steer Limit: %d", this->steerMaxDC);
  
  ROS_INFO("jetRacer Throttle Limits:");
  ROS_INFO("Max Throttle Reverse: %f", this->throttleReverseMax);
  ROS_INFO("Max Throttle Forward: %f", this->throttleForwardMax);
  ROS_INFO("Max Duty Cycle Throttle Reverse: %d",this->throttleMaxReverseDC);
  ROS_INFO("Max Duty Cycle Throttle Forward: %d",this->throttleMaxFwrdDC);

  this->steerPWMOffset    = (this->steerMinDC + this->steerMaxDC)/2;
  this->steerPWMGain      = round((this->steerMinDC - this->steerPWMOffset)/this->steerAngleMin);

  this->throttlePWMOffset = (this->throttleMaxFwrdDC + this->throttleMaxReverseDC)/2;
  this->throttlePWMGain   = round((this->throttleMaxFwrdDC - this->throttlePWMOffset)/this->throttleForwardMax);


  ROS_INFO("Initializing PCA9685 PWM Driver...");
  PCA9685Driver::Init();
  ROS_INFO("PCA9685 PWM Driver ready");
}

/*
* setPWM_DutyCycle(jetRacerCar msg)
* This function is the callback function which is subscribed to the jetRacer_Controller topic
* It is responsible for transforming the steering angle and throttle percentage into appropriate
* PWM duty cylce signals to be sent over i2c to the PCA9685 PWM driver board
*    jetRacerCar msg has fields: steerAngle and throttle
*    
*/
void PCA9685Driver::setPWM_DutyCycle(const jetracer::jetRacerCar::ConstPtr& msg)
{
	int pwmSteerAngle;
	int pwmThrottle; 
	
	//managing steering messages    
	ROS_INFO("Steering set to: %f", msg->steerAngle);
        pwmSteerAngle = PCA9685Driver::angleToPWM(msg->steerAngle);
        //ROS_INFO("Converted PWM Steer: %d",pwmSteerAngle);
	PCA9685Driver::sendData(this->steerChannel,pwmSteerAngle);
	

	//managing throttle messages
	ROS_INFO("Throttle msg: %f", msg->throttle);
   	pwmThrottle = PCA9685Driver::throttleToPWM(msg->throttle);
	//ROS_INFO("Converted PWM Throttle: %d",pwmThrottle);
        PCA9685Driver::sendData(this->throttleChannel,pwmThrottle);

}

bool PCA9685Driver::sendData(uint8_t channel, uint16_t dutyCycle)
{
   uint16_t value_12bit = dutyCycle >> 4;
    uint8_t values[4];
    if(value_12bit == 0x0FFF) { // always on
    	values[0] = 0x10;
    	values[1] = 0x00;
    	values[2] = 0x00;
    	values[3] = 0x00;
    } else if(value_12bit == 0x0000) { // always off
    	values[0] = 0x00;
    	values[1] = 0x00;
    	values[2] = 0x10;
    	values[3] = 0x00;
    } else { // PWM
    	values[0] = 0x00;
    	values[1] = 0x00;
    	values[2] = (value_12bit + 1) & 0xFF;
    	values[3] = (value_12bit + 1) >> 8;
    }

    _i2c_smbus_write_i2c_block_data(this->file, PCA9685_CHANNEL0_REG + (channel * 4), 4, values);

   return true;
}

int PCA9685Driver::angleToPWM(float steerAngle)
{
   int pwmSteer = round(this->steerPWMOffset + this->steerPWMGain*steerAngle);
   
   if(pwmSteer > this->steerMaxDC)
   { 
      pwmSteer = this->steerMaxDC;
      ROS_INFO("Warning - Steer Angle saturated to: %f",this->steerAngleMax);
   }

   if(pwmSteer < this->steerMinDC)
   {
     pwmSteer = this->steerMinDC;
     ROS_INFO("Warning - Steer Angle saturated to: %f",this->steerAngleMin);
   }

  return pwmSteer;
}

int PCA9685Driver::throttleToPWM(float throttle)
{
  int pwmThrottle = round(this->throttlePWMOffset + this->throttlePWMGain*throttle);
 
  if(pwmThrottle < this->throttleMaxFwrdDC)
  {
     pwmThrottle = this->throttleMaxFwrdDC;
     ROS_WARN("Warning - Throttle saturated to: %f",this->throttleForwardMax);
  }

  if(pwmThrottle > this->throttleMaxReverseDC)
  {
    pwmThrottle = this->throttleMaxReverseDC;
    ROS_WARN("Warning - Throttle saturated to: %f", this->throttleReverseMax);
  }

  return pwmThrottle;
}

bool PCA9685Driver::Init()
{
  this->file = open("/dev/i2c-1",O_RDWR);
  if(ioctl(this->file, I2C_SLAVE,this->i2cAddress) < 0)
  {
     ROS_ERROR("i2c device could not open properly");
  }
  if(!PCA9685Driver::Reset())
  {
    ROS_ERROR("Failed to Reset PCA9685 PWM Driver");
  }

  return true;
}

bool PCA9685Driver::Reset()
{
  int i = 0;
  
  // reset
    _i2c_smbus_write_byte_data(this->file, PCA9685_MODE1_REG, 0b10000000);
    ros::Duration(0.500).sleep();

  // set frequency
    uint8_t prescale = (uint8_t)(25000000.0 / 4096.0 / this->pwmFrequency + 0.5);
  
  _i2c_smbus_write_byte_data(this->file, PCA9685_MODE1_REG, 0b00010000); // sleep
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(this->file, PCA9685_PRESCALE_REG, prescale); // set prescale
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(this->file, PCA9685_MODE2_REG, 0x04); // outdrv
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(this->file, PCA9685_MODE1_REG, 0xA1); // un-sleep
    ros::Duration(0.025).sleep();

    int pwmSteerCenter  = PCA9685Driver::angleToPWM(0.0);
    int pwmThrottleZero = PCA9685Driver::throttleToPWM(0.0);

    PCA9685Driver::sendData(this->steerChannel,pwmSteerCenter);
   PCA9685Driver::sendData(this->throttleChannel,pwmThrottleZero);

    return true;
}

