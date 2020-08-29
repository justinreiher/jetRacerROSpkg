/* Copyright (C) 2020, Justin Reiher
*
* Redistribution and use in source and binary froms, with or without modification are permitted
* provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/* This code is to function in conjunction with the NVIDIA jetRacer platform build by following the instruction at:
* https://github.com/NVIDIA-AI-IOT/jetracer/
* This ROS node has been built with insperation from: https://github.com/dheera/ros-pwm-pca9685
* but modified to be specific to the jetRacer platform
* parameters to re-configure and modify the i2c address, pwm limits and offsets can be found in ../config/racerParams.yaml
*
* This ROS node subscribes and listens for commands to arrive on jetRacerCar and expects messages defined in ../msg/jetRacerCar.msg

* the expected steering range is between [-pi/12,pi/12]
* the expected throttle range is between [-100,100]
*/

#include "ros/ros.h"
#include "PCA9685.h"


/*
* The main function instantiates the driver object which handles receiving and sending
* the appropriate duty cycles configured for the frequency specified in the racerParam.yaml file
*/
int main(int argc, char **argv)
{
  
  int bufferSize;
  int pwmFrequency;
  int i2cAddress;

  ros::init(argc,argv, "jetRacer_Car");

  ros::NodeHandle node;

  node.getParam("/racerParam/messageBufferSize", bufferSize);

  ROS_INFO("Initializing the jetRacer...");
  ROS_INFO("Message Buffer Size: %d",bufferSize);

  PCA9685Driver carController(node);
  
  ros::Subscriber sub = node.subscribe("/jetRacer_Controller", bufferSize, &PCA9685Driver::setPWM_DutyCycle,&carController);

  ros::spin();

  return 0;
}

