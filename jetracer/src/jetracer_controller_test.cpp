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

/* This node is to test the functionality of the jetracer_car node. It sweeps the range of the messages to be received by the jetracer_car node.
*
* This ROS node publishes commands to jetRacer_Controller using the messages defined in ../msg/jetRacerCar.msg

* the expected steering range is between [-pi/12,pi/12]
* the expected throttle range is between [-100,100]
*/

#include "ros/ros.h"
#include "jetracer/jetRacerCar.h"

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <cstring>
#include <iostream>
#include <stdio.h>

#include <cmath>

int main(int argc, char **argv)
{

  int bufferSize;
  
  ros::init(argc,argv, "jetRacer_Controller");

  ros::NodeHandle node;

  node.getParam("/racerParam/messageBufferSize",bufferSize);

  ros::Publisher controller = node.advertise<jetracer::jetRacerCar>("/jetRacer_Controller",bufferSize);

  
  ros::Rate loop_rate(10);

  int count = 0;
  ROS_INFO("Starting in 20\n");

  ros::Duration(20).sleep();
  ROS_INFO("Starting test...");

  while(ros::ok())
  {
   
   jetracer::jetRacerCar msg;

   if(count <= 100)
   {
    
      msg.steerAngle = -M_PI/12;//count*M_PI/1200;
      msg.throttle = count/2;

      controller.publish(msg);

   }
   else
   {
      msg.steerAngle = 0.0;
      msg.throttle = 0.0;
      
      controller.publish(msg);

      ROS_INFO("Test Complete - end by pressing Ctrl-C");
   }

    ros::spinOnce();
    loop_rate.sleep();
    count++;

  }

  return 0;
}
