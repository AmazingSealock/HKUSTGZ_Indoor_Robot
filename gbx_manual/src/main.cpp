//
// Created by lsy on 24-11-5.
//

#include "ros/ros.h"
#include "gbx_manual/GBXManual.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gbx_manual");

  ros::NodeHandle nh("gbx_manual");
  tf2_ros::Buffer buffer(ros::Duration(10));
  gbx_manual::GBXManual manual_control(nh, buffer);

  manual_control.initialize();

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    manual_control.update();

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}