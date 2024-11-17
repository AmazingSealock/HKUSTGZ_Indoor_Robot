//
// Created by lsy on 24-10-22.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class CmdTransit
{
public:
  CmdTransit()
  {
    ros::NodeHandle private_nh("~");
    private_nh.param("input_topic", input_topic_, std::string("/cmd_vel"));
    private_nh.param("output_topic", output_topic_, std::string("/ranger/cmd_vel"));
    private_nh.param("frequency", frequency_, 10.0);

    cmd_vel_sub_ = nh_.subscribe(input_topic_, 10, &CmdTransit::cmdVelCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 10);

    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &CmdTransit::timerCallback, this);

    has_received_msg_ = false;
  }

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    last_cmd_vel_ = *msg;
    has_received_msg_ = true;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (has_received_msg_)
    {
      cmd_vel_pub_.publish(last_cmd_vel_);
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Timer timer_;

  geometry_msgs::Twist last_cmd_vel_;
  bool has_received_msg_;

  std::string input_topic_;
  std::string output_topic_;
  double frequency_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_transit");
  CmdTransit transit;

  ros::spin();
  return 0;
}
