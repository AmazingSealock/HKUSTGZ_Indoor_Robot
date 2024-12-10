//
// Created by lsy on 24-11-19.
//

// NavigationMonitor.h
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <ctime>
#include <vector>

namespace gbx_manual {

struct NavigationSegment {
  ros::Time start_time;
  ros::Time end_time;
  geometry_msgs::Point start_position;
  geometry_msgs::Point end_position;
  double average_velocity;
  double distance;
};

class NavigationMonitor {
public:
  NavigationMonitor(ros::NodeHandle& nh);
  ~NavigationMonitor() = default;

  void initialize();
  void reset();
  void publishSpeedMarkers();

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void navigationStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
  void navigationFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);

  std::string convertToBeijingTime(const ros::Time& time);
  double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
  void updateSegment();
  void printNavigationSummary();
  void publishDeliveryPointMarkers();
  bool loadDeliveryPoints(ros::NodeHandle& nh);

  ros::NodeHandle& nh_;
  ros::Subscriber cmdVelSub_;
  ros::Subscriber navStateSub_;
  ros::Subscriber navFeedbackSub_;
  ros::Publisher speedMarkerPub_;
  ros::Publisher deliveryPointsPub_;
  ros::Time navigationStartTime_;
  ros::Time navigationEndTime_;
  ros::Time lastPositionTime_;
  geometry_msgs::Point lastPosition_;
  std::vector<NavigationSegment> segments_;
  double totalDistance_;
  double waitTime_;
  double velocityThreshold_;  // 速度阈值，低于此值认为是等待状态
  bool isNavigating_;
  bool isWaiting_;
  ros::Time waitStartTime_;
  std::map<int, std::vector<double>> deliveryPoints_;
};

} // namespace gbx_manual
