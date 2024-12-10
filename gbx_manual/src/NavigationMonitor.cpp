//
// Created by lsy on 24-11-19.
//
#include "gbx_manual/NavigationMonitor.h"

namespace gbx_manual {

NavigationMonitor::NavigationMonitor(ros::NodeHandle& nh)
    : nh_(nh),
      totalDistance_(0.0),
      waitTime_(0.0),
      velocityThreshold_(0.1),  // 设置速度阈值为0.1 m/s
      isNavigating_(false),
      isWaiting_(false) {
}

void NavigationMonitor::initialize() {
  cmdVelSub_ = nh_.subscribe("/cmd_vel", 1, &NavigationMonitor::cmdVelCallback, this);
  navStateSub_ = nh_.subscribe("/move_base/status", 1, &NavigationMonitor::navigationStateCallback, this);
  navFeedbackSub_ = nh_.subscribe("/move_base/feedback", 1, &NavigationMonitor::navigationFeedbackCallback, this);
  speedMarkerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/navigation_speed_markers", 1);
  deliveryPointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/delivery_points_markers", 1, true);
  if (loadDeliveryPoints(nh_)) {
    publishDeliveryPointMarkers();
  }
}

void NavigationMonitor::reset() {
  navigationStartTime_ = ros::Time();
  navigationEndTime_ = ros::Time();
  totalDistance_ = 0.0;
  waitTime_ = 0.0;
  segments_.clear();
  isNavigating_ = false;
  isWaiting_ = false;
}

bool NavigationMonitor::loadDeliveryPoints(ros::NodeHandle& nh) {
  XmlRpc::XmlRpcValue delivery_points;

  if (!nh.getParam("delivery_point", delivery_points)) {
    ROS_ERROR("Failed to get delivery_point from parameter server.");
    return false;
  }

  if (delivery_points.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("delivery_point should be an array.");
    return false;
  }

  for (int i = 0; i < delivery_points.size(); ++i) {
    if (delivery_points[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_WARN("Each entry in delivery_point should be a dictionary.");
      continue;
    }

    for (auto it = delivery_points[i].begin(); it != delivery_points[i].end(); ++it) {
      int id;
      try {
        id = std::stoi(it->first);
      } catch (const std::exception& e) {
        ROS_WARN("Failed to convert key to integer: %s", it->first.c_str());
        continue;
      }

      XmlRpc::XmlRpcValue coords = it->second;
      if (coords.getType() != XmlRpc::XmlRpcValue::TypeArray || coords.size() != 3) {
        ROS_WARN("Coordinates for point %d should be an array of size 3", id);
        continue;
      }

      std::vector<double> point_coords;
      for (int j = 0; j < 3; ++j) {
        try {
          // XmlRpc::XmlRpcValue到double的转换
          if (coords[j].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            point_coords.push_back(static_cast<double>(coords[j]));
          } else if (coords[j].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            point_coords.push_back(static_cast<int>(coords[j]));
          } else {
            throw std::runtime_error("Invalid coordinate type");
          }
        } catch (const std::exception& e) {
          ROS_WARN("Error converting coordinate for point %d: %s", id, e.what());
          continue;
        }
      }

      if (point_coords.size() == 3) {
        deliveryPoints_[id] = point_coords;
        ROS_INFO("Loaded delivery point %d: [%.2f, %.2f, %.2f]",
                 id, point_coords[0], point_coords[1], point_coords[2]);
      }
    }
  }

  return !deliveryPoints_.empty();
}

void NavigationMonitor::publishDeliveryPointMarkers() {
  visualization_msgs::MarkerArray markerArray;
  for (const auto& point : deliveryPoints_) {
    visualization_msgs::Marker pointMarker;
    pointMarker.header.frame_id = "map";
    pointMarker.header.stamp = ros::Time::now();
    pointMarker.ns = "delivery_points";
    pointMarker.id = point.first;
    pointMarker.type = visualization_msgs::Marker::SPHERE;
    pointMarker.action = visualization_msgs::Marker::ADD;
    pointMarker.pose.position.x = point.second[0];
    pointMarker.pose.position.y = point.second[1];
    pointMarker.pose.position.z = point.second[2];
    pointMarker.pose.orientation.w = 1.0;
    pointMarker.scale.x = 5;
    pointMarker.scale.y = 5;
    pointMarker.scale.z = 5;
    pointMarker.color.r = 0.0;
    pointMarker.color.g = 0.0;
    pointMarker.color.b = 1.0;
    pointMarker.color.a = 1.0;
    pointMarker.lifetime = ros::Duration(0);
    visualization_msgs::Marker textMarker = pointMarker;
    textMarker.id = point.first + 1000;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.text = std::to_string(point.first);
    textMarker.pose.position.z += 5;
    textMarker.scale.z = 5;
    textMarker.color.r = 1.0;
    textMarker.color.g = 1.0;
    textMarker.color.b = 1.0;
    markerArray.markers.push_back(pointMarker);
    markerArray.markers.push_back(textMarker);
  }
  deliveryPointsPub_.publish(markerArray);
}

std::string NavigationMonitor::convertToBeijingTime(const ros::Time& time) {
  time_t raw_time = time.sec;
  // raw_time += 8 * 3600;  // 转换为北京时间（UTC+8）
  struct tm* timeinfo = localtime(&raw_time);
  char buffer[80];
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
  return std::string(buffer);
}

double NavigationMonitor::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

void NavigationMonitor::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (!isNavigating_) return;
  double current_velocity = sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2));
  if (current_velocity < velocityThreshold_) {
    if (!isWaiting_) {
      isWaiting_ = true;
      waitStartTime_ = ros::Time::now();
    }
  } else {
    if (isWaiting_) {
      waitTime_ += (ros::Time::now() - waitStartTime_).toSec();
      isWaiting_ = false;
    }
  }
}

void NavigationMonitor::navigationStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if (msg->status_list.empty()) return;
  const auto& status = msg->status_list.back();
  // PENDING = 0
  // ACTIVE = 1
  // SUCCEEDED = 3
  if (status.status == 1 && !isNavigating_) {  // 开始导航
    isNavigating_ = true;
    navigationStartTime_ = ros::Time::now();
    ROS_INFO("Navigation started at: %s", convertToBeijingTime(navigationStartTime_).c_str());
  }
  else if (status.status == 3 && isNavigating_) {  // 导航完成
    isNavigating_ = false;
    navigationEndTime_ = ros::Time::now();
    printNavigationSummary();
  }
}

void NavigationMonitor::navigationFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
  if (!isNavigating_) return;
  const auto& current_pos = msg->feedback.base_position.pose.position;
  ros::Time current_time = ros::Time::now();
  if (!lastPositionTime_.isZero()) {
    double segment_distance = calculateDistance(lastPosition_, current_pos);
    double time_diff = (current_time - lastPositionTime_).toSec();
    if (time_diff > 0) {
      NavigationSegment segment;
      segment.start_time = lastPositionTime_;
      segment.end_time = current_time;
      segment.start_position = lastPosition_;
      segment.end_position = current_pos;
      segment.distance = segment_distance;
      segment.average_velocity = segment_distance / time_diff;
      segments_.push_back(segment);
      totalDistance_ += segment_distance;
      publishSpeedMarkers();
    }
  }
  lastPosition_ = current_pos;
  lastPositionTime_ = current_time;
}

void NavigationMonitor::publishSpeedMarkers(){
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  // 找到最大和最小速度用于归一化
  double max_velocity = 0.0;
  double min_velocity = std::numeric_limits<double>::max();

  for (const auto& segment : segments_) {
    max_velocity = std::max(max_velocity, segment.average_velocity);
    min_velocity = std::min(min_velocity, segment.average_velocity);
  }

  for (const auto& segment : segments_) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "speed_segments";
    line_marker.id = id++;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1;  // 线宽

    // 根据速度设置颜色（红->黄->绿）
    double normalized_speed = (segment.average_velocity - min_velocity) / (max_velocity - min_velocity);
    line_marker.color.r = (1.0 - normalized_speed);
    line_marker.color.g = normalized_speed;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    p1 = segment.start_position;
    p2 = segment.end_position;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    marker_array.markers.push_back(line_marker);
  }
  speedMarkerPub_.publish(marker_array);
}

void NavigationMonitor::printNavigationSummary() {
  double total_time = (navigationEndTime_ - navigationStartTime_).toSec();
  double average_speed = totalDistance_ / (total_time - waitTime_);

  ROS_INFO("Navigation Summary:");
  ROS_INFO("Start time: %s", convertToBeijingTime(navigationStartTime_).c_str());
  ROS_INFO("End time: %s", convertToBeijingTime(navigationEndTime_).c_str());
  ROS_INFO("Total time: %.2f seconds", total_time);
  ROS_INFO("Total distance: %.2f meters", totalDistance_);
  ROS_INFO("Total wait time: %.2f seconds", waitTime_);
  ROS_INFO("Average speed (excluding wait time): %.2f m/s", average_speed);
}

} // namespace gbx_manual