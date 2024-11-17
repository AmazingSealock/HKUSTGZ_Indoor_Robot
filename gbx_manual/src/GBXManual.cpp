//
// Created by lsy on 24-11-5.
//

#include "gbx_manual/GBXManual.h"

namespace gbx_manual
{
GBXManual::GBXManual(ros::NodeHandle nh, tf2_ros::Buffer& tfBuffer)
    : nh_(nh), is_paused_(false), currentState_(NavigationState::STOP), tfBuffer_(tfBuffer), tfListener_(tfBuffer)
{
  std::map<std::string, std::string> csv_paths;
  if (loadStoryTrajectories(nh_, csv_paths)) {
    for (const auto& pair : csv_paths) {
      ROS_INFO("Trajectory name: %s, Path: %s", pair.first.c_str(), pair.second.c_str());
    }
  } else {
    ROS_ERROR("Failed to load trajectories.");
  }
  trajectoryPublisher_ = std::make_unique<TrajectoryPublisher>(nh_,csv_paths);
  cancelNavigationClient_ = nh_.serviceClient<std_srvs::Empty>("/cancel_navigation");
  globalCostmapRos_ = new costmap_2d::Costmap2DROS("global_costmap", tfBuffer_);
  localCostmapRos_ = new costmap_2d::Costmap2DROS("local_costmap", tfBuffer_);
  globalCostmapRos_->start();
  localCostmapRos_->start();
  globalCostmap_ = globalCostmapRos_->getCostmap();
  localCostmap_ = localCostmapRos_->getCostmap();
}

GBXManual::~GBXManual()
{
}

void GBXManual::initialize()
{
  nh_.param<std::string>("point_cloud_topic", pointCloudTopic_, "/point_cloud");
  nh_.param<std::string>("imu_topic", imuTopic_, "/imu/data");
  nh_.param<std::string>("global_path_topic", globalPathTopic_, "/global_path");
  nh_.param<std::string>("global_waypoint_path_topic", globalWaypointsPathTopic_, "/global_waypoint_path");
  nh_.param<std::string>("local_path_topic", localPathTopic_, "/local_path");
  nh_.param<std::string>("velocity_cmd_topic", velocityCmdTopic_, "/cmd_vel");
  nh_.param<std::string>("system_state_topic", systemStateTopic_, "/system_state");

  pointCloudSub_ = nh_.subscribe(pointCloudTopic_, 1, &GBXManual::pointCloudCallback, this);
  imuSub_ = nh_.subscribe(imuTopic_, 1, &GBXManual::imuCallback, this);
  globalPathSub_ = nh_.subscribe(globalPathTopic_, 1, &GBXManual::globalPathCallback, this);
  globalWaypointsPathSub_ = nh_.subscribe(globalWaypointsPathTopic_, 1, &GBXManual::globalWaypointsPathCallback, this);
  localPathSub_ = nh_.subscribe(localPathTopic_, 1, &GBXManual::localPathCallback, this);
  velocityCmdSub_ = nh_.subscribe(velocityCmdTopic_, 1, &GBXManual::velocityCmdCallback, this);
  systemStateSub_ = nh_.subscribe(systemStateTopic_, 1, &GBXManual::systemStateCallback, this);

  ros::NodeHandle cloud_nh(nh_,"cloud_filter");
  cloud_nh.param<double>("min_z", cloudMinZ_, 0.0);
  cloud_nh.param<double>("max_z", cloudMaxZ_, 1.0);
  cloud_nh.param<double>("max_radius", cloudRadius_, 5.0);
  cloud_nh.param<double>("leaf_size", cloudLeafSize_, 0.1);

  pubTrajectoryServer_ = nh_.advertiseService("pub_trajectory", &GBXManual::pubTrajectory, this);
}

bool GBXManual::pubTrajectory(navigation_msgs::pub_trajectory::Request& req,navigation_msgs::pub_trajectory::Response& res)
{
  const std::string& sender = req.sender;
  const std::string& path_name = req.path_name;

  auto trajectory = trajectoryPublisher_->getTrajectory();
  auto it = trajectory.find(path_name);
  if (it != trajectory.end()) {
    ROS_INFO("Sender: %s, Path %s found. Publishing after 1 second delay.", sender.c_str(), path_name.c_str());
    ros::Duration(1.0).sleep();
    trajectoryPublisher_->publishTrajectory(path_name);
    lastPubTrajectory_ = path_name;
    res.success = true;
    res.message = "Path published successfully by " + sender;
    transitionToState(NavigationState::MOVE);
  } else {
    ROS_WARN("Sender: %s, Path %s not found in the trajectories map.", sender.c_str(), path_name.c_str());
    res.success = false;
    res.message = "Path not found.";
  }

  return true;
}

bool GBXManual::loadStoryTrajectories(ros::NodeHandle& nh, std::map<std::string, std::string>& csv_paths) {
  XmlRpc::XmlRpcValue trajectory_list;

  if (!nh.getParam("story_trajectories", trajectory_list)) {
    ROS_ERROR("Failed to get story_trajectories from parameter server.");
    return false;
  }

  if (trajectory_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < trajectory_list.size(); ++i) {
      if (trajectory_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct && trajectory_list[i].size() == 1) {
        auto it = trajectory_list[i].begin();
        csv_paths[static_cast<std::string>(it->first)] = static_cast<std::string>(it->second);
      } else {
        ROS_WARN("Each entry in story_trajectories should be a single key-value pair.");
      }
    }
    return true;
  } else {
    ROS_ERROR("story_trajectories should be a list of dictionaries.");
    return false;
  }
}

void GBXManual::update()
{
  switch (currentState_)
  {
    case NavigationState::STOP:
      handleStop();
      break;
    case NavigationState::MOVE:
      handleMove();
      break;
    case NavigationState::WAIT:
      handleWait();
      break;
    case NavigationState::PULL_OVER:
      handlePullOver();
      break;
    case NavigationState::ARRIVE:
      handleArrive();
      break;
  }
}

void GBXManual::stop()
{
  transitionToState(NavigationState::STOP);
}

void GBXManual::transitionToState(NavigationState new_state)
{
  if (currentState_ != new_state)
  {
    ROS_INFO("Transitioning from state %d to %d", static_cast<int>(currentState_), static_cast<int>(new_state));
    currentState_ = new_state;
  }
}

void GBXManual::handleStop()
{
//  cancelNavigation();
}

void GBXManual::handleMove()
{
  // checkForObstaclesOnPath(20);
  if (!isPubTrajectory_)
  {
    trajectoryPublisher_->publishTrajectory(lastPubTrajectory_);
    isPubTrajectory_ = true;
  }

//  ROS_INFO("In MOVE state. Robot is moving.");
}

void GBXManual::handleWait()
{
  isPubTrajectory_ = false;
  ros::Duration(1).sleep();
  transitionToState(NavigationState::MOVE);
//  ROS_INFO("In WAIT state. Robot is waiting.");
  // 实现等待逻辑，例如等待信号或超时等
}

void GBXManual::handlePullOver()
{
  // 靠边停车状态下的操作
  ROS_INFO("In PULL_OVER state. Robot is pulling over.");
  // 实现靠边停车的逻辑
}

void GBXManual::handleArrive()
{
  // 到达状态下的操作
  ROS_INFO("In ARRIVE state. Robot has arrived.");
  // 实现到达目标的逻辑
}

void GBXManual::pauseMoveBase()
{
  if (!is_paused_) {
    // 暂停move_base
    // 这里需要根据实际情况调用暂停的服务或者发送控制命令
    ROS_INFO("Pausing move_base.");
    is_paused_ = true;
  }
}

void GBXManual::checkForObstaclesAndHandle()
{
  if (globalPath_.poses.empty()) {
    ROS_WARN("No global path available.");
    return;
  }

//  for (const auto& pose : globalPath_.poses) {
//    for (const auto& point : pointCloudData_.data) {
//      double distance = std::sqrt(std::pow(pose.pose.position.x - point.x, 2) +
//                                  std::pow(pose.pose.position.y - point.y, 2));
//      if (distance < 0.5) { // 如果点云点离路径点很近
//        ROS_INFO("Obstacle detected near global path at (%f, %f)", point.x, point.y);
//
//        // 判断是动态的还是静态的障碍物
//        // 这里你可以实现动态/静态障碍物的判断
//        if (isDynamicObstacle(point)) {
//          ROS_WARN("Dynamic obstacle detected.");
//          // 做出相应的行动，比如重新规划路径等
//        } else {
//          ROS_INFO("Static obstacle detected.");
//          // 做出相应的行动
//        }
//      }
//    }
//  }
}

bool GBXManual::checkForObstaclesOnPath(int points_to_check)
{
  int points_checked = 0;
  int maxPointsToCheck = std::min(points_to_check, static_cast<int>(globalPath_.poses.size()));
  for (const auto& pose : globalPath_.poses) {
    double wx = pose.pose.position.x;
    double wy = pose.pose.position.y;
    unsigned int mx, my;

    if (localCostmap_->worldToMap(wx, wy, mx, my)) {
      unsigned char cost = localCostmap_->getCost(mx, my);
//      ROS_INFO_STREAM("Cost: " << static_cast<int>(cost));

      if (cost == costmap_2d::NO_INFORMATION) {
//        ROS_WARN("Point (%.2f, %.2f) is in an unknown area (NO_INFORMATION).", wx, wy);
        continue;
      }
//      ROS_INFO_STREAM("INSCRIBED_INFLATED_OBSTACLE: " << static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE));

      if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        ROS_WARN("Obstacle detected on global path at point (%.2f, %.2f)", wx, wy);
        cancelNavigation();
        transitionToState(NavigationState::WAIT);
        return true;
      }
    } else {
      ROS_WARN("Point (%.2f, %.2f) is outside the local costmap bounds.", wx, wy);
    }

    points_checked++;
  }
  return false;
}

bool GBXManual::isDynamicObstacle(const pcl::PointXYZ& point)
{
  // 实现动态障碍物的判断逻辑
  // 比如通过比较连续时间的点云数据来判断
  return false;  // 这里仅为示例，实际判断逻辑需要根据需求来实现
}

void GBXManual::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(cloudMinZ_, cloudMaxZ_);
  pass.filter(*cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(cloudLeafSize_, cloudLeafSize_, cloudLeafSize_);
  voxel_filter.filter(*cloud);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
  radius_filter.setInputCloud(cloud);
  radius_filter.setRadiusSearch(cloudRadius_);
  radius_filter.setMinNeighborsInRadius(2);
  radius_filter.filter(*cloud);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, pointCloudData_);
}

// -----------------Server Action---------------------
bool GBXManual::cancelNavigation()
{
  if (!cancelNavigationClient_.exists())
  {
    ROS_ERROR("move_base/cancel service is not available.");
    return false;
  }

  std_srvs::Empty srv;
  if (cancelNavigationClient_.call(srv))
  {
    ROS_INFO("Navigation goal canceled successfully.");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to cancel navigation goal.");
    return false;
  }
}

// -----------------Topic CallBack--------------------
void GBXManual::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imuData_ = *msg;
}

void GBXManual::globalPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  globalPath_ = *msg;
}

void GBXManual::globalWaypointsPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  globalWaypointsPath_ = *msg;
}

void GBXManual::localPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  localPath_ = *msg;
}

void GBXManual::velocityCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  velocityCmd_ = *msg;
}

void GBXManual::systemStateCallback(const ranger_msgs::SystemState::ConstPtr& msg)
{
  systemState_ = *msg;
}

} //namespace gbx_manual
