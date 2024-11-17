//
// Created by lsy on 24-10-31.
//

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <iostream>

class PointToCsvWriter {
public:
  PointToCsvWriter(ros::NodeHandle& nh, const std::string& csv_file_path)
      : nh_(nh), csv_file_path_(csv_file_path) {
    // 订阅 /clicked_point 话题
    sub_ = nh_.subscribe("/clicked_point", 10, &PointToCsvWriter::pointCallback, this);

    // 打开文件，准备追加写入
    csv_file_.open(csv_file_path_, std::ios::out | std::ios::app);
    if (!csv_file_.is_open()) {
      ROS_ERROR("Failed to open CSV file for writing: %s", csv_file_path_.c_str());
    } else {
      ROS_INFO("Started writing to CSV file: %s", csv_file_path_.c_str());
    }
  }

  ~PointToCsvWriter() {
    if (csv_file_.is_open()) {
      csv_file_.close();
      ROS_INFO("CSV file closed.");
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::ofstream csv_file_;
  std::string csv_file_path_;

  void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (csv_file_.is_open()) {
      // 获取时间戳并格式化
      double time = msg->header.stamp.toSec();
      double x = msg->point.x;
      double y = msg->point.y;
      double z = msg->point.z;

      // 写入CSV文件
      csv_file_ << x << "," << y << "," << z << "\n";
      ROS_INFO("Point written to CSV: [time: %.3f, x: %.3f, y: %.3f, z: %.3f]", time, x, y, z);
    } else {
      ROS_ERROR("CSV file is not open. Cannot write data.");
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_to_csv_writer");
  ros::NodeHandle nh("~");

  // 从参数服务器中获取 CSV 文件路径
  std::string csv_file_path;
  nh.param<std::string>("csv_file_path", csv_file_path, "/path/to/your/output.csv");

  PointToCsvWriter writer(nh, csv_file_path);
  ros::spin();

  return 0;
}

