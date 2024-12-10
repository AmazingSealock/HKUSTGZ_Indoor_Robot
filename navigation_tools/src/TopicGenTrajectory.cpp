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
      : nh_(nh), csv_file_path_(csv_file_path), point_count_(0) {
    sub_ = nh_.subscribe("/clicked_point", 10, &PointToCsvWriter::pointCallback, this);

    csv_file_.open(csv_file_path_, std::ios::out | std::ios::app);
    if (!csv_file_.is_open()) {
      ROS_ERROR("Failed to open CSV file for writing: %s", csv_file_path_.c_str());
    } else {
      csv_file_ << "index,x,y,z\n";
      ROS_INFO("Started writing to CSV file: %s", csv_file_path_.c_str());
    }
  }

  ~PointToCsvWriter() {
    if (csv_file_.is_open()) {
      csv_file_.close();
      ROS_INFO("CSV file closed. Total points written: %d", point_count_);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::ofstream csv_file_;
  std::string csv_file_path_;
  int point_count_;

  void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (csv_file_.is_open()) {
      double x = msg->point.x;
      double y = msg->point.y;
      double z = 0.0;

      csv_file_ << point_count_ << "," << x << "," << y << "," << z << "\n";
      ROS_INFO("Point %d written to CSV: [x: %.3f, y: %.3f, z: %.3f]",
               point_count_, x, y, z);

      point_count_++;
    } else {
      ROS_ERROR("CSV file is not open. Cannot write data.");
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_to_csv_writer");
  ros::NodeHandle nh("~");

  std::string csv_file_path;
  nh.param<std::string>("csv_file_path", csv_file_path, "/path/to/your/output.csv");

  PointToCsvWriter writer(nh, csv_file_path);
  ros::spin();

  return 0;
}