//
// Created by lsy on 24-10-31.
//

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <sstream>
#include <string>

class CsvToPointPublisher {
public:
  CsvToPointPublisher(ros::NodeHandle& nh, const std::string& csv_file_path)
      : nh_(nh), csv_file_path_(csv_file_path) {
    pub_ = nh_.advertise<geometry_msgs::PointStamped>("/clicked_point", 10);
    readAndPublish();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string csv_file_path_;

  void readAndPublish() {
    std::ifstream file(csv_file_path_);
    std::string line;

    while (std::getline(file, line) && ros::ok()) {
      std::istringstream ss(line);
      std::string x_str, y_str, z_str;

      if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, z_str, ',')) {
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = "map";
        point_msg.point.x = std::stof(x_str);
        point_msg.point.y = std::stof(y_str);
        point_msg.point.z = std::stof(z_str);

        pub_.publish(point_msg);
        ros::Duration(0.25).sleep();  // 控制发布频率
      }
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_transit");
  ros::NodeHandle nh("~");

  std::string csv_file_path;
  nh.param<std::string>("csv_file_path", csv_file_path, "/path/to/your/file.csv");

  CsvToPointPublisher publisher(nh, csv_file_path);
  ros::spin();

  return 0;
}
