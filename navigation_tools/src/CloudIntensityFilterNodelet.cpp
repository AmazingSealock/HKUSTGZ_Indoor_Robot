//
// Created by lsy on 24-11-29.
//

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace cloud_filter {

class IntensityFilterNodelet : public nodelet::Nodelet {
private:
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double intensity_threshold_;
  std::string input_topic_;
  std::string output_topic_;
  std::string intensity_field_;

  virtual void onInit() override {
    auto& nh = getNodeHandle();
    auto& private_nh = getPrivateNodeHandle();

    intensity_threshold_ = private_nh.param("intensity_threshold", 100.0);
    input_topic_ = private_nh.param<std::string>("input_topic", "points");
    output_topic_ = private_nh.param<std::string>("output_topic", "points_filtered");
    intensity_field_ = private_nh.param<std::string>("intensity_field", "intensity");

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        input_topic_, 10, &IntensityFilterNodelet::cloudCallback, this);

    NODELET_INFO("Intensity filter nodelet initialized:");
    NODELET_INFO(" - Input topic: %s", input_topic_.c_str());
    NODELET_INFO(" - Output topic: %s", output_topic_.c_str());
    NODELET_INFO(" - Intensity threshold: %f", intensity_threshold_);
    NODELET_INFO(" - Intensity field name: %s", intensity_field_.c_str());
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    sensor_msgs::PointCloud2 output;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);

    pcl::PCLPointCloud2 filtered_pcl_pc2;

    int intensity_idx = pcl::getFieldIndex(pcl_pc2, intensity_field_);
    if (intensity_idx == -1) {
      NODELET_WARN_THROTTLE(1, "No %s field found in point cloud", intensity_field_.c_str());
      return;
    }

    std::vector<pcl::PCLPointField> fields = pcl_pc2.fields;
    size_t point_step = pcl_pc2.point_step;
    size_t num_points = pcl_pc2.width * pcl_pc2.height;

    filtered_pcl_pc2.fields = fields;
    filtered_pcl_pc2.point_step = point_step;
    filtered_pcl_pc2.height = 1;
    filtered_pcl_pc2.header = pcl_pc2.header;
    filtered_pcl_pc2.is_bigendian = pcl_pc2.is_bigendian;

    filtered_pcl_pc2.data.reserve(pcl_pc2.data.size());

    try {
      for (size_t i = 0; i < num_points; ++i) {
        float intensity;
        memcpy(&intensity,
               &pcl_pc2.data[i * point_step + fields[intensity_idx].offset],
               sizeof(float));

        if (intensity >= intensity_threshold_) {
          filtered_pcl_pc2.data.insert(
              filtered_pcl_pc2.data.end(),
              pcl_pc2.data.begin() + i * point_step,
              pcl_pc2.data.begin() + (i + 1) * point_step
          );
        }
      }

      filtered_pcl_pc2.width = filtered_pcl_pc2.data.size() / point_step;
      filtered_pcl_pc2.row_step = filtered_pcl_pc2.width * point_step;

      pcl_conversions::fromPCL(filtered_pcl_pc2, output);
      output.header = input->header;

      pub_.publish(output);

      NODELET_DEBUG("Filtered cloud: %zu points (input: %zu points)",
                    filtered_pcl_pc2.width, num_points);
    }
    catch (const std::exception& e) {
      NODELET_ERROR("Error processing point cloud: %s", e.what());
    }
  }
};

} // namespace cloud_filter

PLUGINLIB_EXPORT_CLASS(cloud_filter::IntensityFilterNodelet, nodelet::Nodelet)