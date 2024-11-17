#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFilter {
public:
  PointCloudFilter() {
    ros::NodeHandle private_nh("~");

    // 从参数服务器获取参数
    private_nh.param("min_z", min_z_, 0.0);
    private_nh.param("max_z", max_z_, 1.0);
    private_nh.param("max_radius", max_radius_, 5.0);
    private_nh.param("leaf_size", leaf_size_, 0.1);  // 新增：体素栅格大小
    private_nh.param("input_topic", input_topic_, std::string("/cloud_registered"));
    private_nh.param("output_topic", output_topic_, std::string("/cloud_registered_filter"));
    private_nh.param("frequency", frequency_, 10.0);

    point_cloud_sub_ = nh_.subscribe(input_topic_, 1, &PointCloudFilter::pointCloudCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    rate_ = new ros::Rate(frequency_);  // 设置频率控制
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // 将ROS点云数据转换为PCL格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // 基于Z轴（高度）过滤点云
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z_, max_z_);
    pass.filter(*cloud);

    // 基于体素栅格降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);  // 设置体素栅格大小
    voxel_filter.filter(*cloud);

    // 半径过滤（基于PCL的RadiusOutlierRemoval）
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(cloud);
    radius_filter.setRadiusSearch(max_radius_);  // 设置半径
    radius_filter.setMinNeighborsInRadius(2);  // 设置邻居点最小数量
    radius_filter.filter(*cloud);

    // 将处理后的点云转换回ROS消息格式并发布
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = input->header;
    output.header.frame_id = "camera_init";
    output.header.stamp = ros::Time::now();
    point_cloud_pub_.publish(output);

    rate_->sleep();  // 控制发布频率
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;
  ros::Rate* rate_;  // 频率控制

  double min_z_;  // 最小高度
  double max_z_;  // 最大高度
  double max_radius_;  // 最大半径
  double leaf_size_;  // 体素栅格大小
  std::string input_topic_;
  std::string output_topic_;
  double frequency_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_transit");
  PointCloudFilter filter_node;
  ros::spin();
  return 0;
}
