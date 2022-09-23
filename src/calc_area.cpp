
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include "handmade_ndt_scan_matcher/ndt_scan_matcher.hpp"

ros::Publisher cloud_pub1,cloud_pub2,pose_pub,pose_pub2,area_pub,volume_pub;

std::vector<float> area_array;
geometry_msgs::PoseStamped last_pose;
bool first_gnss = true;
float total_volume = 0;

void
CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients );
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared ());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*outlier_cloud);
  sensor_msgs::PointCloud2 outlier_msg;
  pcl::toROSMsg(*outlier_cloud, outlier_msg);
  outlier_msg.header.frame_id = "laser";
  cloud_pub1.publish (outlier_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr limit_outlier_cloud {new pcl::PointCloud<pcl::PointXYZ>};
  for(int i = 0 ; i < outlier_cloud->points.size() ; i++)
  {
    if(abs(outlier_cloud->points[i].y)< 0.25){
      limit_outlier_cloud->points.push_back(outlier_cloud->points[i]);
    }
  }

  float area_size = 0.0f;
  if(limit_outlier_cloud->points.size()>0){
    Eigen::Vector3f e,p,r,base_p;
    e = Eigen::Vector3f::Zero();
    e.x() = coefficients->values[4];
    e.y() = coefficients->values[3];
    float e_norm = e.norm();
    e.x() /= e.norm();
    e.y() /= e.norm();
    base_p = Eigen::Vector3f::Zero();
    base_p.x() = coefficients->values[0];
    base_p.y() = coefficients->values[1];

    geometry_msgs::PoseStamped pose_stamped_msg2;
    pose_stamped_msg2.header.stamp = cloud_msg->header.stamp;
    pose_stamped_msg2.header.frame_id = "laser";
    pose_stamped_msg2.pose.position.x = base_p.x();
    pose_stamped_msg2.pose.position.y = base_p.y();
    pose_stamped_msg2.pose.position.z = 0;
    pose_stamped_msg2.pose.orientation.x = 0;
    pose_stamped_msg2.pose.orientation.y = 0;
    pose_stamped_msg2.pose.orientation.z = 0;
    pose_stamped_msg2.pose.orientation.w = 1;
    pose_pub2.publish(pose_stamped_msg2);

    p = Eigen::Vector3f::Zero();
    r = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector3f> d(limit_outlier_cloud->points.size(), Eigen::Vector3f::Zero());
    std::vector<Eigen::Vector3f> l(limit_outlier_cloud->points.size(), Eigen::Vector3f::Zero());
    float l_all = 0;
    for(size_t i=0;i<limit_outlier_cloud->points.size();i++){
      p.x() = limit_outlier_cloud->points[i].x;
      p.y() = limit_outlier_cloud->points[i].y;
      r = p - base_p;
      d[i] = r.dot(e)*e;
      l[i] = r - d[i];  
    }
    // std::cout << " d.norm : " << d[0].norm() << std::endl;
    std::vector<float> s(limit_outlier_cloud->points.size()-1, 0);
    Eigen::Vector3f l_dist;
    l_dist = Eigen::Vector3f::Zero();
    for(size_t i=0;i<limit_outlier_cloud->points.size()-1;i++){
      l_dist = l[i+1] - l[i];
      l_all += l_dist.norm();
      s[i] = ((d[i].norm() + d[i+1].norm()) / 2.0f) * l_dist.norm();
      area_size += s[i];
    }
    // std::cout << " l_all : " << l_all << std::endl;
  }
  std_msgs::Float32 area_size_msg;
  area_size_msg.data = area_size;
  area_pub.publish(area_size_msg);
  area_array.push_back(area_size);
}

void
PoseCallback (const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  if(!first_gnss && area_array.size()>1){
    float distance_x,distance_y,gnss_distance,lidar_distance,gnss_volume;
    gnss_volume = 0;
    distance_x = pose_msg->pose.position.x - last_pose.pose.position.x;
    distance_y = pose_msg->pose.position.y - last_pose.pose.position.y;
    gnss_distance = std::hypot(distance_x,distance_y);
    lidar_distance = gnss_distance / (area_array.size()-1);
    std::vector<float> lidar_volume(area_array.size()-1,0);
    for(size_t i=0;i<area_array.size()-1;i++){
        lidar_volume[i] = ((area_array[i] + area_array[i+1]) / 2)*lidar_distance;
        gnss_volume += lidar_volume[i];
        total_volume += lidar_volume[i];
    }
    std_msgs::Float32 total_volume_msg;
    total_volume_msg.data = total_volume;
    volume_pub.publish(total_volume_msg);
    float last_lidar_area;
    last_lidar_area = area_array[area_array.size()-1];
    area_array.clear();
    area_array.push_back(last_lidar_area);
  }
  last_pose = *pose_msg;
  first_gnss = false;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "calc_area");
  ros::NodeHandle nh;

  // Set ROS param
  ros::param::set("down_rate", 0.5);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber cloud_sub = nh.subscribe ("limit_cloud", 1, CloudCallback);
  ros::Subscriber pose_sub = nh.subscribe ("gnss_pose", 1, PoseCallback);

  // Create a ROS publisher for the output point cloud
  cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2> ("line_cloud", 1);
  cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2> ("two_cloud", 1);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("e", 1);
  pose_pub2 = nh.advertise<geometry_msgs::PoseStamped> ("o", 1);
  area_pub = nh.advertise<std_msgs::Float32>("area_size", 1);
  volume_pub = nh.advertise<std_msgs::Float32>("total_volume", 1);
  // Spin
  ros::spin ();
}