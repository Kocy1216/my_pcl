
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

ros::Publisher cloud_pub1,cloud_pub2,pose_pub,pose_pub2,area_pub;

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

  float area_size = 0.0f;
  if(outlier_cloud->points.size()>0){
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
    std::vector<Eigen::Vector3f> d(outlier_cloud->points.size(), Eigen::Vector3f::Zero());
    std::vector<Eigen::Vector3f> l(outlier_cloud->points.size(), Eigen::Vector3f::Zero());
    float l_all = 0;
    for(size_t i=0;i<outlier_cloud->points.size();i++){
      p.x() = outlier_cloud->points[i].x;
      p.y() = outlier_cloud->points[i].y;
      r = p - base_p;
      d[i] = r.dot(e)*e;
      l[i] = r - d[i];  
    }
    std::cout << " d.norm : " << d[0].norm() << std::endl;
    std::vector<float> s(outlier_cloud->points.size()-1, 0);
    Eigen::Vector3f l_dist;
    l_dist = Eigen::Vector3f::Zero();
    for(size_t i=0;i<outlier_cloud->points.size()-1;i++){
      l_dist = l[i+1] - l[i];
      l_all += l_dist.norm();
      s[i] = ((d[i].norm() + d[i+1].norm()) / 2.0f) * l_dist.norm();
      area_size += s[i];
    }
    std::cout << " l_all : " << l_all << std::endl;
  }
  std_msgs::Float32 area_size_msg;
  area_size_msg.data = area_size;
  area_pub.publish(area_size_msg);
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
  ros::Subscriber sub = nh.subscribe ("limit_cloud", 1, CloudCallback);

  // Create a ROS publisher for the output point cloud
  cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2> ("line_cloud", 1);
  cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2> ("two_cloud", 1);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("e", 1);
  pose_pub2 = nh.advertise<geometry_msgs::PoseStamped> ("o", 1);
  area_pub = nh.advertise<std_msgs::Float32>("area_size", 1);
  // Spin
  ros::spin ();
}