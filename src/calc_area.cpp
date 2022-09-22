
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//#include "handmade_ndt_scan_matcher/ndt_scan_matcher.hpp"

ros::Publisher pub;

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

  // x = alpha * y + beta
  double alpha = coefficients->values[3] / coefficients->values[4];
  double beta = coefficients->values[0] - alpha * coefficients->values[1];////////////////////////////////////////////////////////

  pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared ());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*outlier_cloud);////////////////////////////////////////////////////////////////////////////////////////////////////////
  sensor_msgs::PointCloud2 outlier_msg;
  pcl::toROSMsg(*outlier_cloud, outlier_msg);
  outlier_msg.header.frame_id = "laser";
  // Publish the data
  pub.publish (outlier_msg);
  
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
  pub = nh.advertise<sensor_msgs::PointCloud2> ("line_cloud", 1);

  // Spin
  ros::spin ();
}