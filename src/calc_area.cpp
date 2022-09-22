
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//#include "handmade_ndt_scan_matcher/ndt_scan_matcher.hpp"

ros::Publisher cloud_pub1,cloud_pub2,pose_pub;

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
  // double alpha = coefficients->values[3] / coefficients->values[4];
  // double beta = coefficients->values[0] - alpha * coefficients->values[1];////////////////////////////////////////////////////////
  double a = coefficients->values[4];
  double b = -coefficients->values[3];
  double c = -coefficients->values[4] * coefficients->values[0] + coefficients->values[3] * coefficients->values[1];

  pcl::PointCloud<pcl::PointXYZ>::Ptr two_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  two_cloud->points.resize(2);
  two_cloud->points[0].x = (b-c)/a;
  two_cloud->points[0].y = -1;
  two_cloud->points[1].x = -(b+c)/a;
  two_cloud->points[1].y=  1;
  /*
  two_cloud->points[0].x = coefficients->values[0] + coefficients->values[3] * 0.5;
  two_cloud->points[0].y =  -0.5;
  two_cloud->points[1].x = coefficients->values[0] + coefficients->values[3] * -0.5;
  two_cloud->points[1].y =  0.5;
  std::cout<< two_cloud->points[0]<<std::endl;
  std::cout<< two_cloud->points[1]<<std::endl;
  */
  sensor_msgs::PointCloud2 two_msg;
  pcl::toROSMsg(*two_cloud, two_msg);
  two_msg.header.frame_id = "laser";
  cloud_pub2.publish (two_msg);
  
  double yaw = tan(b/a);
  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header.stamp = cloud_msg->header.stamp;
  pose_stamped_msg.header.frame_id = "laser";
  pose_stamped_msg.pose.position.x = 0;
  pose_stamped_msg.pose.position.y = 0;
  pose_stamped_msg.pose.position.z = 0;
  tf::Quaternion tf_quat=tf::createQuaternionFromRPY(0, 0, yaw);
  geometry_msgs::Quaternion geo_quat;
  quaternionTFToMsg(tf_quat, geo_quat);
  pose_stamped_msg.pose.orientation = geo_quat;
  pose_pub.publish(pose_stamped_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared ());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*outlier_cloud);////////////////////////////////////////////////////////////////////////////////////////////////////////
  sensor_msgs::PointCloud2 outlier_msg;
  pcl::toROSMsg(*outlier_cloud, outlier_msg);
  outlier_msg.header.frame_id = "laser";
  cloud_pub1.publish (outlier_msg);
  // std::vector<int> d(outlier_cloud->points.size(), 0);
  // std::vector<int> l(outlier_cloud->points.size(), 0);
  // for(size_t i=0;i<outlier_cloud->points.size();i++){
  //   d[i] = abs(alpha*outlier_cloud->points[i].y - outlier_cloud->points[i].x + beta) / sqrt(alpha*alpha + 1.0f);
  //   l[i] = sqrt(outlier_cloud->points[i].y*outlier_cloud->points[i].y+pow(outlier_cloud->points[i].x - beta,2) - d[i]*d[i]);
  // }
  // std::vector<int> s(outlier_cloud->points.size()-1, 0);
  // for(size_t i=0;i<outlier_cloud->points.size()-1;i++){
  //   s[i] = ((d[i] + d[i+1]) / 2.0f) * (l[i+1] - l[i]);
  // }

  
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

  // Spin
  ros::spin ();
}