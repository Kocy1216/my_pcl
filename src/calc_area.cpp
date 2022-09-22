
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
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering


  
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::fromROSMsg (*cloud_msg, cloud1);
  //std::cout <<cloud1[cloud1.size()/2].x<< std::endl;
    //std::cout <<area<< std::endl;
 pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients );
 pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
 
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory std::cout <<cloud1[i].x<< std::endl;
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (cloud1.makeShared ());
  seg.segment (*inliers, *coefficients);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);
   /*
   std::cout <<coefficients->values[0]<<"|"<<coefficients->values[1]
   <<"|"<<coefficients->values[2]
   <<"|"<<coefficients->values[3]
   <<"|"<<coefficients->values[4]
   <<"|"<<coefficients->values[5]
   << std::endl;
   */
   std::cout <<"--------------------------------------"<< std::endl;
  // x = alpha * y + beta
  double alpha = coefficients->values[3] / coefficients->values[4];
  double beta = coefficients->values[0] - alpha * coefficients->values[1];
  std::cout<< alpha << "||||" << beta<<std::endl;
    
  // double ground = 0.4;
  // double area = 0;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 {new pcl::PointCloud<pcl::PointXYZ>};
  // pcl::PointCloud<pcl::PointXYZ>::Ptr two_cloud {new pcl::PointCloud<pcl::PointXYZ>};
  // // Convert to PCL data type
  // pcl::fromROSMsg(*cloud_msg, *cloud2);
  // two_cloud->points.resize(4);
  // int j = 0;
  // bool first =true;
  // ///std::cout<< cloud2->points[1]<<std::endl;
  // for(int i = 0 ; i < cloud2->points.size()-1 ; i++)
  // {

  //   if(cloud2->points[i].x>0.1 && cloud2->points[i].y>-0.3 && cloud2->points[i].y<0.3){


  //     if(first){
  //       two_cloud->points[0] = cloud2->points[i];
  //       first = false; 
  //     }
  //     two_cloud->points[1] = cloud2->points[i];
  //   }
    
  // }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud1.makeShared ());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_p);////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  /*
  two_cloud->points[2].x = coefficients->values[0] + coefficients->values[3] * 0.5;
  two_cloud->points[2].y =  -0.5;
  two_cloud->points[3].x = coefficients->values[0] + coefficients->values[3] * -0.5;
  two_cloud->points[3].y =  0.5;
  std::cout<< two_cloud->points[0]<<std::endl;
  std::cout<< two_cloud->points[1]<<std::endl;
  */
  sensor_msgs::PointCloud2 output1;
  //pcl_conversions::fromPCL(*two_cloud, output1);
  pcl::toROSMsg(*cloud_p, output1);
  output1.header.frame_id = "laser";

  // Publish the data
  pub.publish (output1);
  
  
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