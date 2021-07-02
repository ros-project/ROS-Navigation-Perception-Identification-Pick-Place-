#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::string cloud_topic, world_frame, camera_frame;
  float voxel_leaf_size;
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  int plane_max_iter;
  float plane_dist_thresh;
  float cluster_tol;
  int cluster_min_size;
  int cluster_max_size;
  cloud_topic = priv_nh_.param<std::string>("cloud_topic", "/camera2/depth/points2");
  world_frame = priv_nh_.param<std::string>("world_frame", "kinect_link");
  camera_frame = priv_nh_.param<std::string>("camera_frame", "kinect_link");
  voxel_leaf_size = priv_nh_.param<float>("voxel_leaf_size", 0.002);
  x_filter_min = priv_nh_.param<float>("x_filter_min", -2.5);
  x_filter_max = priv_nh_.param<float>("x_filter_max",  2.5);
  y_filter_min = priv_nh_.param<float>("y_filter_min", -2.5);
  y_filter_max = priv_nh_.param<float>("y_filter_max",  2.5);
  z_filter_min = priv_nh_.param<float>("z_filter_min", -2.5);
  z_filter_max = priv_nh_.param<float>("z_filter_max",  2.5);
  plane_max_iter = priv_nh_.param<int>("plane_max_iterations", 50);
  plane_dist_thresh = priv_nh_.param<float>("plane_distance_threshold", 0.05);
  cluster_tol = priv_nh_.param<float>("cluster_tolerance", 0.01);
  cluster_min_size = priv_nh_.param<int>("cluster_min_size", 100);
  cluster_max_size = priv_nh_.param<int>("cluster_max_size", 50000);



  /*
   * SETUP PUBLISHERS
   */
  ros::Publisher object_pub, cluster_pub, pose_pub;
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1);

 while (ros::ok())
 {
  /*
   * LISTEN FOR POINTCLOUD
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

  /*
   * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
   */
  tf::TransformListener listener;
  tf::StampedTransform stransform;
  try
  {
    listener.waitForTransform(world_frame, recent_cloud->header.frame_id,  ros::Time::now(), ros::Duration(6.0));
    listener.lookupTransform(world_frame, recent_cloud->header.frame_id,  ros::Time(0), stransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  sensor_msgs::PointCloud2 transformed_cloud;
//  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
//               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
  pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

  /*
   * CONVERT POINTCLOUD ROS->PCL
   */
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg (transformed_cloud, cloud);

  /* ========================================
   * Fill Code: VOXEL GRID
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> (cloud));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud (cloud_ptr);
  voxel_filter.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_filter.filter (*cloud_voxel_filtered);

  //ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
  //ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");
  /* ========================================
   * CONVERT POINTCLOUD PCL->ROS
   * PUBLISH CLOUD
   * Fill Code: UPDATE AS NECESSARY
   * ========================================*/
  sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_voxel_filtered, *pc2_cloud);
  pc2_cloud->header.frame_id=world_frame;
  pc2_cloud->header.stamp=ros::Time::now();
  object_pub.publish(pc2_cloud);
  }

  return 0;
}
