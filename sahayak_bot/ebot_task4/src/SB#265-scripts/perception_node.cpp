#include <ros/ros.h>
#include "object_msgs/ObjectPose.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro
#include <ebot_task4/legends.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"


#include <geometry_msgs/Pose.h>

// MoveIt!
//#include "moveit/move_group_interface/move_group_interface.h"
//#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <pluginlib/class_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

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

int c=0;
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
  std::string cloud_topic, world_frame, camera_frame,nn;
  float voxel_leaf_size;
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  int plane_max_iter;
  float plane_dist_thresh;
  float cluster_tol;
  int cluster_min_size;
  int cluster_max_size;
  int num;
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
  num= priv_nh_.param<int>("num",0);


  /*
   * SETUP PUBLISHERS
   */
  ros::Publisher table_pub,all_objects,object_pub,object_pub1,object_pub2,pose_pub,detection_info,legends,voxel_filter_cloud;
  table_pub = nh.advertise<sensor_msgs::PointCloud2>("table_pub", 1);
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
  object_pub1 = nh.advertise<sensor_msgs::PointCloud2>("object_cluster1", 1);
  object_pub2 = nh.advertise<sensor_msgs::PointCloud2>("object_cluster2", 1);
  all_objects= nh.advertise<sensor_msgs::PointCloud2>("all_objects", 1);
  voxel_filter_cloud= nh.advertise<sensor_msgs::PointCloud2>("voxel_filter_cloud", 1);
  detection_info=nh.advertise<object_msgs::ObjectPose>("detection_info",1);
  legends=nh.advertise<ebot_task4::legends>("legends",1);

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


  pcl::PointCloud<pcl::PointXYZRGB> xf_cloud, yf_cloud, zf_cloud;
pcl::PassThrough<pcl::PointXYZRGB> pass_x;
pass_x.setInputCloud(cloud_voxel_filtered);
pass_x.setFilterFieldName("x");
pass_x.setFilterLimits(-1.0,1.0);
pass_x.filter(xf_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(xf_cloud));
pcl::PassThrough<pcl::PointXYZRGB> pass_y;
pass_y.setInputCloud(xf_cloud_ptr);
pass_y.setFilterFieldName("y");
pass_y.setFilterLimits(-1.0, 1.0);
pass_y.filter(yf_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(yf_cloud));
pcl::PassThrough<pcl::PointXYZRGB> pass_z;
pass_z.setInputCloud(yf_cloud_ptr);
pass_z.setFilterFieldName("z");
pass_z.setFilterLimits(-1.0, 1.0);
pass_z.filter(zf_cloud);


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(zf_cloud));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
// Create the segmentation object for the planar model and set all the parameters
pcl::SACSegmentation<pcl::PointXYZRGB> seg;
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
seg.setOptimizeCoefficients (true);
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setMaxIterations (200);
seg.setDistanceThreshold (0.004);
// Segment the largest planar component from the cropped cloud
seg.setInputCloud (cropped_cloud);
seg.segment (*inliers, *coefficients);
if (inliers->indices.size () == 0)
{
  ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
  //break;
}

// Extract the planar inliers from the input cloud
pcl::ExtractIndices<pcl::PointXYZRGB> extract;
extract.setInputCloud (cropped_cloud);
extract.setIndices(inliers);
extract.setNegative (false);

// Get the points associated with the planar surface
extract.filter (*cloud_plane);
ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

extract.setNegative (true);
extract.filter (*cloud_f);


// Creating the KdTree object for the search method of the extraction
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
*cloud_filtered = *cloud_f;
tree->setInputCloud (cloud_filtered);

std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
ec.setClusterTolerance (0.01); // 2cm
ec.setMinClusterSize (300);
ec.setMaxClusterSize (10000);
ec.setSearchMethod (tree);
ec.setInputCloud (cloud_filtered);
ec.extract (cluster_indices);

std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters;
for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
  std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
  clusters.push_back(cloud_cluster);
  sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
  pc2_clusters.push_back(tempROSMsg);
}
/* ========================================
 * Fill Code: CROPBOX (OPTIONAL)
 * ========================================*/


/* ========================================
 * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
 * ========================================*/

/* ========================================
 * BROADCAST TRANSFORM (OPTIONAL)
 * ========================================*/
  /* ========================================
   * CONVERT POINTCLOUD PCL->ROS
   * PUBLISH CLOUD
   * Fill Code: UPDATE AS NECESSARY
   * ========================================*/

  sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_plane, *pc2_cloud);
  pc2_cloud->header.frame_id=world_frame;
  pc2_cloud->header.stamp=ros::Time::now();
  table_pub.publish(pc2_cloud);

  sensor_msgs::PointCloud2::Ptr pc2_cloud1 (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(clusters.at(1)), *pc2_cloud1);
  pc2_cloud1->header.frame_id=world_frame;
  pc2_cloud1->header.stamp=ros::Time::now();
  object_pub1.publish(pc2_cloud1);

  sensor_msgs::PointCloud2::Ptr pc2_cloud2 (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(clusters.at(2)), *pc2_cloud2);
  pc2_cloud2->header.frame_id=world_frame;
  pc2_cloud2->header.stamp=ros::Time::now();
  object_pub2.publish(pc2_cloud2);

  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*(clusters.at(0)), *pc2_cloud3);
  pc2_cloud3->header.frame_id=world_frame;
  pc2_cloud3->header.stamp=ros::Time::now();
  object_pub.publish(pc2_cloud3);

  sensor_msgs::PointCloud2::Ptr pc2_cloud4 (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(zf_cloud, *pc2_cloud4);
  pc2_cloud4->header.frame_id=world_frame;
  pc2_cloud4->header.stamp=ros::Time::now();
  voxel_filter_cloud.publish(pc2_cloud4);

  sensor_msgs::PointCloud2::Ptr pc2_cloud5 (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_filtered, *pc2_cloud5);
  pc2_cloud5->header.frame_id=world_frame;
  pc2_cloud5->header.stamp=ros::Time::now();
  all_objects.publish(pc2_cloud5);

  Eigen::Vector4f centroid0,centroid1,centroid2;


pcl::compute3DCentroid (*(clusters.at(0)), centroid0);
pcl::compute3DCentroid (*(clusters.at(1)), centroid1);
pcl::compute3DCentroid (*(clusters.at(2)), centroid2);


if(c==0){
  object_msgs::ObjectPose object_coke;
object_coke.name="Coke Can";
object_coke.pose.pose.position.x = centroid0[0];
object_coke.pose.pose.position.y = centroid0[1];
object_coke.pose.pose.position.z = centroid0[2];
detection_info.publish(object_coke);

for(int i=0;i<100000;i++);
for(int i=0;i<100000;i++);
for(int i=0;i<100000;i++);
for(int i=0;i<100000;i++);

object_msgs::ObjectPose object_battery;
object_battery.name="Battery";
object_battery.pose.pose.position.x = centroid1[0];
object_battery.pose.pose.position.y = centroid1[1];
object_battery.pose.pose.position.z = centroid1[2];
detection_info.publish(object_battery);

for(int i=0;i<100000;i++);
for(int i=0;i<100000;i++);
for(int i=0;i<100000;i++);
for(int i=0;i<100000;i++);

object_msgs::ObjectPose object_glue;
object_glue.name="Glue";
object_glue.pose.pose.position.x = centroid2[0];
object_glue.pose.pose.position.y = centroid2[1];
object_glue.pose.pose.position.z = centroid2[2];
detection_info.publish(object_glue);

}
ebot_task4::legends some_object;
some_object.x1=centroid0[0];
some_object.y1=centroid0[1];
some_object.z1=centroid0[2];

some_object.x2=centroid1[0];
some_object.y2=centroid1[1];
some_object.z2=centroid1[2];

some_object.x3=centroid2[0];
some_object.y3=centroid2[1];
some_object.z3=centroid2[2];
legends.publish(some_object);

std::cout<<"CocaCola : " << centroid0[0]<<" "<<centroid0[1]<<" "<< centroid0[2]<<std::endl;
std::cout<<"Battery : " << centroid1[0]<<" "<<centroid1[1]<<" "<< centroid1[2]<<std::endl;
std::cout<<"Glue : " << centroid2[0]<<" "<<centroid2[1]<<" "<< centroid2[2]<<std::endl;
c++;
}
  return 0;
}


