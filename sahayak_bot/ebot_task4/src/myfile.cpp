#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro
#include<geometry_msgs/PointStamped.h>

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
  tf::TransformListener* listenerr;
  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::string cloud_topic, world_frame, camera_frame;
  float voxel_leaf_size;
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  int plane_max_iter,num=0;
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
  pose_pub = nh.advertise<geometry_msgs::PointStamped>("quad_pos", 1);
  listenerr = new tf::TransformListener();

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
pcl::PointCloud<pcl::PointXYZRGB> xyz_filtered_cloud;
pcl::CropBox<pcl::PointXYZRGB> crop;
crop.setInputCloud(cloud_voxel_filtered);
Eigen::Vector4f min_point = Eigen::Vector4f(-1.0, -1.0, -1.0, 0);
Eigen::Vector4f max_point = Eigen::Vector4f(1.0, 1.0, 1.0, 0);
crop.setMin(min_point);
crop.setMax(max_point);
crop.filter(xyz_filtered_cloud);


/* ========================================
 * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
 * ========================================*/


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr= clusters.at(num);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
sor.setInputCloud (cluster_cloud_ptr);
sor.setMeanK (50);
sor.setStddevMulThresh (1.0);

sor.filter (*sor_cloud_filtered);

/* ========================================
 * BROADCAST TRANSFORM (OPTIONAL)
 * ========================================*/

static tf::TransformBroadcaster br;
tf::Transform part_transform;

//Here in the tf::Vector3(x,y,z) x,y, and z should be calculated based on the pointcloud filtering results
part_transform.setOrigin( tf::Vector3(sor_cloud_filtered->at(1).x, sor_cloud_filtered->at(1).y, sor_cloud_filtered->at(1).z) );
tf::Quaternion q;
q.setRPY(0, 0, 0);
part_transform.setRotation(q);

br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "part"));


Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*sor_cloud_filtered, centroid);

// publish centroid 
	geometry_msgs::PointStamped out_pos;
  	out_pos.header.frame_id = world_frame;
	out_pos.header.stamp = ros::Time();
	out_pos.point.x = centroid[0];
	out_pos.point.y = centroid[1];
	out_pos.point.z = centroid[2];
	geometry_msgs::PointStamped base_point;
    listenerr->transformPoint(world_frame, out_pos, base_point);
	pose_pub.publish(base_point);


  /* ========================================
   * CONVERT POINTCLOUD PCL->ROS
   * PUBLISH CLOUD
   * Fill Code: UPDATE AS NECESSARY
   * ========================================*/

  sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*sor_cloud_filtered, *pc2_cloud);
  pc2_cloud->header.frame_id=world_frame;
  pc2_cloud->header.stamp=ros::Time::now();
  object_pub.publish(pc2_cloud);

  }

  return 0;
}
