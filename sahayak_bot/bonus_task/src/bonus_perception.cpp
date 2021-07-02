#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro
#include <visualization_msgs/Marker.h>
#include <ebot_task4/legends.h>
#include <task5_msgs/Data_msg.h>
#include <task5_msgs/Flag_msg.h>
#include <nav_msgs/Odometry.h>
#include <bits/stdc++.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"


#include <geometry_msgs/Pose.h>

// MoveIt!
//#include "moveit/move_group_interface/move_group_interface.h"
//#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <thread>
#include<map>
#include <std_msgs/Int64.h>

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

int f=0;
int flag=0;
int flag1=0;
int flag2=0;
int another_flag=0;

void flag_callback(const std_msgs::Int64::ConstPtr& myMsg)
{
    int w=myMsg->data;
	flag=w;
}

void flag_callback1(const std_msgs::Int64::ConstPtr& myMsg)
{
    int w=myMsg->data;
	flag1=w;
}

void flag_callback2(const std_msgs::Int64::ConstPtr& myMsg)
{
    int w=myMsg->data;
	flag2=w;
}


int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle n;
  ros::NodeHandle priv_nh_("~");
////  moveit::planning_interface::MoveGroupInterface group("arm_controller");
////  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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

  ros::Publisher object1,object2,object3,object4,object5,display_publisher,task5_msg;
  ros::Publisher object6,object7,object8,object9,object10;
  ros::Subscriber sub,sub1,sub2,sub3;
  sub1 = n.subscribe("testing", 1000, flag_callback);
  sub2 = n.subscribe("testing_1", 1000, flag_callback1);
  sub3 = n.subscribe("testing_2", 1000, flag_callback2);
  object1= nh.advertise<sensor_msgs::PointCloud2>("object1", 1);
  object2= nh.advertise<sensor_msgs::PointCloud2>("object2", 1);
  object3= nh.advertise<sensor_msgs::PointCloud2>("object3", 1);
  object4= nh.advertise<sensor_msgs::PointCloud2>("object4", 1);
  object5= nh.advertise<sensor_msgs::PointCloud2>("object5", 1);
  object6= nh.advertise<sensor_msgs::PointCloud2>("object6", 1);
  object7= nh.advertise<sensor_msgs::PointCloud2>("object7", 1);
  object8= nh.advertise<sensor_msgs::PointCloud2>("object8", 1);
  object9= nh.advertise<sensor_msgs::PointCloud2>("object9", 1);
  object10= nh.advertise<sensor_msgs::PointCloud2>("object10", 1);
  task5_msg=nh.advertise<task5_msgs::Data_msg>("object_data",1);
  display_publisher=nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  std::cout<<"sarthak is gadha"<<std::endl;
  std::cout<<"sarthak is gadha"<<std::endl;
  std::cout<<"sarthak is gadha"<<std::endl;
  std::cout<<"sarthak is gadha"<<std::endl;
  std::cout<<"sarthak is gadha"<<std::endl;
  std::cout<<"sarthak is gadha"<<std::endl;
while (ros::ok())
{

	if (flag2>1)
	{
		int ind;
		if(f==0){
			ind=flag2-1;
			f=1;
		}
	   //LISTEN FOR POINTCLOUD
	  std::string topic = nh.resolveName(cloud_topic);
	  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
	  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
				   ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

	   //TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
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

	   //CONVERT POINTCLOUD ROS->PCL
	  pcl::PointCloud<pcl::PointXYZRGB> cloud;
	  pcl::fromROSMsg (transformed_cloud, cloud);

	   //========================================
	   //Fill Code: VOXEL GRID
	   //========================================
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

	   //========================================
	   //CONVERT POINTCLOUD PCL->ROS
	   //PUBLISH CLOUD
	   //Fill Code: UPDATE AS NECESSARY
	   //========================================
	  int x=clusters.size();
	  Eigen::Vector4f centroid0,centroid1,centroid2,centroid3,centroid4;
	  Eigen::Vector4f centroid5,centroid6,centroid7,centroid8,centroid9;
	  std::cout<<"Number of Point Clouds= "<<x<<std::endl;
	  std::map<int,std::vector<float> >mp;
	  std::vector<float>V;
	  std::vector<std::pair<float,int>>mp_ans;
	  for(int i=0;i<x;i++){
		  V.clear();
		if(i==0){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(0)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object1.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(0)), centroid0);
		  float temp_y=centroid0[1];
		  if(ind>3)
		  {
			  centroid0[1]=-centroid0[1];
		  }
		  if(centroid0[2]>=0.4&&centroid0[1]<=0.1)
		  {
			V.push_back(centroid0[0]);
			V.push_back(centroid0[1]);
			V.push_back(centroid0[2]);
			mp[i]=V;
			mp_ans.emplace_back(temp_y,i);
		  }
		  std::cout<<"Object1 : " << centroid0[0]<<" "<<centroid0[1]<<" "<< centroid0[2]<<std::endl;
		}
		if(i==1){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(1)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object2.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(1)), centroid1);
		  float temp_y=centroid1[1];
		  if(ind>3)
		  {
			  centroid1[1]=-centroid1[1];
		  }
		  if(centroid1[2]>=0.4&&centroid1[1]<=0.1)
		  {
			V.push_back(centroid1[0]);
			V.push_back(centroid1[1]);
			V.push_back(centroid1[2]);
			mp_ans.emplace_back(temp_y,i);
			mp[i]=V;
		  }
		  std::cout<<"Object2 : " << centroid1[0]<<" "<<centroid1[1]<<" "<< centroid1[2]<<std::endl;
		}
		if(i==2){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(2)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object3.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(2)), centroid2);
		  float temp_y=centroid2[1];
		  if(ind>3)
		  {
			  centroid2[1]=-centroid2[1];
		  }
		  if(centroid2[2]>=0.4&&centroid2[1]<=0.1)
		  {
			V.push_back(centroid2[0]);
			V.push_back(centroid2[1]);
			V.push_back(centroid2[2]);
			mp_ans.emplace_back(temp_y,i);
			mp[i]=V;
		  }
		  task5_msgs::Data_msg some_object;
		  some_object.x2=centroid2[0];
		  some_object.y2=centroid2[1];
		  some_object.z2=centroid2[2];
		  std::cout<<"Object3 : " << centroid2[0]<<" "<<centroid2[1]<<" "<< centroid2[2]<<std::endl;
		}
		if(i==3){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(3)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object4.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(3)), centroid3);
		  float temp_y=centroid3[1];
		  if(ind>3)
		  {
			  centroid3[1]=-centroid3[1];
		  }
		  if(centroid3[2]>=0.4&&centroid3[1]<=0.1)
		  {
			V.push_back(centroid3[0]);
			V.push_back(centroid3[1]);
			V.push_back(centroid3[2]);
			mp_ans.emplace_back(temp_y,i);
			mp[i]=V;
		  }
		  std::cout<<"Object4 : " << centroid3[0]<<" "<<centroid3[1]<<" "<< centroid3[2]<<std::endl;
		}
		if(i==4){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(4)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object5.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(4)), centroid4);
		  float temp_y=centroid4[1];
		  if(ind>3)
		  {
			  centroid4[1]=-centroid4[1];
		  }
		  if(centroid4[2]>=0.4&&centroid4[1]<=0.1)
		  {
			V.push_back(centroid4[0]);
			V.push_back(centroid4[1]);
			V.push_back(centroid4[2]);
			mp_ans.emplace_back(temp_y,i);
			mp[i]=V;
		  }
		  std::cout<<"Object5 : " << centroid4[0]<<" "<<centroid4[1]<<" "<< centroid4[2]<<std::endl;
		}
		if(i==5){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(5)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object6.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(5)), centroid5);
		  float temp_y=centroid5[1];
		  if(ind>3)
		  {
			  centroid5[1]=-centroid5[1];
		  }
		  if(centroid5[2]>=0.4&&centroid5[1]<=0.1)
		  {
			V.push_back(centroid5[0]);
			V.push_back(centroid5[1]);
			V.push_back(centroid5[2]);
			mp_ans.emplace_back(temp_y,i);
			mp[i]=V;
		  }
		  std::cout<<"Object6 : " << centroid5[0]<<" "<<centroid5[1]<<" "<< centroid5[2]<<std::endl;
		}
		if(i==6){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(6)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object7.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(6)), centroid6);
		  float temp_y=centroid6[1];
		  if(ind>3)
		  {
			  centroid6[1]=-centroid6[1];
		  }
		  if(centroid6[2]>=0.4&&centroid6[1]<=0.1)
		  {
			V.push_back(centroid6[0]);
			V.push_back(centroid6[1]);
			V.push_back(centroid6[2]);
			mp_ans.emplace_back(temp_y,i);
			mp[i]=V;
		  }
		  std::cout<<"Object7 : " << centroid6[0]<<" "<<centroid6[1]<<" "<< centroid6[2]<<std::endl;
		}
		if(i==7){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(7)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object8.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(7)), centroid7);
		  float temp_y=centroid7[1];
		  if(ind>3)
		  {
			  centroid7[1]=-centroid7[1];
		  }
		  if(centroid7[2]>=0.4&&centroid7[1]<=0.1)
		  {
			V.push_back(centroid7[0]);
			V.push_back(centroid7[1]);
			V.push_back(centroid7[2]);
			mp[i]=V;
			mp_ans.emplace_back(temp_y,i);
		  }
		  std::cout<<"Object8 : " << centroid7[0]<<" "<<centroid7[1]<<" "<< centroid7[2]<<std::endl;
		}
		if(i==8){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(8)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object9.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(8)), centroid8);
		  float temp_y=centroid8[1];
		  if(ind>3)
		  {
			  centroid8[1]=-centroid8[1];
		  }
		  if(centroid8[2]>=0.4&&centroid8[1]<=0.1)
		  {
			V.push_back(centroid8[0]);
			V.push_back(centroid8[1]);
			V.push_back(centroid8[2]);
			mp[i]=V;
			mp_ans.emplace_back(temp_y,i);
		  }
		  std::cout<<"Object9 : " << centroid8[0]<<" "<<centroid8[1]<<" "<< centroid8[2]<<std::endl;
		}
		if(i==9){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(9)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object10.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(9)), centroid9);
		  float temp_y=centroid9[1];
		  if(ind>3)
		  {
			  centroid9[1]=-centroid9[1];
		  }
		  if(centroid9[2]>=0.4&&centroid9[1]<=0.1)
		  {
			V.push_back(centroid9[0]);
			V.push_back(centroid9[1]);
			V.push_back(centroid9[2]);
			mp[i]=V;
			mp_ans.emplace_back(temp_y,i);
		  }
		  std::cout<<"Object10 : " << centroid9[0]<<" "<<centroid9[1]<<" "<< centroid9[2]<<std::endl;
		}
	  }
	  	std::vector<std::pair<float,std::vector<float>>>vec;
		std::cout<<"INDEX in pcl: "<<ind<<std::endl;
		if(ind<=3){
			for(auto i:mp)
			{
				vec.emplace_back(i.second[1],i.second);
			}
			std::cout<<"Vector Size: "<<vec.size()<<" "<<std::endl;
			std::sort(vec.rbegin(),vec.rend());
			task5_msgs::Data_msg some_object;
	        some_object.x3=vec[ind-1].second[0];
	        some_object.y3=vec[ind-1].second[1];
	        some_object.z3=vec[ind-1].second[2];
	        task5_msg.publish(some_object);

		}
		if(ind>=4&&ind<=6){
			for(auto i:mp)
			{
				vec.emplace_back(i.second[1],i.second);
			}
			ind=ind-3;
			std::cout<<"Vector Size: "<<vec.size()<<" "<<std::endl;
			std::sort(vec.begin(),vec.end());
			task5_msgs::Data_msg some_object;
	        some_object.x3=vec[ind-1].second[0];
	        some_object.y3=-vec[ind-1].second[1];
	        some_object.z3=vec[ind-1].second[2];
	        task5_msg.publish(some_object);

		}
		flag2=0;
		break;
	}


	if (flag1>1)
	{
	   //LISTEN FOR POINTCLOUD
	  std::string topic = nh.resolveName(cloud_topic);
	  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
	  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
				   ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

	   //TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
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

	   //CONVERT POINTCLOUD ROS->PCL
	  pcl::PointCloud<pcl::PointXYZRGB> cloud;
	  pcl::fromROSMsg (transformed_cloud, cloud);

	   //========================================
	   //Fill Code: VOXEL GRID
	   //========================================
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

	   //========================================
	   //CONVERT POINTCLOUD PCL->ROS
	   //PUBLISH CLOUD
	   //Fill Code: UPDATE AS NECESSARY
	   //========================================
	  int x=clusters.size();
	  Eigen::Vector4f centroid0,centroid1,centroid2,centroid3,centroid4;
	  Eigen::Vector4f centroid5,centroid6,centroid7,centroid8,centroid9;
	  std::cout<<"Number of Point Clouds= "<<x<<std::endl;
	  std::map<int,std::vector<float> >mp;
	  std::vector<float>V;
	  std::map<float,int>mp_ans;
	  for(int i=0;i<x;i++){
		  V.clear();
		if(i==0){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(0)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object1.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(0)), centroid0);
		  if(centroid0[2]>=0.25)
		  {
			V.push_back(centroid0[0]);
			V.push_back(centroid0[1]);
			V.push_back(centroid0[2]);
			mp[i]=V;
			mp_ans[centroid0[0]]=i;

		  }
		  std::cout<<"Object1 : " << centroid0[0]<<" "<<centroid0[1]<<" "<< centroid0[2]<<std::endl;
		}
		if(i==1){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(1)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object2.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(1)), centroid1);
		  if(centroid1[2]>=0.25)
		  {
			V.push_back(centroid1[0]);
			V.push_back(centroid1[1]);
			V.push_back(centroid1[2]);
			mp_ans[centroid1[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object2 : " << centroid1[0]<<" "<<centroid1[1]<<" "<< centroid1[2]<<std::endl;
		}
		if(i==2){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(2)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object3.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(2)), centroid2);
		  if(centroid2[2]>=0.25)
		  {
			V.push_back(centroid2[0]);
			V.push_back(centroid2[1]);
			V.push_back(centroid2[2]);
			mp_ans[centroid2[0]]=i;
			mp[i]=V;
		  }
		  task5_msgs::Data_msg some_object;
		  some_object.x2=centroid2[0];
		  some_object.y2=centroid2[1];
		  some_object.z2=centroid2[2];
		  std::cout<<"Object3 : " << centroid2[0]<<" "<<centroid2[1]<<" "<< centroid2[2]<<std::endl;
		}
		if(i==3){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(3)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object4.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(3)), centroid3);
		  if(centroid3[2]>=0.25)
		  {
			V.push_back(centroid3[0]);
			V.push_back(centroid3[1]);
			V.push_back(centroid3[2]);
			mp_ans[centroid3[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object4 : " << centroid3[0]<<" "<<centroid3[1]<<" "<< centroid3[2]<<std::endl;
		}
		if(i==4){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(4)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object5.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(4)), centroid4);
		  if(centroid4[2]>=0.25)
		  {
			V.push_back(centroid4[0]);
			V.push_back(centroid4[1]);
			V.push_back(centroid4[2]);
			mp_ans[centroid4[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object5 : " << centroid4[0]<<" "<<centroid4[1]<<" "<< centroid4[2]<<std::endl;
		}
		if(i==5){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(5)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object6.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(5)), centroid5);
		  if(centroid5[2]>=0.25)
		  {
			V.push_back(centroid5[0]);
			V.push_back(centroid5[1]);
			V.push_back(centroid5[2]);
			mp_ans[centroid5[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object6 : " << centroid5[0]<<" "<<centroid5[1]<<" "<< centroid5[2]<<std::endl;
		}
		if(i==6){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(6)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object7.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(6)), centroid6);
		  if(centroid6[2]>=0.25)
		  {
			V.push_back(centroid6[0]);
			V.push_back(centroid6[1]);
			V.push_back(centroid6[2]);
			mp_ans[centroid6[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object7 : " << centroid6[0]<<" "<<centroid6[1]<<" "<< centroid6[2]<<std::endl;
		}
		if(i==7){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(7)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object8.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(7)), centroid7);
		  if(centroid7[2]>=0.25)
		  {
			V.push_back(centroid7[0]);
			V.push_back(centroid7[1]);
			V.push_back(centroid7[2]);
			mp_ans[centroid7[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object8 : " << centroid7[0]<<" "<<centroid7[1]<<" "<< centroid7[2]<<std::endl;
		}
		if(i==8){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(8)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object9.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(8)), centroid8);
		   if(centroid8[2]>=0.25)
		  {
			V.push_back(centroid8[0]);
			V.push_back(centroid8[1]);
			V.push_back(centroid8[2]);
			mp_ans[centroid8[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object9 : " << centroid8[0]<<" "<<centroid8[1]<<" "<< centroid8[2]<<std::endl;
		}
		if(i==9){
		  sensor_msgs::PointCloud2::Ptr pc2_cloud3 (new sensor_msgs::PointCloud2);
		  pcl::toROSMsg(*(clusters.at(9)), *pc2_cloud3);
		  pc2_cloud3->header.frame_id=world_frame;
		  pc2_cloud3->header.stamp=ros::Time::now();
		  object10.publish(pc2_cloud3);
		  pcl::compute3DCentroid (*(clusters.at(9)), centroid9);
		   if(centroid9[2]>=0.25)
		  {
			V.push_back(centroid9[0]);
			V.push_back(centroid9[1]);
			V.push_back(centroid9[2]);
			mp_ans[centroid9[0]]=i;
			mp[i]=V;
		  }
		  std::cout<<"Object10 : " << centroid9[0]<<" "<<centroid9[1]<<" "<< centroid9[2]<<std::endl;
		}
	  }

	  int index=flag1-1;
	  int c=0;
	  int ind;
	  for(auto i:mp_ans){
	  	  c++;
	  	  if(c==index){
	  	  	ind=i.second;
	  	 }
	  }
	  task5_msgs::Data_msg some_object;
	  some_object.x2=mp[ind][0];
	  some_object.y2=mp[ind][1];
	  some_object.z2=mp[ind][2];
	 task5_msg.publish(some_object);
	 // flag1=0;
	 // break;
	}

	ros::spinOnce();
}
  return 0;
}
