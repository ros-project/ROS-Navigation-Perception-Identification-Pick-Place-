#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/projection_matrix.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>


ros::Publisher pub;

class AddBoundingBox
{
public:
  AddBoundingBox() 
  {
     ros::NodeHandle nh;
     //add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
     ros::Subscriber sub = nh.subscribe("/darknet_ros_3d_markers", 1, &AddBoundingBox::cloudCB, this);
    ros::spin();
  }


   void addBox()
       {
        
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_new(new pcl::PointCloud<pcl::PointXYZRGB pcl_point(point.x, point.y, point.z>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_new(new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;

		//const std::string PLANNING_GROUP = "crane";
		const std::string PLANNING_GROUP = "crane_control";
    	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    		
		// Define a collision object ROS message.
		moveit_msgs::CollisionObject collision_object;
		//collision_object.header.frame_id = "zed_left_camera_optical_frame";
        collision_object.header.frame_id = move_group.getPlanningFrame();
		collision_object.id = "bounding_box";

		// Define a CUBE  with h, w, d which will be added to the world.
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr output;
        Eigen::Vector4f centroid;
        Eigen::Vector4f min;
	    Eigen::Vector4f max;
        
        //double xmin, xmin, ymin, ymax, zmin, zmax;
  		pcl::getMinMax3D (*cloud_cluster_new, min, max);
		/*min[0] = xmin;
		min[1] = ymin;
		min[2] = zmin;
		max[0] = xmax;
		max[1] = ymax;
		min[2] = zmax;*/
		/* Setting width of cube. */
		primitive.dimensions[0] = (min[0]-max[0]);
		/* Setting height of cube. */
		primitive.dimensions[1] = (min[1]-max[1]);
        /* Setting radius of cylinder. */
		primitive.dimensions[1] = (min[2]-max[1]);

		// Define a pose for the cube (specified relative to frame_id).
    	geometry_msgs::Pose cube_pose;
  		pcl::compute3DCentroid (*cloud_cluster_new, centroid);
    	/* Computing and setting quaternion from axis angle representation. */
		
		// Setting the position of cube.
    	//centroid[0] = cube_pose.position.x;
    	//centroid[1] = cube_pose.position.y;
    	//centroid[2] = cube_pose.position.z;
		cube_pose.position.x = centroid[0];
 		cube_pose.position.y = centroid[1];
  		cube_pose.position.z = centroid[2];

		feature_extractor.setInputCloud(cloud_cluster_new);
		feature_extractor.compute();
		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter(mass_center);
        

		pcl::PointXYZRGB center(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
		Eigen::Vector3f x_axis(major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
		Eigen::Vector3f y_axis(middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
		Eigen::Vector3f z_axis(minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
		//pcl::PointXYZRGB x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
		//pcl::PointXYZRGB y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
		//pcl::PointXYZRGB z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));

		cube_pose.orientation.x =  x_axis.x();
		cube_pose.orientation.y =  y_axis.y();
		cube_pose.orientation.z =  z_axis.z();
			
		pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
 
		//viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
		//viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
		//viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(cube_pose);
		collision_object.operation = collision_object.ADD;

		std::vector<moveit_msgs::CollisionObject> collision_objects;
		collision_objects.push_back(collision_object);
		//Now, let’s add the collision object into the world
		ROS_INFO_NAMED("tutorial", "Add an object into the world");
		planning_scene_interface.applyCollisionObject(collision_object);
		//move_group.attachObject(collision_object.id);
		
	}

 	visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b)
	{
     
	  visualization_msgs::Marker marker;
      uint32_t shape = visualization_msgs::Marker::CUBE;
	  marker.header.frame_id = cloud_cluster->header.frame_id;
	  marker.header.stamp = ros::Time::now();

      marker.header.frame_id = "/darknet_ros_3d_markers";

	  Eigen::Vector4f centroid;
	  Eigen::Vector4f min;
	  Eigen::Vector4f max;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output;

	  pcl::compute3DCentroid (*cloud_cluster, centroid);
	  pcl::getMinMax3D (*cloud_cluster, min, max);

	  
	  marker.ns = ns;
	  marker.id = id;
	  marker.type = shape;
	  marker.action = visualization_msgs::Marker::ADD;

	  marker.pose.position.x = centroid[0];
	  marker.pose.position.y = centroid[1];
	  marker.pose.position.z = centroid[2];
	  marker.pose.orientation.x = 0.0;
	  marker.pose.orientation.y = 0.0;
	  marker.pose.orientation.z = 0.0;
	  marker.pose.orientation.w = 1.0;

	  marker.scale.x = (max[0]-min[0]);
	  marker.scale.y = (max[1]-min[1]);
	  marker.scale.z = (max[2]-min[2]);

	  if (marker.scale.x ==0)
		  marker.scale.x=0.1;

	  if (marker.scale.y ==0)
		marker.scale.y=0.1;

	  if (marker.scale.z ==0)
		marker.scale.z=0.1;

	  marker.color.r = r;
	  marker.color.g = g;
	  marker.color.b = b;
	  marker.color.a = 0.5;

	  pub.publish(marker);

	  marker.lifetime = ros::Duration();
	  //marker.lifetime = ros::Duration(0.5);
	  return marker;
	}


    void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
   		{
   
    	// First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for most of the processing.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*input, *cloud_cluster);
        //pcl_conversions::toPCL(*input,  *cloud_cluster);
      	//applyCollisionObject(cloud_cluster);
		addBox();
   		}

};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tracking");
  AddBoundingBox();
  while(ros::ok())
	{
		ros::spin();	
	}
	return EXIT_SUCCESS;
}
