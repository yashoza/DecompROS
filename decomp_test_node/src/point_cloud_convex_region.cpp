#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
// #include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <decomp_util/seed_decomp.h>
#include <visualization_msgs/MarkerArray.h>

std_msgs::Header header_;

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);

  std::string file_name, topic_name, path_file;

  nh.param("path_file", path_file, std::string("path.txt"));
  nh.param("bag_file", file_name, std::string("voxel_map"));
  nh.param("bag_topic", topic_name, std::string("voxel_map"));
  //Read the point cloud from bag
  sensor_msgs::PointCloud cloud = read_bag<sensor_msgs::PointCloud>(file_name, topic_name);
  cloud.header.frame_id = "map";
  cloud_pub.publish(cloud);

  vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);
  
  vec_Vec2f obs2d;
  for(const auto& it: obs)
    obs2d.push_back(it.topRows<2>());

  //Read path from txt
  vec_Vec2f path;
  if(!read_path<2>(path_file, path))
    ROS_ERROR("Fail to read a path!");

  nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
  path_msg.header.frame_id = "map";
  path_pub.publish(path_msg);

  //Using ellipsoid decomposition
  EllipsoidDecomp2D decomp_util;
  decomp_util.set_obs(obs2d);
  decomp_util.set_local_bbox(Vec2f(1, 2));
  decomp_util.dilate(path); //Set max iteration number of 10, do fix the path

  //Publish visualization msgs
  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
  es_msg.header.frame_id = "map";
  es_pub.publish(es_msg);

  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
  poly_msg.header.frame_id = "map";
  poly_pub.publish(poly_msg);



#if 1
  
  // starting new part from here 

  // ros::init(argc, argv, "test");
  // ros::NodeHandle nh("~");

  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud>("cloud_new", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  ros::Publisher seed_pub = nh.advertise<sensor_msgs::PointCloud>("seed", 1, true);
  ros::Publisher es_pub_new = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array_new", 1, true);
  ros::Publisher poly_pub_new = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array_new", 1, true);
  // ros::Subscriber pointclsub = nh.subscribe("/png2pcd_test/cloud", 1000, pointCloudCallback);
  ros::Publisher point_pub = nh.advertise<visualization_msgs::Marker>("seed_point", 1, true);


  ROS_INFO("launched the publishers ");

  header_.frame_id = std::string("map");
  std::string file_name_new, topic_name_new, marker_name_new, seed_file_new;

  nh.param("seed_file", seed_file_new, std::string("seeds_test.txt"));
  // nh.param("bag_file", file_name_new, std::string("voxel_map"));
  // nh.param("bag_topic", topic_name_new, std::string("voxel_map"));
  nh.param("bag_marker", marker_name_new, std::string("voxel_map"));
  // std::cout << "bag file  " << file_name_new << std::endl;
  ROS_INFO("finished reading params, now waiting for pcl msg");

  //Read the point cloud from bag
  // sensor_msgs::PointCloud2 cloud2 = read_bag<sensor_msgs::PointCloud2>(file_name_new, topic_name_new);
  //Read the point cloud from bag
  
  auto cloud1 = ros::topic::waitForMessage<sensor_msgs::PointCloud>("/png2pcd_test/cloud", nh, ros::Duration(60.0));
  vec_Vec3f obs_new2 = DecompROS::cloud_to_vec(*cloud1);
  
  // auto cloud11 = ros::topic::waitForMessage<std_msgs::String>("png2pcd_test/cloud", nh, ros::Duration(10.0));
  // std::cout << cloud11 << std::endl;
  
  // sensor_msgs::PointCloud cloud1 = read_bag<sensor_msgs::PointCloud>(file_name, topic_name);
  // vec_Vec3f obs_new2 = DecompROS::cloud_to_vec(cloud1);

  ROS_INFO("finished reading pcl from bag file ");

  //Convert into vector of Eigen
  // sensor_msgs::PointCloud cloud1;
  // sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud1);
  // cloud1.header = header_;
  map_pub.publish(cloud1);

  ROS_INFO("finished converting and publishing");


  vec_Vec2f obs_new2d;
  for(const auto& it: obs_new2)
    obs_new2d.push_back(it.topRows<2>());

  visualization_msgs::MarkerArray markers = read_bag<visualization_msgs::MarkerArray>(file_name, marker_name_new);
  for(auto & it: markers.markers)
    it.header = header_;
  marker_pub.publish(markers);

  //Read path from txt
  vec_Vec2f seeds;
  if(!read_path<2>(seed_file_new, seeds))
    ROS_ERROR("Fail to read seeds!");

  ROS_INFO("finished reading seeds ");


  // sensor_msgs::PointCloud seed_msg = DecompROS::vec_to_cloud(seeds);
  // seed_msg.header = header_;
  // seed_pub.publish(seed_msg);

  ROS_INFO("finished publishing seeds ");

  vec_E<Ellipsoid2D> es_new;
  vec_E<Polyhedron2D> polys_new;

  // TODO
  // test for -13 -5 on simple_image_6

  visualization_msgs::Marker point_marker;

  for(const auto& it: seeds) {
    
    point_marker.header.frame_id = "map";
    point_marker.header.stamp = ros::Time();
    point_marker.ns = "my_namespace";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = it[0];
    point_marker.pose.position.y = it[1];
    point_marker.pose.position.z = 1;
    point_marker.pose.orientation.x = 0.0;
    point_marker.pose.orientation.y = 0.0;
    point_marker.pose.orientation.z = 0.0;
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.2;
    point_marker.scale.y = 0.2;
    point_marker.scale.z = 0.2;
    point_marker.color.a = 1.0; // Don't forget to set the alpha!
    point_marker.color.r = 0.0;
    point_marker.color.g = 1.0;
    point_marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    
    // std::cout << "it" << it[0] << std::endl;
    // std::cout << "it" << it[1] << std::endl;
    
    //Using seed decomposition
    SeedDecomp2D decomp_util(it);
    decomp_util.set_obs(obs_new2d);
    decomp_util.dilate(5.0);

    es_new.push_back(decomp_util.get_ellipsoid());
    polys_new.push_back(decomp_util.get_polyhedron());

    ROS_INFO("ran a poly search loop");
  }

  //Publish visualization msgs
  decomp_ros_msgs::EllipsoidArray es_msg_new = DecompROS::ellipsoid_array_to_ros(es_new);
  es_msg_new.header = header_;
  es_pub_new.publish(es_msg_new);

  decomp_ros_msgs::PolyhedronArray poly_msg_new = DecompROS::polyhedron_array_to_ros(polys_new);
  poly_msg_new.header = header_;
  poly_pub_new.publish(poly_msg_new);

  point_pub.publish( point_marker );

// ending new part here 
#endif 

  ros::spin();

  return 0;
}
