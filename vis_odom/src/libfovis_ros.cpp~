/*
 * libfovis_ros.cpp
 *
 *  Created on: 16-Jan-2013
 *      Author: karan
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <iostream>
#include <string>
#include <fovis/fovis.hpp>

#include </home/guru/ros_workspace/odoo/libfovis/libfovis/depth_image.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/visual_odometry.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/frame.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/motion_estimation.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/initial_homography_estimation.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/tictoc.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/fast.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/feature_matcher.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/gauss_pyramid.c>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/grid_filter.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/intensity_descriptor.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/internal_utils.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/normalize_image.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/pyramid_level.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/rectification.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/refine_feature_match.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/refine_motion_estimate.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/stereo_depth.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/stereo_frame.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/stereo_rectify.cpp>
#include </home/guru/ros_workspace/odoo/libfovis/libfovis/depth_traits.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

using namespace sensor_msgs;
using namespace message_filters;
using namespace konz;
typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

nav_msgs::Odometry Odom;
geometry_msgs::TransformStamped odom_trans;

namespace fovis_example
{
class DataCapture
{
public:
    fovis::DepthImage* depth_image;

    int width;
    int height;

    fovis::CameraIntrinsicsParameters rgb_params;

    float* depth_data;

    uint8_t* gray_buf;

DataCapture()
{

  width = 640;
	  //width = cam_info.width;
  height = 480;
  memset(&rgb_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
  rgb_params.width = width;
  rgb_params.height = height;

  // TODO read these values from the camera somehow, instead of hard-coding it
  // Unfortunately, the OpenNI API doesn't seem to expose them.
  rgb_params.fx = 528.49404721;
  rgb_params.fy = rgb_params.fx;
  rgb_params.cx = width / 2.0;
  rgb_params.cy = height / 2.0;
  depth_image = new fovis::DepthImage(rgb_params, width, height);
  depth_data = new float[width * height];
  gray_buf = new uint8_t[width * height];


}

~DataCapture()
{
  delete[] depth_data;
  delete[] gray_buf;
}

bool
initialize(const sensor_msgs::CameraInfo cam_info)
{

	  width = 640;
	  //width = cam_info.width;
	  height = 480;
	  //height =cam_info.height;
	  memset(&rgb_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
	  //rgb_params.width = width;
	  //rgb_params.height = height;
      //cout<<width<<" "<<height<<endl;
	  // TODO read these values from the camera somehow, instead of hard-coding it
	  // Unfortunately, the OpenNI API doesn't seem to expose them.
	  rgb_params.fx = 528.49404721;
	  rgb_params.fy = rgb_params.fx;
	  rgb_params.cx = width / 2.0;
	  rgb_params.cy = height / 2.0;
	  /*rgb_params.fx = cam_info.K[0];
	  rgb_params.fy = cam_info.K[4];
	  rgb_params.cx = cam_info.K[2];
	  rgb_params.cy = cam_info.K[5];*/
	//  cout<<rgb_params.fx<<" "<<rgb_params.fy<<rgb_params.cx<<" "<<rgb_params.cy<<endl;
      depth_image = new fovis::DepthImage(rgb_params, width, height);
	  depth_data = new float[width * height];
	  gray_buf = new uint8_t[width * height];




  return true;
}


fovis::DepthImage* getDepthImage() {
  return depth_image;
}

const fovis::CameraIntrinsicsParameters& getRgbParameters() const {
  return rgb_params;
}

const uint8_t* getGrayImage() {
  return gray_buf;
}
};
}

using namespace std;


std::string
isometryToString(const Eigen::Isometry3d& m)
{
  char result[80];
  memset(result, 0, sizeof(result));
  Eigen::Vector3d xyz = m.translation();
  Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
  snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
      xyz(0), xyz(1), xyz(2),
      rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
  return std::string(result);
}
fovis_example::DataCapture* cap;
fovis::Rectification *rect;
fovis::VisualOdometry* odom;
fovis::VisualOdometryOptions options;
float depth_data[640*480];
int k=0;
float* depth_data_ =new float[480*640];
float* prepareDepthImage(const sensor_msgs::ImageConstPtr& depth_msg)
{
  const float* depth_data_original = reinterpret_cast<const float*>(&depth_msg->data[0]);
  size_t size = depth_msg->width * depth_msg->height;
  for (size_t i = 0; i < size; i++)
  {
    if (converters::DepthTraits<float>::valid(depth_data_original[i]))
     depth_data_[i] = converters::DepthTraits<float>::toMeters(depth_data_original[i]);
    else
      depth_data_[i] = NAN;
  }
  return depth_data_;
}

cv::Mat mono_img_;

//nav_msgs::Odometry Odom;
geometry_msgs::Quaternion odom_quat;
ros::Publisher *odom_pub;

void imageCb(const ImageConstPtr& rgb_image,const ImageConstPtr& depth_image)
{
      mono_img_ = cv_bridge::toCvCopy(rgb_image, "mono8")->image;
    // Convert depth image from whatever format it has to what fovis expects.
      cap->depth_image->setDepthImage(prepareDepthImage(depth_image));
	  odom->processFrame(mono_img_.data,cap->getDepthImage());
	  Eigen::Isometry3d cam_to_local = odom->getPose();
	  Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();
	  Eigen::Vector3d xyz_pos = cam_to_local.translation();
	  Eigen::Vector3d rpy_pos = cam_to_local.rotation().eulerAngles(0, 1, 2);
	  Eigen::Vector3d Vxyz = motion_estimate.translation();
	  Eigen::Vector3d Vrpy = motion_estimate.rotation().eulerAngles(0, 1, 2);
	 //PRINT ROUTINE
	  char result[160];
	  memset(result, 0, sizeof(result));
	  snprintf(result, 159, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",xyz_pos(2),-xyz_pos(0), -xyz_pos(1),
	      (rpy_pos(2) * 180/M_PI),( -rpy_pos(0) * 180/M_PI),( -rpy_pos(1) * 180/M_PI), Vxyz(2), -Vxyz(0), -Vxyz(1),
	      (Vrpy(2) * 180/M_PI),( -Vrpy(0) * 180/M_PI),( -Vrpy(1) * 180/M_PI));
	  cout<<result<<endl;
	  /*char result2[80];
	  memset(result2, 0, sizeof(result2));
	  printf(result2, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",*/
	  
	  
	  ros::Time current_time = ros::Time::now();
	  
	  
	  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,-(double)rpy_pos(1));
	  
	  odom_trans.header.stamp = current_time;
	  odom_trans.header.frame_id = "/odom";
	  odom_trans.child_frame_id = "/base_footprint";

	  odom_trans.transform.translation.x = xyz_pos(2);
	  odom_trans.transform.translation.y = -xyz_pos(0);
	  odom_trans.transform.translation.z = 0.0;
	  odom_trans.transform.rotation = odom_quat;

	  //send the transform
	  
	  
	  //publish over ROS
	  
	  nav_msgs::Odometry Odom;
	  Odom.header.stamp = ros::Time::now();
	  Odom.header.frame_id = "/odom";
		    //set the position
	  //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw((double)rpy_pos(2),-(double)rpy_pos(0),-(double)rpy_pos(1));
	  //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,-(double)rpy_pos(1));
	  Odom.pose.pose.position.x = xyz_pos(2);
	  Odom.pose.pose.position.y = -xyz_pos(0);
	  Odom.pose.pose.position.z = 0.0;
	  Odom.pose.pose.orientation = odom_quat;
		    //set the velocity
      Odom.child_frame_id = "/base_footprint";
	  Odom.twist.twist.linear.x = Vxyz(2);
	  Odom.twist.twist.linear.y = -Vxyz(0);
	  //Odom.twist.twist.linear.z = -Vxyz(1);
	  //Odom.twist.twist.angular.x = Vrpy(2);
	  //Odom.twist.twist.angular.y = -Vrpy(0);
	  Odom.twist.twist.angular.z = -Vrpy(1);
	  Odom.pose.covariance[0]  = .01;
	  Odom.pose.covariance[7]  = .01;
	  Odom.pose.covariance[14] = .01;
	  Odom.pose.covariance[21] = .001;
	  Odom.pose.covariance[28] = .001;
	  Odom.pose.covariance[35] = .001;
	  Odom.twist.covariance[0]  = .01;
	  Odom.twist.covariance[7]  = .01;
	  Odom.twist.covariance[14] = .01;
	  Odom.twist.covariance[21] = .01;
	  Odom.twist.covariance[28] = .01;
	  Odom.twist.covariance[35] = .01;
	  //odom_pub->publish(Odom);
/*
 * camera coordinate frame
 * x right
 * y downwards
 * z forward
 * Rpy clockwise
 * map to world frame
 * x  forward
 * y  left
 * z  upwards
 * */


}







int main(int argc, char **argv)
{
  ros::init(argc, argv, "fovis");
  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;
  odom_pub = new ros::Publisher;
  *odom_pub = n.advertise<nav_msgs::Odometry>("vo", 50);
  message_filters::Subscriber<Image> imageRGB_sub(n, "camera/rgb/image_color", 1);
  message_filters::Subscriber<Image> imageDEPTH_sub(n, "camera/depth/image", 1);
  Synchronizer<MySyncPolicy> syncs(MySyncPolicy(10), imageRGB_sub, imageDEPTH_sub);
  syncs.registerCallback(boost::bind(&imageCb, _1, _2));
  cap = new fovis_example::DataCapture();
  rect = new fovis::Rectification(cap->getRgbParameters());
  options= fovis::VisualOdometry::getDefaultOptions();
  odom = new fovis::VisualOdometry(rect, options);
  
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
  
  odom_trans.transform.translation.x = 0.0;
  odom_trans.transform.translation.y = 0.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  
  ros::Rate r(10.0);
  while(n.ok()){
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_broadcaster.sendTransform(odom_trans);
    ros::spinOnce();
    r.sleep();
   }

  return 0;
}



