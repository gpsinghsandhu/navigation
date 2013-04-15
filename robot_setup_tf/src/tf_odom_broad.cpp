#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


geometry_msgs::TransformStamped odom_trans;
ros::Time current_time;

void odomCallback(const nav_msgs::Odometry& odom){
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom.pose.pose.orientation);

    current_time = ros::Time::now();

    //first, we'll publish the transform over tf
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom.pose.pose.orientation;
}
  

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
  
  odom_trans.transform.translation.x = 0.0;
  odom_trans.transform.translation.y = 0.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  ros::Subscriber sub = n.subscribe("/vo",50,odomCallback);

  ros::Rate r(100.0);
  while(n.ok()){
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_broadcaster.sendTransform(odom_trans);
    r.sleep();
   }
}
