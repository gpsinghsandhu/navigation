#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(15);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.2, 0.0, 0.0)),
        ros::Time::now(),"base_footprint", "base_link"));
    r.sleep();
  }
}
