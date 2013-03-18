#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
ros::Publisher marker_pub; 
uint32_t shape = visualization_msgs::Marker::ARROW;

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%f]", msg->linear_acceleration.x);
	//ROS_INFO("Received quaternion: (%f, %f, %f, %f)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, 0.0) );
     transform.setRotation( tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w) );
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));

visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = msg->linear_acceleration.x;
    marker.pose.position.y = msg->linear_acceleration.y;
    marker.pose.position.z = msg->linear_acceleration.z;
    marker.pose.orientation.x = msg->orientation.x;
    marker.pose.orientation.y = msg->orientation.y;
    marker.pose.orientation.z = msg->orientation.z;
    marker.pose.orientation.w = msg->orientation.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);

    // Cycle between different shapes
/*
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
*/

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_position");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ardrone/imu", 100, chatterCallback);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::spin();
  return 0;
}
