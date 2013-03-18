#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


ros::Publisher odom_pub;
double x,y,z,vx,vy,vz;
ros::Time current_time, last_time;
#define THRESHOLD 0.3

geometry_msgs::Vector3 accel_rotate(double q0,double q1,double q2,double q3, double ax,double ay,double az){

    		geometry_msgs::Vector3 ra; // rotated accel
        ra.x= (2*q0*q0 - 1 + 2*q1*q1 ) * ax + 2 * (q1*q2+q0*q3)     * ay + 2* (q1*q3 - q0*q2)     * az;
        ra.y= 2*(q1*q2-q0*q3)     * ax + (2*q0*q0-1+2*q2*q2) * ay + 2*(q2*q3+q0*q1)     * az;
        ra.z= 2*(q1*q3+q0*q2)     * ax + 2*(q2*q3 - q0*q1)     * ay + (2*q0*q0-1+2*q3*q3) * az;
				
if(ra.x < THRESHOLD && ra.x > -THRESHOLD) ra.x=0;
if(ra.y < THRESHOLD && ra.y > -THRESHOLD) ra.y=0;
if(ra.z < THRESHOLD && ra.z > -THRESHOLD) ra.z=0;

				return ra;


//printf("\nRot: %6.4f, %6.4f, %6.4f\t", rax, ray, raz);
//velocity(rax, ray, raz);

}

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%f]", msg->linear_acceleration.x);
	//ROS_INFO("Received quaternion: (%f, %f, %f, %f)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	

geometry_msgs::Vector3 ra = accel_rotate(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z,msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

ra.z = 0;

  static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(ra.x/2, ra.y/2, ra.z/2) );
     transform.setRotation( tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w) );
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "imu"));



  static tf::TransformBroadcaster odom_broadcaster;
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	
	vx += ra.x * dt;
	vy += ra.y * dt;
	vz += ra.z * dt;
	
  double delta_x = vx * dt;
  double delta_y = vy * dt;
  double delta_z = vz * dt;

    x += delta_x;
    y += delta_y;
		z += delta_z;

	//first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = msg->orientation;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = msg->orientation;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vz;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_position");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ardrone/imu", 100, chatterCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
	x = 0.0;
	y = 0.0;
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
 	current_time = ros::Time::now();
  last_time = ros::Time::now();
  ROS_INFO("I am running");
  ros::spin();
  return 0;
}



