#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>

ros::Publisher odom_pub, imu_position, imu_filtered_accel;
double x=0,y=0,z=0,vx=0,vy=0,vz=0;
ros::Time current_time, last_time, initialize_time,calibrate_time;
#include <stdlib.h>

double xAtemp=0,yAtemp=0,zAtemp=0,xA=0,yA=0,zA=0;
int samples=-100;
int maxSamples = 200;
bool rSpeed = false, rDist=false;
double accelMax = 50;
double accelMin = 0;

double limit (double a, double min, double max){
	if(a <= min && a >= -min) return 0;
	if(a >= max ) return max;
	if(a <= -max ) return max;
	return a;
}

geometry_msgs::Vector3 accel_threshold(geometry_msgs::Vector3 a, double min, double max){
	a.x = limit (a.x, min, max);
	a.y = limit (a.y, min, max);	
	a.z = limit (a.z, min, max);
	return a;
}

geometry_msgs::Vector3 accel_rotate(double q0,double q1,double q2,double q3, double ax,double ay,double az){

    	geometry_msgs::Vector3 ra; // rotated accel
      ra.x= (2*q0*q0 - 1 + 2*q1*q1 ) * ax + 2 * (q1*q2+q0*q3)     * ay + 2* (q1*q3 - q0*q2)     * az;
      ra.y= 2*(q1*q2-q0*q3)     * ax + (2*q0*q0-1+2*q2*q2) * ay + 2*(q2*q3+q0*q1)     * az;
      ra.z= 2*(q1*q3+q0*q2)     * ax + 2*(q2*q3 - q0*q1)     * ay + (2*q0*q0-1+2*q3*q3) * az;
			return ra;
}
/*
if(ra.x < THRESHOLD && ra.x > -THRESHOLD) ra.x=0;
if(ra.y < THRESHOLD && ra.y > -THRESHOLD) ra.y=0;
if(ra.z < THRESHOLD && ra.z > -THRESHOLD) ra.z=0;
*/
/*
geometry_msgs::Vector3 accel_rotate(double q0,double q1,double q2,double q3, double ax,double ay,double az){

    	geometry_msgs::Vector3 ra; // rotated accel

      ra.x= (2*q0*q0 + 2*q1*q1-1)	* ax 		+ 	(2*q1*q2 - 2*q0*q3)		* ay		+		(2*q1*q3 + 2*q0*q2) 	* az;

      ra.y= (2*q1*q2 + 2*q0*q3)		* ax 		+ 	(2*q0*q0 + 2*q2*q2-1)	* ay 		+ 	(2*q2*q3 - 2*q0*q1) 	* az;

      ra.z= (2*q1*q3 - 2*q0*q2)   * ax 		+ 	(2*q2*q3 + 2*q0*q1)   * ay 		+ 	(2*q0*q0 + 2*q3*q3-1) * az;
			
			return ra;
}



geometry_msgs::Vector3 accel_rotate(double q3,double q0,double q1,double q2, double ax,double ay,double az){

    	geometry_msgs::Vector3 ra; // rotated accel

      ra.x= (2*q0*q0 + 2*q1*q1-1)	* ax 		+ 	(2*q1*q2 - 2*q0*q3)		* ay		+		(2*q1*q3 + 2*q0*q2) 	* az;

      ra.y= (2*q1*q2 + 2*q0*q3)		* ax 		+ 	(2*q0*q0 + 2*q2*q2-1)	* ay 		+ 	(2*q2*q3 - 2*q0*q1) 	* az;

      ra.z= (2*q1*q3 - 2*q0*q2)   * ax 		+ 	(2*q2*q3 + 2*q0*q1)   * ay 		+ 	(2*q0*q0 + 2*q3*q3-1) * az;
			
			return ra;
}
*/
void resetSpeedCallback(const std_msgs::Bool::ConstPtr& msg){
	rSpeed = msg->data;
}

void resetPositionCallback(const std_msgs::Bool::ConstPtr& msg){
	rDist = msg->data;
}


void thresholdMaxCallback(const std_msgs::Float64::ConstPtr& msg){
	accelMax = msg->data;
}

void thresholdMinCallback(const std_msgs::Float64::ConstPtr& msg){
	accelMin = msg->data;
}

void calibrate (geometry_msgs::Vector3 a){
	xAtemp+=a.x;
	yAtemp+=a.y;
	zAtemp+=a.z;
	samples++;
	xA=xAtemp/samples;
	yA=yAtemp/samples;
	zA=zAtemp/samples;

	//ROS_INFO("mean values\tx:%f\t y:%f\t z:%f\n",xA,yA,zA);
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
//ROS_INFO("I heard: [%f]", msg->linear_acceleration.x);
//ROS_INFO("Received quaternion: (%f, %f, %f, %f)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	

//geometry_msgs::Vector3 ra = 	accel_rotate(msg->orientation.w, msg->orientation.x, msg->orientation.y,msg->orientation.z,msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
/*
  static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(ra.x/2, ra.y/2, ra.z/2) );
     transform.setRotation( tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w) );
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "imu"));
*/
//ra.z = 0;
geometry_msgs::Vector3 ra  = msg->linear_acceleration;
     //static tf::TransformBroadcaster br;
     //tf::Transform transform;
     //transform.setOrigin( tf::Vector3(ra.x, ra.y, ra.z) );
     //transform.setRotation( tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w) );
     //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "imu"));



 // static tf::TransformBroadcaster odom_broadcaster;
	current_time = msg->header.stamp;
	double dt = (current_time - last_time).toSec();

	
	if (samples<0) {
 		rSpeed = true; rDist=true;
		samples++;
	} else if( samples>=0 && samples<maxSamples ) {
		calibrate(ra);
		rSpeed = false; rDist=false;
		ra.x = ra.y  = ra.z = 0;
	} else {
		ra.x-=xA;
		ra.y-=yA;
		ra.z-=zA;

//ROS_INFO("MEAN %f %f %f", xA,yA,zA);

	}


		ra = accel_threshold(ra, accelMin, accelMax);

		vx += ra.x * dt;
		vy += ra.y * dt;
		vz += ra.z * dt;
	
	if(rSpeed){
		vx = vy = vz  = 0;
	}

	double delta_x = vx * dt;
	double delta_y = vy * dt;
	double delta_z = vz * dt;


		x += delta_x;
		y += delta_y;
		z += delta_z;
		if(rDist){
		x = y = z = 0;
	} 
	//first, we'll publish the transform over tf
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "/base_link_imu";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = msg->orientation;

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "/odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = msg->orientation;

    //set the velocity
    odom.child_frame_id = "/base_link_imu";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vz;

    //publish the message
    //odom_pub.publish(odom);

    imu_position.publish(odom.pose.pose.position);
 		imu_filtered_accel.publish(ra);

    last_time = current_time;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_position_node");
  
  ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("imu/data", 1, imuCallback);
  ros::Subscriber restSpeed = n.subscribe("imu/resetSpeed", 1, resetSpeedCallback);
  ros::Subscriber restPosition = n.subscribe("imu/resetPosition", 1, resetPositionCallback);

  ros::Subscriber thresholdMax = n.subscribe("imu/accelThresholdMax", 1, thresholdMaxCallback);
  ros::Subscriber thresholdMin = n.subscribe("imu/accelThresholdMin", 1, thresholdMinCallback);


	//odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  imu_position = n.advertise<geometry_msgs::Vector3>("imu/position",1);
	imu_filtered_accel = n.advertise<geometry_msgs::Vector3>("imu/filtered_accel",1);

 	current_time = ros::Time::now();
	last_time = current_time;
	//initialize_time = current_time + ros::Duration(1);	
	//calibrate_time = current_time + ros::Duration(7);	

  ros::spin();
  return 0;
}



