#include <dynamic_reconfigure/server.h>
#include <imu_position/IMUpositionConfig.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
ros::Publisher imu_reset_speed_pub,imu_reset_position_pub, imu_threashold_max_pub,imu_threashold_min_pub;

void callback(imu_position::IMUpositionConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: aMin:%f\taMax:%f\t rSpeed:%d\t rDist:%d",
            //config.int_param,
						config.aMin, 
						config.aMax, 
	          //config.str_param.c_str(), 
            config.rSpeed,
						config.rDist
						//config.bool_param?"True":"False",
            //config.size
					);
	std_msgs::Bool b;

	b.data = config.rSpeed;
	imu_reset_speed_pub.publish(b);

	b.data = config.rDist;
  imu_reset_position_pub.publish(b);

	std_msgs::Float64 t;

	t.data =  config.aMax;
	imu_threashold_max_pub.publish(t);

	t.data =  config.aMin;
	imu_threashold_min_pub.publish(t);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure");


  ros::NodeHandle n;
	imu_reset_speed_pub = n.advertise<std_msgs::Bool>("imu/resetSpeed", 1);
	imu_reset_position_pub = n.advertise<std_msgs::Bool>("imu/resetPosition", 1);

	imu_threashold_max_pub = n.advertise<std_msgs::Float64>("imu/accelThresholdMax", 1);
	imu_threashold_min_pub = n.advertise<std_msgs::Float64>("imu/accelThresholdMin", 1);

  dynamic_reconfigure::Server<imu_position::IMUpositionConfig> srv;
  dynamic_reconfigure::Server<imu_position::IMUpositionConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);




  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}

