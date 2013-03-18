imu_position
============

install:
--------
sudo apt-get install git-core  
git clone https://github.com/eemmyy23/imu_position.git  
rosdep install imu_position  
rosmake imu_position
usage:
------
rosrun imu_position imu_position_node  
rosrun imu_position imu_position_node imu/data:=/your_imu_topic/data
demo:
-----
