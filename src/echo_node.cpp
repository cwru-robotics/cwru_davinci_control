/******************************************************************************
**
**   Hardware interface program for the dvrk.
**   Copyright (C) 2017  Tom Shkurti, Russell Jackson, & Wyatt Newman
**   Case Western Reserve University
**
**   This program is free software: you can redistribute it and/or modify
**   it under the terms of the GNU General Public License as published by
**   the Free Software Foundation, either version 3 of the License, or
**   (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**   but WITHOUT ANY WARRANTY; without even the implied warranty of
**   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**   GNU General Public License for more details.
**
**   You should have received a copy of the GNU General Public License
**   along with this program.  If not, see <http://www.gnu.org/licenses/>.
**
******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <cwru_davinci_control/joint_names.h>

ros::Publisher PSM1_input_pub;
ros::Publisher PSM2_input_pub;
ros::Publisher PSM1_output_pub;
ros::Publisher PSM2_output_pub;

void CB_1_in(const sensor_msgs::JointState & old_state){
	sensor_msgs::JointState new_state;
	new_state.position = old_state.position;
	new_state.velocity = old_state.position;
	new_state.effort = old_state.effort;
	new_state.name = dvrk_control::joint_names;
	
	PSM1_input_pub.publish(new_state);
}
void CB_2_in(const sensor_msgs::JointState & old_state){
	sensor_msgs::JointState new_state;
	new_state.position = old_state.position;
	new_state.velocity = old_state.position;
	new_state.effort = old_state.effort;
	new_state.name = dvrk_control::joint_names;
	
	PSM2_input_pub.publish(new_state);
}
void CB_1_out(const sensor_msgs::JointState & old_state){
	sensor_msgs::JointState new_state;
	new_state.position = old_state.position;
	new_state.velocity = old_state.position;
	new_state.effort = old_state.effort;
	
	std::vector<std::string> new_names = std::vector<std::string>(7);
	for(int i = 0; i < 7; i++){
		new_names[i] = "PSM1/" + dvrk_control::joint_names[i];
	}
	new_state.name = new_names;
	
	PSM1_output_pub.publish(new_state);
}
void CB_2_out(const sensor_msgs::JointState & old_state){
	sensor_msgs::JointState new_state;
	new_state.position = old_state.position;
	new_state.velocity = old_state.position;
	new_state.effort = old_state.effort;
	
	std::vector<std::string> new_names = std::vector<std::string>(7);
	for(int i = 0; i < 7; i++){
		new_names[i] = "PSM2/" + dvrk_control::joint_names[i];
	}
	new_state.name = new_names;
	
	PSM2_output_pub.publish(new_state);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "repeater");
  ros::NodeHandle nh;
  
  std_msgs::String unfreeze;
  unfreeze.data = "DVRK_POSITION_JOINT";
  
  ros::Publisher unfreeze_1 = nh.advertise<std_msgs::String>("/dvrk/PSM1/set_robot_state", 1);
  unfreeze_1.publish(unfreeze);
  ros::Publisher unfreeze_2 = nh.advertise<std_msgs::String>("/dvrk/PSM2/set_robot_state", 1);
  unfreeze_2.publish(unfreeze);
  
  PSM1_output_pub = nh.advertise<sensor_msgs::JointState>("/prefixed/dvrk/PSM1/joint_states", 1);
  PSM2_output_pub = nh.advertise<sensor_msgs::JointState>("/prefixed/dvrk/PSM2/joint_states", 1);
  
  PSM1_input_pub = nh.advertise<sensor_msgs::JointState>("/dvrk/PSM1/set_position_joint", 1);
  PSM2_input_pub = nh.advertise<sensor_msgs::JointState>("/dvrk/PSM2/set_position_joint", 1);
  
  ros::Subscriber PSM1_input_sub = nh.subscribe("/dvrk/PSM1/joint_states", 1, CB_1_out);
  ros::Subscriber PSM2_input_sub = nh.subscribe("/dvrk/PSM2/joint_states", 1, CB_2_out);
  
  ros::Subscriber PSM1_output_sub = nh.subscribe("/prefixed/dvrk/PSM1/set_position_joint", 1, CB_1_in);
  ros::Subscriber PSM2_output_sub = nh.subscribe("/prefixed/dvrk/PSM2/set_position_joint", 1, CB_2_in);
  
  ros::spin();

  return 0;
}
