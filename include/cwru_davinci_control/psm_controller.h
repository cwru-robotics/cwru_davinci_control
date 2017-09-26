#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <cwru_davinci_control/joint_names.h>

#ifndef PSM_CONTROLLER_H
#define PSM_CONTROLLER_H
class psm_controller{
	private:
		int psm_number;
		
		bool fresh_joint_state;
		bool first_joint_state;
		sensor_msgs::JointState joint_state;
		ros::Subscriber joint_state_subscriber;
		void pos_cb(const sensor_msgs::JointState::ConstPtr& incoming){
			joint_state = * incoming;
			fresh_joint_state = true;
			first_joint_state = true;
		}
		
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client;
	public:
		/**
 * @brief Create a controller for a given PSM
 *
 * By convention PSM numbers begin at 1 and increase from left to right. The default Da Vinci configuration involves two PSMs.
 */
		psm_controller(int psm, ros::NodeHandle & nh);
		
		/**
 * @brief Populate most recent robot position
 *
 * Returns true if the state is "fresh" (not been checked yet since the most recent update), false otherwise
 */
		bool get_psm_state(sensor_msgs::JointState & js);
		/**
 * @brief Wait a given amount of time for a position update, and populate js with it.
 *
 * Returns true if updated, returns false and does not change js if timed out. Omit time update to wait forever, in which case it can never return false.
 */
		bool get_fresh_psm_state(sensor_msgs::JointState & js, double t = -1.0);
		
 /* @brief Move all 7 joints to location by time
 *
 * Joints are numbered from base to tip.
 */
		void move_psm(double (& location) [7], double time);
 /* @brief Pass through location with velocity by time
 *
 * Joints are numbered from base to tip.
 */
		void move_psm(double (& location) [7], double (& velocity) [7], double time);
		 /* @brief Move all one specific joint to location by time
 *
 * Joints are numbered from base to tip.
 */
		void move_psm(int joint, double location, double time);
		
 /* @brief Move all some number of specified joints (between none and all 7) to location by time
 *
 * Joints are numbered from base to tip.
 */
		void move_psm(
			std::vector<int> & joints,
			std::vector<double> & locations,
			double time
		);
		
		void move_psm(sensor_msgs::JointState & goal, double time);
		void move_psm(std::vector<sensor_msgs::JointState> & goals, std::vector<double> & times);
		void move_psm(trajectory_msgs::JointTrajectoryPoint & goal);
		void move_psm(std::vector<trajectory_msgs::JointTrajectoryPoint> & goals);	
		void move_psm(trajectory_msgs::JointTrajectory & goal);
		void move_psm(std::vector<trajectory_msgs::JointTrajectory> & goals);
		
		int psm(){return psm_number;}
};
#endif
