#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cwru_davinci_interface/davinci_interface.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cwru_davinci_traj_streamer/trajAction.h>

using namespace davinci_interface;

/** \file traj_interpolator_as.cpp
*@brief Trajectory streamer
*
*Run in bkg, provides trajectory interpolation for the DaVinci unit
*
*Recommended to use with playfile_reader_jointspace, playfile_reader_cameraspace, or playfile_reader_cartspace.
*/
const double DT_TRAJ = 0.01;

//Persistant things relating to the publishers and subscribers.
actionlib::SimpleActionServer<cwru_davinci_traj_streamer::trajAction> * g_server;
//The action server has a pointer stored globally because it's constructed in Main
//as everything is initialized, but the goal callback uses it and has no arguments.

//CALLBACKS
void CB_goal();

//UTILITY FUNCS
void tl_execute(
	const cwru_davinci_traj_streamer::trajGoalConstPtr& goal,
	actionlib::SimpleActionServer<cwru_davinci_traj_streamer::trajAction> * server
);
bool exec_position(
	const trajectory_msgs::JointTrajectoryPoint & in_point,
	const std::vector<double> & current_jnts,
	double & elapsed_time
);
std::vector<std::vector<double> > get_robot_pos();//Not currently used, kept for archival reasons until moved to another pkg.

static ros::NodeHandle * n = NULL;

/** @brief main funcion
*/
int main(int argc, char **argv) {
	//Set up the node.
	ros::init(argc, argv, "traj_interpolator_both_as");
	ros::NodeHandle nh;
	
	init_joint_feedback(nh);
	n = &nh;
	ROS_INFO("NODE LINKS LATCHED");
	
	//Begin offering the service
	actionlib::SimpleActionServer<cwru_davinci_traj_streamer::trajAction> server(nh, "trajActionServer", false);
	g_server = & server;//Update that pointer for the goal callback.
	server.registerGoalCallback(&CB_goal);
	server.start();
	
	//Go into spin.
	while (ros::ok()){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}	
	
	return 0;
}
//
//Accept goals manually to avoid the "Total Recall" bug
void CB_goal(){
	
	//I guess you could just run the whole trajectory in
	//here, but I've kept the execute function.
	tl_execute(g_server->acceptNewGoal(), g_server);
	
	//Could add logic that checks if already busy, etc.
	
	//g_server->isPreemptRequested();
	//g_server->isActive();
	//g_server->isNewGoalAvailable();
}

//The bulk of the trajectory execution is still done in what used to be the exec callback.
void tl_execute(
	const cwru_davinci_traj_streamer::trajGoalConstPtr& goal,
	actionlib::SimpleActionServer<cwru_davinci_traj_streamer::trajAction>* server
){
	ROS_INFO("Got a trajectory request with id %u, %lu positions.", goal->traj_id, goal->trajectory.points.size());
	ROS_INFO("Latching node links.");
	init_joint_control(*n);
	ROS_INFO("NODE LINKS LATCHED.");
	//Start the timer.
	double elapsed_time = 0.0;
	//Kick us out early if the trajectory does not have at least a start point and an end point.
	if(goal->trajectory.points.size() < 1){
		ROS_ERROR("Trajectory %u is too small! Aborting.", goal->traj_id);
		cwru_davinci_traj_streamer::trajResult r;
		r.return_val = 0;
		r.traj_id = goal->traj_id;
		server->setAborted();
		return;
	}
	
	//Record our initial starting position
	ROS_INFO("Recording initial position.");
	std::vector<std::vector<double> > prev_point_tmp;
	get_fresh_robot_pos(prev_point_tmp);
	std::vector<double> prev_point(14);
	for(int i = 0; i < 7; i++){
		prev_point[i] = prev_point_tmp[0][i];
		prev_point[i + 7] = prev_point_tmp[1][i];
	}
	
	ROS_INFO("Moving...");
	//Move to each point in sequence and update the timer,
	for(int i = 0; i < goal->trajectory.points.size(); i++){
		if(!exec_position(goal->trajectory.points[i], prev_point, elapsed_time)){
			//Abort if we discover that one of the points we are going to is malformed.
			cwru_davinci_traj_streamer::trajResult r;
			r.return_val = 0;
			r.traj_id = goal->traj_id;
			server->setAborted();
			return;
		}
		prev_point = goal->trajectory.points[i].positions;
	}
	//Tell the requester we did a good job.
	ROS_INFO("Trajectory %u complete.", goal->traj_id);
	cwru_davinci_traj_streamer::trajResult r;
	r.return_val = 1;
	r.traj_id = goal->traj_id;
	server->setSucceeded(r);
	
	return;
}

//Move the robot to each individual markpoint.
bool exec_position(
	const trajectory_msgs::JointTrajectoryPoint & in_point,
	const std::vector<double> & current_jnts,
	double & elapsed_time
){
	//Give us at least one tick in which to execute the trajectory
	double input_gtime = std::max(in_point.time_from_start.toSec(), elapsed_time + DT_TRAJ);
	
	std::vector<double> input_jnts = in_point.positions;
	//std::vector<double> current_jnts = prev_point.positions;
	
	//Run some checks...
	if(input_jnts.size() != 14){
		ROS_ERROR("DaVinci trajectory points should have 14 joints, and this one has %lu!", input_jnts.size());
		return false;
	}
	if(current_jnts.size() != 14){
		ROS_ERROR("DaVinci trajectory points should have 14 joints, and the last one has %lu!", current_jnts.size());
		return false;
	}
	
	ROS_INFO("	Go to position %f, %f, %f, %f, %f, %f, %f and %f, %f, %f, %f, %f, %f, %f by %f at %f",
		input_jnts[0],
		input_jnts[1],
		input_jnts[2],
		input_jnts[3],
		input_jnts[4],
		input_jnts[5],
		input_jnts[6],
		input_jnts[7],
		input_jnts[8],
		input_jnts[9],
		input_jnts[10],
		input_jnts[11],
		input_jnts[12],
		input_jnts[13],
		input_gtime,
		elapsed_time
	);
	
	double old_elapsed_time = elapsed_time;
	while(elapsed_time < input_gtime){//Each tick...
		ros::Duration wait_time(DT_TRAJ);
		elapsed_time = elapsed_time + DT_TRAJ;
		
		double unexpanded_joints[14];
		
		double parametric_value = (elapsed_time - old_elapsed_time) / (input_gtime - old_elapsed_time);//Normalize our time.
		for(int i = 0; i < 14; i++){
			//Get new positions by treating our path as a parametric equation of time.
			unexpanded_joints[i] = current_jnts[i] + (input_jnts[i] - current_jnts[i]) * parametric_value;
		}
		
		publish_joints(unexpanded_joints);
		
		ros::spinOnce();
		wait_time.sleep();
	}
		
	return true;
}
