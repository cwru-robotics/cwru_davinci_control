#include <cwru_davinci_control/psm_controller.h>

using namespace dvrk_control;

psm_controller::psm_controller(int psm, ros::NodeHandle & nh) :
	action_client(
		nh,
		"/dvrk/PSM" + std::to_string(psm) + "/joint_traj_controller/follow_joint_trajectory",
		true
	)
{
	psm_number = psm;
	fresh_joint_state = false;
	first_joint_state = false;
	joint_state_subscriber = nh.subscribe(
		"/dvrk/PSM" + std::to_string(psm) + "/state_joint_current",
		1, 
		&psm_controller::pos_cb,
		this
	);
}

bool psm_controller::get_psm_state(sensor_msgs::JointState & js){
	bool fjs_prev = fresh_joint_state;
	if(first_joint_state){
		js = joint_state;
		fresh_joint_state = false;
	}
	return fjs_prev;
}

bool psm_controller::get_fresh_psm_state(sensor_msgs::JointState & js, double t){
	if(t > 0.0){
		fresh_joint_state = false;
		ros::Time goal_time = ros::Time::now() + ros::Duration(t);
		while(!fresh_joint_state){
			ros::spinOnce();
			if(ros::Time::now() > goal_time){
				return false;
			}
		}
		fresh_joint_state = false;
		js = joint_state;
		return true;
	} else{
		fresh_joint_state = false;
		while(!fresh_joint_state){
			ros::spinOnce();
		}
		fresh_joint_state = false;
		js = joint_state;
		return true;
	}
}

void psm_controller::move_psm(trajectory_msgs::JointTrajectory & t){
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = t;
	action_client.sendGoal(goal);
}
	
void psm_controller::move_psm(std::vector<trajectory_msgs::JointTrajectory> & ts){
	trajectory_msgs::JointTrajectory total;
	total.joint_names = joint_names;
	
	ros::Duration total_time(0.0);
	for(int i = 0; i < ts.size(); i++){
		for(int j = 0; j < ts[i].points.size(); j++){
			total.points.push_back(ts[i].points[j]);
			total.points[total.points.size() - 1].time_from_start = ts[i].points[j].time_from_start + total_time;
		}
		total_time = total_time + ts[i].points[ts[i].points.size() - 1].time_from_start;
	}
			
	move_psm(total);	
}

void psm_controller::move_psm(std::vector<trajectory_msgs::JointTrajectoryPoint> & goals){
	trajectory_msgs::JointTrajectory total;
	total.joint_names = joint_names;
	ros::Duration total_time(0.0);
	for(int i = 0; i < goals.size(); i ++){
		total_time = total_time + goals[i].time_from_start;
		total.points.push_back(goals[i]);
		total.points[total.points.size() - 1].time_from_start = total_time;
	}
	
	move_psm(total);
}

void psm_controller::move_psm(trajectory_msgs::JointTrajectoryPoint & goal){
	trajectory_msgs::JointTrajectory total;
	total.joint_names = joint_names;
	total.points.push_back(goal);
	move_psm(total);
}

void psm_controller::move_psm(std::vector<sensor_msgs::JointState> & goals, std::vector<double> & times){
	std::vector<trajectory_msgs::JointTrajectoryPoint> jtps(goals.size());
	for(int i = 0; i < goals.size(); i++){
		trajectory_msgs::JointTrajectoryPoint p;
		//WHY did you think this was a good idea, anonymous-ROS-programmer-who-made-joint-states-and-traj-points-almost-identically-named-but-not-actually-compatible?
		p.positions = goals[i].position;
		p.velocities = goals[i].velocity;
		//We don't care about the efforts.
		p.time_from_start = ros::Duration(times[i]);
		jtps.push_back(p);
	}
	move_psm(jtps);
}

void psm_controller::move_psm(sensor_msgs::JointState & goal, double time){
	trajectory_msgs::JointTrajectoryPoint p;
	//WHY did you think this was a good idea, anonymous-ROS-programmer-who-made-joint-states-and-traj-points-almost-identically-named-but-not-actually-compatible?
	p.positions = goal.position;
	p.velocities = goal.velocity;
	//We don't care about the efforts.
	p.time_from_start = ros::Duration(time);
	
	move_psm(p);
}

void psm_controller::move_psm(double (& location) [7], double time){
	double v[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	move_psm(location, v, time);
}

void psm_controller::move_psm(double (& location) [7], double (& velocity) [7], double time){
	trajectory_msgs::JointTrajectoryPoint p;
	std::vector<double> v1(location, location + 7);
	std::vector<double> v2(velocity, velocity + 7);
	
	p.positions = v1;
	p.velocities = v2;
	p.time_from_start = ros::Duration(time);
	
	move_psm(p);
}

void psm_controller::move_psm(int joint, double location, double time){
	sensor_msgs::JointState js;
	get_psm_state(js);
	js.position[joint] = location;
	move_psm(js, time);
}

void psm_controller::move_psm(
	std::vector<int> & joints,
	std::vector<double> & locations,
	double time
){
	sensor_msgs::JointState js;
	get_psm_state(js);
	for(int i = 0; i < joints.size(); i++){
		js.position[joints[i]] = locations[i];
	}
	move_psm(js, time);
}
