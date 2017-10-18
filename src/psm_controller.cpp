/******************************************************************************
**
**   psm controller library for the dvrk.
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

#include <vector>
#include <string>

#include <cwru_davinci_control/psm_controller.h>


psm_controller::psm_controller(int psm, ros::NodeHandle & nh, bool read_only):
  action_client(
    nh,
    "/dvrk/PSM" + std::to_string(psm) + "/joint_traj_controller/follow_joint_trajectory",
    true),
    read_only_(read_only)
{
  if (!read_only_)
  {
    action_client.waitForServer();
  }
  ROS_INFO("PSM%d is now ready", psm);
  psm_number = psm;
  fresh_joint_state = false;
  first_joint_state = false;
  joint_state_subscriber = nh.subscribe(
    "/dvrk/PSM" + std::to_string(psm) + "/state_joint_current",
    1,
    &psm_controller::pos_cb,
    this);
}

bool psm_controller::get_psm_state(sensor_msgs::JointState & js)
{
  bool fjs_prev = fresh_joint_state;
  if (first_joint_state)
  {
    js = joint_state;
    fresh_joint_state = false;
  }
  return fjs_prev;
}

bool psm_controller::get_fresh_psm_state(sensor_msgs::JointState & js, double t)
{
  if (t > 0.0)
  {
    fresh_joint_state = false;
    ros::Time goal_time = ros::Time::now() + ros::Duration(t);
    while (!fresh_joint_state)
    {
      ros::spinOnce();
      if (ros::Time::now() > goal_time)
      {
        return false;
      }
    }
    fresh_joint_state = false;
    js = joint_state;
    return true;
  }
  else
  {
    fresh_joint_state = false;
    while (!fresh_joint_state)
    {
      ros::spinOnce();
    }
    fresh_joint_state = false;
    js = joint_state;
    return true;
  }
}

void psm_controller::move_psm(trajectory_msgs::JointTrajectory & t)
{
  if (read_only_) return;
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = t;
  action_client.sendGoal(goal);
}

void psm_controller::move_psm(std::vector<trajectory_msgs::JointTrajectoryPoint> & goals)
{
  if (read_only_) return;
  trajectory_msgs::JointTrajectory total;
  total.header = std_msgs::Header();
  total.joint_names = dvrk_control::joint_names;
  total.points = goals;
  move_psm(total);
}

void psm_controller::move_psm(trajectory_msgs::JointTrajectoryPoint & goal)
{
  if (read_only_) return;
  trajectory_msgs::JointTrajectory total;
  total.joint_names = dvrk_control::joint_names;
  total.points.push_back(goal);
  move_psm(total);
}

void psm_controller::move_psm(std::vector<sensor_msgs::JointState> & goals, std::vector<double> & times)
{
  if (read_only_) return;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jtps(goals.size());
  for (int i = 0; i < goals.size(); i++)
  {
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions = goals[i].position;
    p.velocities = goals[i].velocity;
    p.time_from_start = ros::Duration(times[i]);
    jtps.push_back(p);
  }
  move_psm(jtps);
}

void psm_controller::move_psm(sensor_msgs::JointState & goal, double time)
{
  if (read_only_) return;
  trajectory_msgs::JointTrajectoryPoint p;
  p.positions = goal.position;
  p.velocities = goal.velocity;
  p.time_from_start = ros::Duration(time);
  move_psm(p);
}

void psm_controller::move_psm(int joint, double location, double time)
{
  if (read_only_) return;
  sensor_msgs::JointState js;
  get_psm_state(js);
  js.position[joint] = location;
  move_psm(js, time);
}

void psm_controller::move_psm(
  const std::vector<int> & joints,
  const std::vector<double> & locations,
  const std::vector<double> & velocities,
  double time
)
{
  if (read_only_) return;
  sensor_msgs::JointState js;
  get_psm_state(js);
  for (int i = 0; i < joints.size(); i++)
  {
    js.position[joints[i]] = locations[i];
    js.velocity[joints[i]] = velocities[i];
  }
  move_psm(js, time);
}

bool psm_controller::wait_psm(double wait_time)
{
  if (read_only_) return false;
  return action_client.waitForResult(ros::Duration(wait_time));
}
