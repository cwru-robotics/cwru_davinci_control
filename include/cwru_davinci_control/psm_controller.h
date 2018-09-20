/******************************************************************************
**
**   psm controller header for the dvrk.
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
#ifndef CWRU_DAVINCI_CONTROL_PSM_CONTROLLER_H
#define CWRU_DAVINCI_CONTROL_PSM_CONTROLLER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cwru_davinci_control/joint_names.h>


class psm_controller
{
public:
  /**
   * @brief constructor to create a controller for a given PSM
   *
   * By convention PSM numbers begin at 1 and increase from left to right. The default Da Vinci configuration involves two PSMs.
   */
  explicit psm_controller(int psm, ros::NodeHandle & nh, bool read_only = false);

  /**
   * @brief Populate most recent robot position
   *
   * Returns true if the state is "fresh" (not been checked yet since the most recent update), false otherwise
   */
  bool get_psm_state(sensor_msgs::JointState & js);

/**
   * @brief Report our PSM number
   *
   */
  int get_psm(){ return psm_number;}

  /**
   * @brief Wait a given amount of time for a position update, and populate js with it.
   *
   * Returns true if updated, returns false and does not change js if timed out. Omit time update to wait forever, in which case it can never return false.
   */
  bool get_fresh_psm_state(sensor_msgs::JointState & js, double t = -1.0);

  /**
   * @brief wait for a PSM to finish its current motion.
   *
   * @param wait_time How long to wait for the motion to complete (default 30s)
   */
  bool wait_psm(double wait_time = 30.0);

  /**
   * @brief the index of the PSM.
   */
  int psm() const
  {
    return psm_number;
  }

  // TODO(rcj33,tes77) There are way too many move_psm commands. They need to be pruned.

  /* @brief one specific joint to a location by time
   *
   * Joints are numbered from base to tip.
   */
  void move_psm(int joint, double location, double time);

  /* @brief Move all some number of specified joints (between none and all 7) to location by time
   *
   * Joints are numbered from base to tip.
   */
  void move_psm(
    const std::vector<int> &joints,
    const std::vector<double> &locations,
    const std::vector<double> &velocities = std::vector<double>(),
    double time = 5.0);

  void move_psm(sensor_msgs::JointState & goal, double time);
  void move_psm(std::vector<sensor_msgs::JointState> & goals, std::vector<double> & times);
  void move_psm(trajectory_msgs::JointTrajectoryPoint & goal);
  void move_psm(std::vector<trajectory_msgs::JointTrajectoryPoint> & goals);
  void move_psm(trajectory_msgs::JointTrajectory & goal);
  void move_psm(std::vector<trajectory_msgs::JointTrajectory> & goals);

private:
  // The index of the PSM
  int psm_number;

  // Is there a new joint state available?
  bool fresh_joint_state;

  // has any joint state been recieved?
  bool first_joint_state;

  // The sensor populated joing states
  sensor_msgs::JointState joint_state;

  // The subscriber for the joint state
  ros::Subscriber joint_state_subscriber;

  /**
   * @brief The jointState callback
   *
   * @param incoming The incoming joint state information
   */
  void pos_cb(const sensor_msgs::JointState::ConstPtr& incoming)
  {
    joint_state = * incoming;
    fresh_joint_state = true;
    first_joint_state = true;
  }

  // The action client for the JointTrajectory controller.
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client;

  bool read_only_;
};

#endif  // CWRU_DAVINCI_CONTROL_PSM_CONTROLLER_H
