/******************************************************************************
**
**   dvrk Hardware interface header for the DVRK.
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
#ifndef CWRU_DAVINCI_CONTROL_CWRU_DAVINCI_HWI_H
#define CWRU_DAVINCI_CONTROL_CWRU_DAVINCI_HWI_H

#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <ros/init.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <cwru_davinci_control/joint_names.h>


/*! @file cwru_davinci_hwi.h
* @brief Da Vinci hardware interface.
* 
* Used for cwru_davinci_control and related applications.
*/

/*! @classDavinciHWI cwru_davinci_hwi.h <cwru_davinci_traj_streamer/cwru_davinci_hwi.h>
* @brief DaVinci specific ROS hardware interface.
* Converts ROS trajectory-style commands (i.e. writes to memory addresses) into ROS topics that the DVRK or simulator can understand and execute.
*/
class DavinciHWI : public hardware_interface::RobotHW
{
public:
  /**
   * @brief The explicit default constructor for the da Vinci hardware interface object.
   *
   * The constructor initializes the controller interfaces.
   *
   * @param n The ros node handle.
   * @param psm The string name of the Patient Side Manipulator.
   */
  explicit DavinciHWI(ros::NodeHandle & n, const std::string & psm);

  /**
   * @brief empty destructor
   */
  ~DavinciHWI()
  {}

  /**
   * @brief convert the joint controller back to position control and 
   *
   *
   * Regardless of last state recieved, (if any), convert the DVRK to position control mode. 
   * Not sure why this is called every loop iteration
   * TODO(tes77,rcj33) refactor this function for a more concise purpose.
   */
  void pcp();

private:
  // This bool has an unclear definition.
  bool wfcb;

  /**
   * @brief These interfaces are used for sending commands through the robot interface.
   */
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;

  /**
   * @brief The cmd, pos, vel, eff are used to send commands to the dvrk
   * 
   * It isn't obvious why cmd is a vector, while the others are all double arrays.
   */
  std::vector<double> cmd = std::vector<double>(7);
  double pos[7];
  double vel[7];
  double eff[7];

  /**
   * @brief The state of the dvrk robot controller
   */
  std::string state;

  /**
   * @brief The joint state and dvrk mode subscription.
   * 
   * The mode acts more like a service than a topic.
   */
  ros::Subscriber state_sub, mode_sub;

  /**
   * @brief The joint and mode publication for the dvrk controller.
   */
  ros::Publisher joint_publisher, mode_publisher;

  /**
   * @brief The joint state message callback.
   *
   * @param incoming The incoming joint State.
   */
  void CB_update(const sensor_msgs::JointState::ConstPtr& incoming);

  /**
   * @brief The string encoded state of the dvrk
   *
   * @param incoming the string of the state.
   * @TODO find a string encoding, decoding of the string states.
   */
  void CB_state(const std_msgs::String::ConstPtr& incoming);
};

#endif  // CWRU_DAVINCI_CONTROL_CWRU_DAVINCI_HWI_H
