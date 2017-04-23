// \TODO Add a copyright notice...

#ifndef DAVINCI_INTERFACE_H
#define DAVINCI_INTERFACE_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

/*! \namespace davinci_interface
  \brief The davinci simulation and/or robot interface. Use instead of directly calling topics.
  
This will prevent both dupication and unnecessary work (having fifty packages with fifty slightly different ways of reading joint states), and also make it much easier to change methods of interacting with the robot that are still in a fair degree of flux. Since part of the point of this package is to simplify making changes to the interface, anything that does NOT use it may break at any time.
*/  
namespace davinci_interface
{
  /*! \fn void init_joint_control(ros::NodeHandle & nh)
    \brief Set up publishers to connect to DaVinci's joints.
    
    \param[in]  nh  The nodehandle of the node to enable output for.
    
  Call this before attempting to use publish_joints to initiate lines of communication with either the simulation or physical robot. If this is not done at least once, publish_joints will not be able to publish your commands on any topic, the robot won't do anything, and it will return FALSE.
  */
  void init_joint_control(ros::NodeHandle & nh);

  /*! \fn bool publish_joints(const double input[14])
    \brief Send a joint configuration to the robot.
    
    \param[in]  input An array of joint values, one for each joint.
    
    \returns TRUE if there exist topics on which the joint message was sent (i.e.  joint control has been initialized), FALSE otherwise.
    
  Sends a set of joint commands to the simulated or physical DaVinci robot, based on an input array. The array is formatted in accordance with the DaVinci Joint Configuration. Note that there is NO guarantee that the messages sent were recieved or that the controllers executed them; use get_fresh_robot_pos to check this.
  */
  // May want to expand on this later, and create commanders for individual arms, etc.
  bool publish_joints(const double input[14]);

  /*! \fn void init_joint_feedback(ros::NodeHandle & nh)
    \brief Set up subscribers to listen to DaVinci's joints.
    
    \param[in]  nh  The nodehandle of the node to enable input for.
    
  Call this before attempting to use get_fresh_robot_pos to initiate lines of communication with either the simulation or physical robot. If this is not done at least once, get_fresh_robot_pos will not be able to listen to the robot, you won't get any information, and it will return FALSE.
  */
  void init_joint_feedback(ros::NodeHandle & nh);
  
  /*! \fn bool get_fresh_robot_pos(std::vector<std::vector<double> > & output)
    \brief Recieve an up-to-date joint configuration from the robot
    
    \param[in,out]  output  A two-dimensional vector of doubles that will be filled with the joint states.
    
    \returns TRUE if there exist topics on which to listen to the joint states (i.e. joint listening has been initialized), FALSE otherwise.
    
  Fills a vector with a guaranteed up-to-date joint configuration from the robot. This function will block until a joint configuration has been recieved.
  */

  // Same as with the joint commander, we may want to expand this later on and create ones for individual arms, etc.
  bool get_fresh_robot_pos(std::vector<std::vector<double> > & output);

  /*! \fn bool get_last_robot_pos(std::vector<std::vector<double> > & output)
    \brief Recieve the most recent joint configuration from the robot
    
    \param[in,out]  output  A two-dimensional vector of doubles that will be filled with the joint states.
    
    \returns TRUE if there exist topics on which to listen to the joint states (i.e. joint listening has been initialized) <i>and</i> there is a joint state waiting, FALSE otherwise.
    
  Fills a vector with the most recent joint configuration from the robot.
  */
  // Same as with the joint commander, we may want to expand this later on and create ones for individual arms, etc.
  bool get_last_robot_pos(std::vector<std::vector<double> > & output);

  // \TODO missing the function declaration.
  static bool get_jnt_val_by_name(std::string jnt_name,sensor_msgs::JointState jointState,double &qval);
};  // namespace davinci_interface

#endif   /* DAVINCI_INTERFACE_H */
