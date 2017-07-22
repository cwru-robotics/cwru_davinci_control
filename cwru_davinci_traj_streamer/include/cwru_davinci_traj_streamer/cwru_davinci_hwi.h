#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <ros/init.h>

#include <iostream>

#ifndef DAVINCI_HWI_INCLUDE
#define DAVINCI_HWI_INCLUDE
class DavinciHWI : public hardware_interface::RobotHW{

  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;

    std::vector<double> cmd = std::vector<double>(7);
    double pos[7];
    double vel[7];
    double eff[7];
    
    ros::Subscriber state_sub;
    ros::Publisher joint_publisher;
    
    void CB_update(const sensor_msgs::JointState::ConstPtr& incoming);
  
  public:
    DavinciHWI(ros::NodeHandle & n, const std::string & psm);
    virtual ~DavinciHWI(){}
    
    void pcp();
  
    void write(std::string psm){
      std::cout << psm << " command " << "  " << cmd[0] << "  " << cmd[1] << "  " << cmd[2] << "  " << cmd[3] << "  " << cmd[4] << "  " << cmd[5] << "  " << cmd[6] << std::endl;
    }
    void read(std::string psm){
      std::cout << psm << " j_state " << "  " << pos[0] << "  " << pos[1] << "  " << pos[2] << "  " << pos[3] << "  " << pos[4] << "  " << pos[5] << "  " << pos[6] << std::endl;
    }
    
    //static void getFreshRobotPos(
};
#endif
