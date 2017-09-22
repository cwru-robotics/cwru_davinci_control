#include "cwru_davinci_traj_streamer/cwru_davinci_hwi.h"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"

#include <ros/callback_queue.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "interface_echo_node");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  
  //ROS_INFO("Got argument %s", argv[1]);

  DavinciHWI robot(nh, std::string(argv[1]));
  controller_manager::ControllerManager cm(&robot,nh);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(40);
  while (ros::ok()){
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    
    
    cm.update(ts, d);
    
    /*if(!strcmp(argv[1], "PSM2")){
      robot.read(std::string(argv[1]));
      robot.write(std::string(argv[1]));
    }else{
      //std::cout << "\n";
    }*/
    
    
    robot.pcp();
    
    /*if(!strcmp(argv[1], "PSM2")){
      robot.write(std::string(argv[1]));
      std::cout << "==============================\n";
    }else{
      //std::cout << "\n";
    }*/
    
    rate.sleep();
  }

  spinner.stop();

  return 0;
}


DavinciHWI::DavinciHWI(ros::NodeHandle & n, const std::string & psm){

  ROS_INFO("Setting up DP publishers.");
  joint_publisher = n.advertise<sensor_msgs::JointState>("/dvrk/" + psm + "/set_position_joint", 1, true);
  mode_publisher = n.advertise<std_msgs::String>("/dvrk/" + psm + "/set_robot_state", 1, true);
  
  state = "";
  wfcb = true;

  ROS_INFO("Subscribing to feedback...");
  
  state_sub = n.subscribe("/dvrk/" + psm + "/state_joint_current", 1, &DavinciHWI::CB_update, this);
  mode_sub = n.subscribe("/dvrk/" + psm + "/robot_state", 1, &DavinciHWI::CB_state, this);

  for(int i = 0; i < 7; i++){
    cmd[i]=-10.0;
    pos[i]=0;
    vel[i]=0;
    eff[i]=0;
  }
  
  ROS_INFO("Registering JSI...");
  
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_a("outer_yaw", &pos[0], &vel[0], &eff[0]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_a);
  hardware_interface::JointStateHandle state_handle_b("outer_pitch", &pos[1], &vel[1], &eff[1]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_b);
  hardware_interface::JointStateHandle state_handle_c("outer_insertion", &pos[2], &vel[2], &eff[2]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_c);
  hardware_interface::JointStateHandle state_handle_d("outer_roll", &pos[3], &vel[3], &eff[3]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_d);
  hardware_interface::JointStateHandle state_handle_e("outer_wrist_pitch", &pos[4], &vel[4], &eff[4]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_e);
  hardware_interface::JointStateHandle state_handle_f("outer_wrist_yaw", &pos[5], &vel[5], &eff[5]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_f);
  hardware_interface::JointStateHandle state_handle_g("jaw", &pos[6], &vel[6], &eff[6]); //pos vel eff outputs of the state message...
  jnt_state_interface.registerHandle(state_handle_g);


  hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("outer_yaw"), &cmd[0]); //cmd is the commanded value depending on the controller.
  hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("outer_pitch"), &cmd[1]); //cmd is the commanded value depending on the controller.
  hardware_interface::JointHandle pos_handle_c(jnt_state_interface.getHandle("outer_insertion"), &cmd[2]); //cmd is the commanded value depending on the controller.
  hardware_interface::JointHandle pos_handle_d(jnt_state_interface.getHandle("outer_roll"), &cmd[3]); //cmd is the commanded value depending on the controller.
  hardware_interface::JointHandle pos_handle_e(jnt_state_interface.getHandle("outer_wrist_pitch"), &cmd[4]); //cmd is the commanded value depending on the controller.
  hardware_interface::JointHandle pos_handle_f(jnt_state_interface.getHandle("outer_wrist_yaw"), &cmd[5]); //cmd is the commanded value depending on the controller.
  hardware_interface::JointHandle pos_handle_g(jnt_state_interface.getHandle("jaw"), &cmd[6]); //cmd is the commanded value depending on the controller.
  registerInterface(&jnt_state_interface);
  
  ROS_INFO("Registering JPIs...");

  // connect and register the joint position interface
  jnt_pos_interface.registerHandle(pos_handle_a);
  jnt_vel_interface.registerHandle(pos_handle_a);
  jnt_eff_interface.registerHandle(pos_handle_a);
  
  jnt_pos_interface.registerHandle(pos_handle_b);
  jnt_vel_interface.registerHandle(pos_handle_b);
  jnt_eff_interface.registerHandle(pos_handle_b);
  
  jnt_pos_interface.registerHandle(pos_handle_c);
  jnt_vel_interface.registerHandle(pos_handle_c);
  jnt_eff_interface.registerHandle(pos_handle_c);
  
  jnt_pos_interface.registerHandle(pos_handle_d);
  jnt_vel_interface.registerHandle(pos_handle_d);
  jnt_eff_interface.registerHandle(pos_handle_d);
  
  jnt_pos_interface.registerHandle(pos_handle_e);
  jnt_vel_interface.registerHandle(pos_handle_e);
  jnt_eff_interface.registerHandle(pos_handle_e);
  
  jnt_pos_interface.registerHandle(pos_handle_f);
  jnt_vel_interface.registerHandle(pos_handle_f);
  jnt_eff_interface.registerHandle(pos_handle_f);
  
  jnt_pos_interface.registerHandle(pos_handle_g);
  jnt_vel_interface.registerHandle(pos_handle_g);
  jnt_eff_interface.registerHandle(pos_handle_g);
  
  registerInterface(&jnt_pos_interface);
  registerInterface(&jnt_vel_interface);
  registerInterface(&jnt_eff_interface);
  
  ros::spinOnce();
}

void DavinciHWI::CB_update(const sensor_msgs::JointState::ConstPtr& incoming){

  std::vector<double> poss = incoming -> position;
  std::vector<double> vels = incoming -> velocity;
  std::vector<double> effs = incoming -> effort;
  
  for(int i = 0; i < 7; i++){
    pos[i] = poss[i];
    vel[i] = vels[i];
    eff[i] = effs[i];
  }
  
  wfcb = false;
}

void DavinciHWI::CB_state(const std_msgs::String::ConstPtr& incoming){
	state = incoming->data;
}

void DavinciHWI::pcp(){

if(state.compare("DVRK_READY") == 0){
	ROS_INFO("READY STATE DETECTED, MOVING INTO JOINT MODE.");
	std_msgs::String s;
	s.data = "DVRK_POSITION_JOINT";
	mode_publisher.publish(s);
} else if(state.compare("") == 0){
	ROS_INFO("NO STATE DETECTED. ASSUMING ALREADY HOMED.");
	std_msgs::String s;
	s.data = "DVRK_POSITION_JOINT";
	mode_publisher.publish(s);
}

  std::vector<std::string> jns = {
    "outer_yaw",
    "outer_pitch",
    "outer_insertion",
    "outer_roll",
    "outer_wrist_pitch",
    "outer_wrist_yaw",
    "jaw"
  };
  
  bool ok_to_pub = true;
  
  for(int i = 0; i < 7; i++){
    if(cmd[i] == -10.0){
    	ok_to_pub = false;
    }
  }
  
  if(!wfcb && ok_to_pub){
    std::cout << "Publishing " << "  " << cmd[0] << "  " << cmd[1] << "  " << cmd[2] << "  " << cmd[3] << "  " << cmd[4] << "  " << cmd[5] << "  " << cmd[6] << "\n";
  std::cout << "Resultance " << "  " << pos[0] << "  " << pos[1] << "  " << pos[2] << "  " << pos[3] << "  " << pos[4] << "  " << pos[5] << "  " << pos[6] << "\n";
  sensor_msgs::JointState outgoing;
  outgoing.name = jns;
  outgoing.position = cmd;
  outgoing.header.stamp = ros::Time::now() + ros::Duration(0.25);
  
  joint_publisher.publish(outgoing);
  }
}
