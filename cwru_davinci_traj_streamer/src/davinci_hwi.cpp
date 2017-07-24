#include "cwru_davinci_traj_streamer/cwru_davinci_hwi.h"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"

#include <ros/callback_queue.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "interface_echo_node");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  
  ROS_INFO("Got argument %s", argv[1]);

  DavinciHWI robot(nh, std::string(argv[1]));
  controller_manager::ControllerManager cm(&robot,nh);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(50);
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

  ROS_INFO("Subscribing to feedback...");
  
  state_sub = n.subscribe("/dvrk/" + psm + "/joint_states", 1, &DavinciHWI::CB_update, this);

  for(int i = 0; i < 7; i++){
    cmd[i]=0;
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
}

void DavinciHWI::pcp(){
  std::vector<std::string> jns = {
    "outer_yaw",
    "outer_pitch",
    "outer_insertion",
    "outer_roll",
    "outer_wrist_pitch",
    "outer_wrist_yaw",
    "jaw"
  };
  
  //std::cout << "Publishing " << "  " << cmd[0] << "  " << cmd[1] << "  " << cmd[2] << "  " << cmd[3] << "  " << cmd[4] << "  " << cmd[5] << "  " << cmd[6] << "\n";
  
  for(int i = 0; i < 7; i++){
    //cmd[i] = pos[i] - cmd[i] * (1.0 / 1.0);
  }
  
  sensor_msgs::JointState outgoing;
  outgoing.name = jns;
  outgoing.position = cmd;
  
  joint_publisher.publish(outgoing);
}
