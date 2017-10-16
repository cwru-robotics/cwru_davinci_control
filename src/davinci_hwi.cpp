#include <cwru_davinci_control/cwru_davinci_hwi.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/actuator_state_interface.h>

#include <ros/callback_queue.h>

using namespace dvrk_control;

int main(int argc, char** argv){

  ros::init(argc, argv, "interface_echo_node");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
 

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

	hardware_interface::JointStateHandle jsh[7];
	hardware_interface::JointHandle jh[7];

  
  ROS_INFO("Registering JSI...");
  
  // connect and register the joint state interface  
  for(int i = 0; i < 7; i++){
  	jsh[i] = hardware_interface::JointStateHandle(joint_names[i], &pos[i], &vel[i], &eff[i]); //pos vel eff outputs of the state message...
  	jnt_state_interface.registerHandle(jsh[i]);
  	jh[i] = hardware_interface::JointHandle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
  }
  
  ROS_INFO("Final register.");
  	
  registerInterface(&jnt_state_interface);
  
  ROS_INFO("Registering JPIs...");
  
  for(int i = 0; i < 7; i++){
  	jnt_pos_interface.registerHandle(jh[i]);
  	jnt_vel_interface.registerHandle(jh[i]);
  	jnt_eff_interface.registerHandle(jh[i]);
  }
  
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
  
  bool ok_to_pub = true;
  
  for(int i = 0; i < 7; i++){
    if(cmd[i] == -10.0){
    	ok_to_pub = false;
    }
  }
  
  if(!wfcb && ok_to_pub){
  sensor_msgs::JointState outgoing;
  outgoing.name = joint_names;
  outgoing.position = cmd;
  outgoing.header.stamp = ros::Time::now() + ros::Duration(0.25);
  
  joint_publisher.publish(outgoing);
  }
}
