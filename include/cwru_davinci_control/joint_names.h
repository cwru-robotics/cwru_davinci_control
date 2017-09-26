#ifndef CWRU_DAVINCI_CONTROL_JOINT_NAMES_H
#define CWRU_DAVINCI_CONTROL_JOINT_NAMES_H

namespace dvrk_control{
	const std::vector<std::string> joint_names = {
	//These should always reflect the joint names given in cwru_davinci_geometry_models/model
		"outer_yaw",
		"outer_pitch",
		"outer_insertion",
		"outer_roll",
		"outer_wrist_pitch",
		"outer_wrist_yaw",
		"jaw"
	};
};

#endif
