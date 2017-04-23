<b>Kind of needs a complete rewrite.</b>

Node to receive Davinci trajectories and interpolate them smoothly as commands to Davinci;
action message goal includes trajectory_msgs/JointTrajectory.
action server breaks this up into incremental commands, spaced at dt_traj.
populates and publishes joint commands every dt_traj.
values for dt_traj and max velocities are set in header, davinci_traj_streamer.h

March, 2017:
If using Gazebo simulator, start Gazebo, e.g. with:

`roslaunch cwru_davinci_gazebo davinci_yellow_config.launch`
(for the base-frame configuration corresponding to yellow tapes on our DaVinci)

Start the trajectory interpolation action server:
`rosrun cwru_davinci_traj_streamer davinci_traj_interpolator_as`

See traj_action_client_pre_pose.cpp for example of how to use this action server
See, also, cwru_playfile_reader for a compatible playfile node.





