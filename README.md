# cwru_davinci_control

[![Build Status](https://travis-ci.com/cwru-robotics/cwru_davinci_control.svg?token=YmHMxBbcdppbMMkZWTut&branch=master)](https://travis-ci.com/cwru-robotics/cwru_davinci_control)

## Package Features

This repository includes features for controlling a physical or simulated Da Vinci robot, as well as graceful trajectory blending and preemption. By acting as a layer of seperation, this package enables the testing of motion plans on both the simulated and physical hardware and improves accuracy by reducing problems such as ringing and overshoot.
This package includes the following executables and libraries:

### Executables
*davinci_hwi*: Bridges a ROS cotroller to the dvrk. This should be called using a launch file.

### Libraries
*psm_controller*: Wrapper for motion control ROS topics, supporting partial-joint control and a wide array of input types.
The main library invocation uses the index of the PSM arms.

## Installation
This package requires the ros_controllers package, which is *not* included by default in ROS Desktop. Before building, be sure you have run 
```
sudo apt-get install ros-[VERSION]-ros-controllers
sudo apt-get install ros-[VERSION]-joint-trajectory-controller
sudo apt-get install ros-[VERSION]-position-controller
```

## Usage
One controller is created per PSM, using pre-existing hardware or simulation features, ROS's native joint controllers, and davinci_hwi. A launch file suitable for the default 2-PSM configuration of the Da Vinci can be found in launch/control_both.launch.

