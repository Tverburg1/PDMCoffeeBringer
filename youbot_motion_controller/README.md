# YouBotMotionController

## Overview

This package is part of the [PDMCoffeeBringer project](PDMCoffeeBringer). This package is used to move the Kuka YouBot and monitor it's current state.

**Keywords:** Kuka YouBot, Controller

### License

The source code is released under a [MIT license](LICENSE).

**Authors: Jevon Ravenberg, Bauke Schenkelaars, Timo Stienstra, Tim Verburg<br>
Affiliation: [Delft University of Technology](https://www.tudelft.nl/en/)

The YouBotMotionController package has been tested under [ROS] Noetic on respectively Ubuntu 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation
Installation instructions are provided in the README of the entire project.
### Dependencies
* [youbot_description]

## Usage
Run the main node with

	roslaunch youbot_motion_controller youbot.launch

## Config files
* **arm_1_controller.yaml:** Controller settings for the arm.
* **joint_state_controller.yaml:** Settings for the JointStateController.

## Launch files
The launch files are mostly copied and edited from [youbot_simulation].
* **youbot.launch:** loads the YouBot in Gazebo including the necessary controllers.<br>
  Arguments:
    - **`world`** Name of the world launch file. Default: `empty_world`.
    - **`init_pos_x`** X position the YouBot initializes at. Default: `0.0`.
    - **`init_pos_y`** Y position the YouBot initializes at. Default: `0.0`.
    - **`init_pos_z`** Z position the YouBot initializes at. Default: `0.0`.
* **joint_state_controller.launch** Spawns the `joint_state_controller`.
* **arm_controller.launch** Spawns the controller for the arm and gripper.<br>
  Arguments:
  - **`arm_name`** Name of the arm. Default: `arm_1`.
* **empty_world.launch** Launches the empty world of Gazebo.

## Nodes

### youbot_motion_controller_node
Controller for the base and arm of the YouBot.
#### Subscribed Topics
* **`/youbot/control`** ([youbot_msgs/Control])<br>
  The configuration the YouBot should go to.
* **`/youbot/base/set_controller`** ([youbot_msgs/ControlSettings])<br>
  The settings the controller of the base should use, the input force is calculated in the following way: `F = k_pos * x - u(goal_position - brake_distance) * k_vel * v`, where `x` is the distance and `v` the velocity.
#### Published Topics
* **`/cmd_vel`**: Sets the velocity of the YouBot base.
* **`/arm_1/arm_controller/command`**: Configuration the arm should go to.

### youbot_configuration_publisher_node
Publishes the current configuration of the YouBot on the topic `/youbot/current_configuration` ([youbot_msgs/Control]). Other than this it uses the same topics and services as `youbot_motion_control_node`.

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker].

[PDMCoffeeBringer]: https://github.com/Tverburg1/PDMCoffeeBringer
[youbot_description]: https://github.com/Tstienstra/youbot_description
[youbot_simulation]: https://github.com/Tstienstra/youbot_simulation
[youbot_msgs/Control]: https://github.com/Tverburg1/PDMCoffeeBringer/blob/main/youbot_msgs/msg/Control.msg
[youbot_msgs/ControlSettings]: https://github.com/Tverburg1/PDMCoffeeBringer/blob/main/youbot_msgs/msg/ControlSettings.msg
[Issue Tracker]: https://github.com/Tverburg1/PDMCoffeeBringer/issues