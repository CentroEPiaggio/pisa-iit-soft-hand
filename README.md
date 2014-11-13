pisa-iit-soft-hand
==================

This repository contains the model of the Pisa/IIT hand as described in:

* M. G. Catalano, Grioli, G., Farnioli, E., Serio, A., Piazza, C., and Bicchi, A., “Adaptive Synergies for the Design and Control of the Pisa/IIT SoftHand”, International Journal of Robotics Research, vol. 33, no. 5, pp. 768–782, 2014


Unless stated otherwise, all files within the repository are released under the BSD 3-Clause License, see the file LICENSE for the details.

0. Dependencies
---------------

Refer to CMakeLists.txt files and compilation errors if you are missing one.

Typically, all related package with ros_control, controllers, simulation gazebo4, and so on. Simulations tested with Gazebo4.

And you need the forked version of ros_control where the adaptive synergy transmission is implemented:
`git clone  `

1. Adaptive model
-----------------

This model uses one joint actuated by position, and propagate the resulting configuration using the adaptive synergy transmission.

NOTE: The gazebo simulation is still not working!

To see an example, launch:

`roslaunch soft_hand_description gazebo_adaptive_actuation.launch`

Open the hand with:

`rostopic pub /soft_hand/hand_synergy_joint_position_controller/command std_msgs/Float64 "data: 0.0" &`

Close the hand with:

`rostopic pub /soft_hand/hand_synergy_joint_position_controller/command std_msgs/Float64 "data: 1.0" &`

2. Fully actuated model
-----------------------

This model might be useful for any application that implements the adaptive synergy externally, or any other kind of transmission, and publishes the computed torque for each joint.

IMPORTANT: You must ensure that the mimic joints have the same value as the joints they are mimicking!

To see an example, launch:

`roslaunch soft_hand_description gazebo_full_actuation.launch`

Open the hand with:

```
rostopic pub /soft_hand/hand_thumb_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_thumb_inner_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_thumb_outer_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_inner_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_middle_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_outer_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_inner_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_middle_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_outer_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_inner_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_middle_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_outer_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_inner_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_middle_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_outer_joint_position_controller/command std_msgs/Float64 "data: 0.0" &

rostopic pub /soft_hand/hand_thumb_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_thumb_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.0" &
```

Close the hand with:

```
rostopic pub /soft_hand/hand_thumb_abd_joint_position_controller/command std_msgs/Float64 "data: 1.5" &
rostopic pub /soft_hand/hand_thumb_inner_joint_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_thumb_outer_joint_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_index_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_index_inner_joint_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_index_middle_joint_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_index_outer_joint_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_little_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_little_inner_joint_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_little_middle_joint_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_little_outer_joint_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_middle_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_middle_inner_joint_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_middle_middle_joint_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_middle_outer_joint_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_ring_abd_joint_position_controller/command std_msgs/Float64 "data: 0.0" &
rostopic pub /soft_hand/hand_ring_inner_joint_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_ring_middle_joint_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_ring_outer_joint_position_controller/command std_msgs/Float64 "data: 0.6" &

rostopic pub /soft_hand/hand_thumb_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_thumb_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_index_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_index_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_index_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_little_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_little_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_little_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_middle_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_middle_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_middle_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.6" &
rostopic pub /soft_hand/hand_ring_inner_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.7" &
rostopic pub /soft_hand/hand_ring_middle_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.5" &
rostopic pub /soft_hand/hand_ring_outer_joint_mimic_position_controller/command std_msgs/Float64 "data: 0.6" &
```

Here is a video of the gazebo simulator and rviz fed with the simulator data:
[![VIDEO](https://www.youtube.com/upload_thumbnail?v=css6i2ZP3J0&t=3&ts=1415543675751)](http://www.youtube.com/watch?v=css6i2ZP3J0).
