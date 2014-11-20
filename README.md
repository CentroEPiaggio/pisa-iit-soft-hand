pisa-iit-soft-hand (ROS/Gazebo packages)
========================================

This repository contains the model of the Pisa/IIT hand as described in:

* M. G. Catalano, Grioli, G., Farnioli, E., Serio, A., Piazza, C., and Bicchi, A., “Adaptive Synergies for the Design and Control of the Pisa/IIT SoftHand”, International Journal of Robotics Research, vol. 33, no. 5, pp. 768–782, 2014

[Free version of the paper](http://www.centropiaggio.unipi.it/sites/default/files/PisaIIT_SoftHand_0.pdf)
[IJRR version (access required)](http://ijr.sagepub.com/content/33/5/768.abstract)

CAD model freely available through the [Natural Machine Motion Initiative][http://www.naturalmachinemotioninitiative.com/#!softhand/c24r2]

Unless stated otherwise, all files within the repository are released under the BSD 3-Clause License, see the [LICENSE](https://github.com/CentroEPiaggio/pisa-iit-soft-hand/blob/master/LICENSE) file for the details.

0. Dependencies
---------------

Refer to CMakeLists.txt files and compilation errors if you are missing one.

Typically, all related package with ros_control, controllers, simulation gazebo4, and so on. Simulations tested with Gazebo4.

And you need the forked version of ros_control where the adaptive synergy transmission is implemented:
`git clone  https://github.com/CentroEPiaggio/ros_control.git`

1. Adaptive model
-----------------

This model uses one joint controlled by position, and propagate the resulting configuration using the adaptive synergy transmission to the subactuated joints.

To see an example, launch:

`roslaunch soft_hand_description gazebo_adaptive_actuation.launch`

Open the hand with:

`rostopic pub /soft_hand/hand_synergy_joint_position_controller/command std_msgs/Float64 "data: 0.0" &`

Close the hand with:

`rostopic pub /soft_hand/hand_synergy_joint_position_controller/command std_msgs/Float64 "data: 1.0" &`

You can also modify the `gazebo_adaptive_actuation.launch` at will, for instance, by selectin mimic_joint, you load a different hardware interface for simulation where the synergy works un pure kinematic control.

2. Fully actuated model
-----------------------

This model might be useful for any application that implements the adaptive synergy "by soft", in which you compute the torque for each joint and publish it.

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
