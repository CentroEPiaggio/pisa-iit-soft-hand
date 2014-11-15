# Gazebo/ROS Soft Hand plugin #

`default_soft_hand_hw_sim.h` implements how the hand moves by default. Other behaviors can be implemented deriving from the abstract class `soft_hand_hw_sim.h`.

Using the plugin:

```
<gazebo>
	<plugin name="gazebo_ros_soft_hand" filename="libgazebo_ros_soft_hand.so">
		<robotNamespace>soft_hand</robotNamespace>
	</plugin>
</gazebo>
```