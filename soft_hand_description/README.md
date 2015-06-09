Soft Hand model
===============

If you want to use the Pisa/IIT soft hand in your URDF, you can do so by including the model:

`<xacro:include filename="$(find soft_hand_description)/model/soft_hand.urdf.xacro"/>`

And then, using as many hands as you want as:

```
<xacro:soft_hand name="MYHAND" parent="PARENT" 
                 withAdaptiveTransmission="true" useMimicTag="false" left="true">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:soft_hand>
```
Where:

`name` is the name of your hand, it is useful for namespaces, controllers, etc.

`parent` is the link you are attaching your hand to, placed at `<origin...`

`withAdaptiveTransmission` used  for simulation purposes for now.

`useMimicTag` is to have only one joint that controls all joints in a pure-kinematics-like motion, it affects only the visualization. In simulation, the mimicking is done by the hardware interface, and in real, you only have one motor reading.

`left` is to define whether you are using a right or left hand.

