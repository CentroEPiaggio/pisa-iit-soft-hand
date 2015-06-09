# SoftGrasp Publisher

Grasps are identified by the deformation of the soft hand w.r.t. the 1st synergy (used by construction). The ideal motion is computed using the synergy motor position and using the path each joint should have. The deformation is measured using one of the two gloves designed in the lab: the IMU-based and the flexiforce-based gloves. The former being more precise than the later.

Grasp types are defined with a Gaussian distribution per joint that model the typical deformation for that type of grasp (you need data for that).

The node receives two inputs in the form of a JointState topic, and publishes a [GraspState message](msg/GraspState.msg).

Default names are given for the input topics, but you can always remap or namespace depending on your application.

## Test

There is a test example for a grasp/no-grasp classification problem using three joints. Typically, the distribution for a no-grasp configuration is centered a zero with low variance.

Terminal 1:

`roslaunch grasp_state_publisher graspStatePublisher.launch test:=true`

Terminal 2:

`rostopic echo /grasp_state`

Terminal 3, publish a synergistic joint state:

`rostopic pub /joint_states sensor_msgs/JointState "position: [0.003, 0.002, 0.001]
velocity: [0, 0, 0]
effort: [0, 0, 0]" `

Terminal 4, publish examples of measured joint states:

Grasp:

`rostopic pub /glove/joint_states sensor_msgs/JointState "position: [0.5, 0.4, 0.3]
velocity: [0, 0, 0]
effort: [0, 0, 0]" `

No-grasp:

`rostopic pub /glove/joint_states sensor_msgs/JointState "position: [0.05, 0.04, 0.03]
velocity: [0, 0, 0]
effort: [0, 0, 0]" `
