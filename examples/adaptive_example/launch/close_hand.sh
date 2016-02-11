rostopic pub --once /soft_hand/joint_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 1
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'soft_hand_synergy_joint'
points:
- positions: [0.9]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 1, nsecs: 0}"