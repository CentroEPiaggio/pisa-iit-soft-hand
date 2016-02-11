#rosrun tf2_ros static_transform_publisher -.24 .0 0.30 0.0 0.7068 0.0 0.7074 world box_desired
rostopic pub --once /tf_static tf/tfMessage "transforms:
- header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'world'
  child_frame_id: 'box_desired'
  transform:
    translation:
      x: -0.24
      y: 0.0
      z: 0.30
    rotation:
      x: 0.0
      y: 0.6816387600233341
      z: 0.0
      w: 0.7316888688738209"