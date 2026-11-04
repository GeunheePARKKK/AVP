#!/bin/bash
# velodyne -> 각 카메라 static TF (ROS 2)
# 인자 순서: x y z yaw pitch roll parent child

ros2 run tf2_ros static_transform_publisher 0.669415 -0.007431 -0.281479 -1.592790 -0.000312 -1.515928 velodyne cam_front   # 전방 FHH26070139
ros2 run tf2_ros static_transform_publisher -0.005804 0.504076 -0.290000 0.015976 0.026411 -1.498431 velodyne cam_left   # 왼쪽 FHH26070138
ros2 run tf2_ros static_transform_publisher 0.020788 -0.554131 -0.290000 3.092802 0.019487 -1.639147 velodyne cam_right   # 오른쪽 FHH26070137
ros2 run tf2_ros static_transform_publisher -0.679313 -0.003691 -0.290000 1.577236 -0.040048 -1.627304 velodyne cam_rear   # 후방 FHH26070140
