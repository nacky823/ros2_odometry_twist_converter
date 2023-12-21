# ros2_odometry_twist_converter

ROS 2 Topic の Message type を `nav_msgs/msg/odometry` から `geometry_msgs/msg/twist_with_covariance_stamped` に変換します。

## Environment

+ OS: Ubuntu 22.04.3 LTS
+ ROS 2 Distribution: Humble Hawksbill

## Input/Output

| **Topic** | **Type** | **Description** |
| --- | --- | --- |
| `/input_odom` | `nav_msgs/msg/odometry` | Input odometry |
| `/output_twist_with_cov_stamp` | `geometry_msgs/msg/twist_with_covariance_stamped` | Output twist_with_cov |

## Usage

```
ros2 launch ros2_odometry_twist_converter odom_to_twist_cov_stamp.launch.py
```

## License

このソフトウェアパッケージは、Apache License 2.0 のもと、再配布および使用が許可されます。

© 2023 nacky823