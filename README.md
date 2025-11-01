# sonia_bt_runner

*description*

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `rclcpp_action`
* `behaviortree_cpp`
* `std_msgs`
* `std_srvs`

### Sonia packages

* `sonia_common_ros2`

---
## sonia_bt_missions

*description*

###
---

## Node

* Name: `Mission_server`
* Port Name: ``

---

## Registered Topics / Services / Actions

| Type             | Name                            | Direction       | Message/Service Type                       | Description                                 |
| ---------------- | ------------------------------- | ----------------| ------------------------------------------ | ------------------------------------------- |
| Topic            | `/mission_server/status_report` | Published       | `std_msgs/msg/String`                      | Template                                    |
| Service          | `/mission_server/mission_list`  | Service Server  | `sonia_common_ros2/srv/MissionListService` | Template                                    |
| Action           | `MissionControl`                | Action Server   | `sonia_common_ros2/action/MissionControl`  | Measured temperature data                   |

---
## Build Instructions
To build the project, the following commands should be run directly from your ROS2 workspace.

```bash
colcon build --packages-select sonia_bt_runner --symlink-install
source install/setup.bash
```

---

## Launch Instructions

### Default launch

```bash
ros2 launch sonia_bt_runner launch.py
```

---

## Useful ROS 2 Commands

```bash
ros2 node list
ros2 node info /Mission_server
ros2 topic echo /mission_server/status_report
ros2 param list /Mission_server
```

---

## References

* [sonia_common_ros2](https://github.com/sonia-auv/sonia_common_ros2)

---