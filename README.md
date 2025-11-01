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

sonia_bt_missions contains the missions used for autonomous operations. The
|[alt text](https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.behaviortree.dev%2Fgroot%2F&psig=AOvVaw19LZZBtV7SrYXU6vcXOB1O&ust=1762107197801000&source=images&cd=vfe&opi=89978449&ved=0CBUQjRxqFwoTCPjvv_7G0ZADFQAAAAAdAAAAABAE)
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
The command launchs the mission server 

```bash
ros2 launch sonia_bt_runner launch.py
```

### Node generate command
The command generates an xml file of custom behaviortree nodes required by groot to create sonia missions, 

```bash
ros2 run sonia_bt_runner gen_nodes
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
* [BehaviorTree.CPP](https://www.behaviortree.dev/)
* [Groot2](https://www.behaviortree.dev/groot/)

---