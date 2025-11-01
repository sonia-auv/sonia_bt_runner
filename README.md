# sonia_bt_runner

The project runs a **mission server** that accepts and launchs missions on the **AUV** prototype for autonomous operations. It also handles the generation of custom **Behavior Tree nodes** used within these missions. The mission server oversees execution from start to finish, providing real-time control and feedback throughout the operation.

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

sonia_bt_missions contains the missions used for autonomous operations as well as the generated sonia behaviortree nodes required by groot. The missions are created using **Groot2**, an application that helps interact with behaviortree nodes.

![Groot2 interface](https://www.behaviortree.dev/img/groot2.png)

---

## Node

* Name: `Mission_server`

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

### Environment variables
Required environment variables to launch the project

```bash
export SONIA_WS={path_to_your_ros2_workspace}
```
replace `{path_to_your_ros2_workspace}` with the path to your ros2 workspace

### Node generate command
The command generates an xml file of custom behaviortree nodes required by groot to create sonia missions.

```bash
ros2 run sonia_bt_runner gen_nodes
```

### Default launch
The command launchs the mission server 

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
* [BehaviorTree.CPP](https://www.behaviortree.dev/)
* [Groot2](https://www.behaviortree.dev/groot/)

---