# CityLab Project: Autonomous Patrol & Navigation (ROS 2 / C++)

A ROS 2 project for the FastBot mobile robot that combines three core ROS 2 communication patterns (**topics, services and actions**) to build reactive obstacle-avoiding patrol behaviour and goal-based navigation, all in C++.

Built as the CityLab project of the ROS Masterclass by The Construct.

## Demos

### Patrol while avoiding obstacles

https://github.com/user-attachments/assets/c1860156-7674-4a44-9ce7-58ab7aa342f6

### Navigate to a requested pose

https://github.com/user-attachments/assets/6e8ff251-dd96-4c4f-96dc-a8fbf14eabfe

## At a Glance

| | |
|---|---|
| **What it does** | Robot patrols an area indefinitely, steering toward the most open space using LiDAR, and can also navigate to a requested pose |
| **Language / stack** | C++, ROS 2 (rclcpp, rclcpp_action), tf2, RViz2, CMake / colcon |
| **Robot** | FastBot (`fastbot_1` namespace) |
| **Sensors used** | 2D LiDAR (`LaserScan`), wheel odometry (`Odometry`) |
| **ROS 2 patterns** | Publisher/subscriber, service server + client, action server, custom interfaces |
| **Packages** | `robot_patrol` (nodes + launch files), `custom_interfaces` (service + action definitions) |

## Skills Demonstrated

| Area | Where it shows up |
|---|---|
| ROS 2 topics (pub/sub, QoS) | `patrol.cpp`, `patrol_with_service.cpp`, `go_to_pose_action.cpp` |
| ROS 2 services (server and async client) | `direction_service.cpp`, `patrol_with_service.cpp`, `test_service.cpp` |
| ROS 2 actions (goal, feedback, result, cancel) | `go_to_pose_action.cpp` |
| Custom interface design | `custom_interfaces/srv/GetDirection.srv`, `custom_interfaces/action/GoToPose.action` |
| LiDAR data processing | Sector analysis, moving-average filtering, range validation |
| Kinematics & control | Yaw from quaternion, turn-then-drive controller, angle normalisation, tolerances |
| Concurrency | Action goal executed on a dedicated thread so the executor is never blocked |
| Build & tooling | Multi-package colcon workspace, CMake, launch files in Python, RViz2 config |
| Modern C++ | Classes, `std::bind`, smart pointers, `constexpr`, STL containers |

## Repository Structure

```
citylab_project/
├── custom_interfaces/
│   ├── action/GoToPose.action
│   ├── srv/GetDirection.srv
│   ├── CMakeLists.txt
│   └── package.xml
└── robot_patrol/
    ├── src/
    │   ├── patrol.cpp
    │   ├── direction_service.cpp
    │   ├── patrol_with_service.cpp
    │   ├── test_service.cpp
    │   └── go_to_pose_action.cpp
    ├── launch/
    │   ├── start_patrolling.launch.py
    │   ├── main.launch.py
    │   ├── start_direction_service.launch.py
    │   ├── start_test_service.launch.py
    │   └── start_gotopose_action.launch.py
    ├── rviz/patrol.rviz
    ├── CMakeLists.txt
    └── package.xml
```

## What Each File Does

### Interfaces: `custom_interfaces`

| File | Purpose |
|---|---|
| `srv/GetDirection.srv` | Service definition. **Request:** a full `sensor_msgs/LaserScan`. **Response:** a `string` direction (`forward`, `left` or `right`). |
| `action/GoToPose.action` | Action definition. **Goal:** target `Pose2D`. **Feedback:** current `Pose2D`. **Result:** `bool` status. |

### Nodes: `robot_patrol/src`

| File (executable) | What it does | Key techniques |
|---|---|---|
| `patrol.cpp` (`patrol_executable`) | **Standalone reactive patrol.** Subscribes to the LiDAR, looks only at the front 180° (rays 50–150 of 200), smooths them with a 7-ray moving average and finds the most open direction. A 10 Hz timer drives forward at 0.1 m/s and, when an obstacle is detected closer than 0.35 m in the front arc, turns toward the open direction with angular velocity proportional to that angle (`direction / 2`). | Subscriber/publisher, wall timer, QoS, sliding-window filtering, `inf` handling |
| `direction_service.cpp` (`direction_service_executable`) | **Decision service** at `/direction_service`. Receives a LiDAR scan and sums valid ranges in three sectors: front (±30°), left (30° to 90°) and right (−90° to −30°). Returns the direction with the most free space. | Service server, sector analysis, angle-to-index conversion with wrap-around, range validation |
| `patrol_with_service.cpp` (`patrol_client_executable`) | **Service-based patrol.** Caches the latest scan, calls `/direction_service` every 500 ms with an async request, and publishes velocity commands from the answer (forward: 0 rad/s, left: +0.5, right: −0.5 at 0.1 m/s). | Async service client, callbacks, waiting for service availability, decoupled architecture |
| `test_service.cpp` (`direction_tester_executable`) | **Test client** that feeds live LiDAR data to the direction service and logs the responses, so the service can be verified without moving the robot. | Service testing, isolating components |
| `go_to_pose_action.cpp` (`go_to_pose_action_executable`) | **Navigation action server** at `/go_to_pose`. Reads odometry, converts the quaternion to yaw (tf2), then runs a 10 Hz loop: rotate toward the goal, drive to it (0.2 m/s), then rotate to the requested final heading. Publishes feedback with the current pose, supports cancellation, and finishes with a success result. Tolerances: 7.5 cm position, 10° orientation. | Action server, tf2, angle normalisation, threaded execution, feedback/cancel handling |

### Launch & Config

| File | Purpose |
|---|---|
| `launch/start_patrolling.launch.py` | Runs the standalone patrol node and RViz2. |
| `launch/main.launch.py` | Runs the full service-based system: direction service, patrol client and RViz2. |
| `launch/start_direction_service.launch.py` | Starts only the direction service. |
| `launch/start_test_service.launch.py` | Starts only the service tester client. |
| `launch/start_gotopose_action.launch.py` | Runs the GoToPose action server and RViz2. |
| `rviz/patrol.rviz` | Preconfigured RViz2 layout for visualising the robot and scan. |

## System Architecture

```
Standalone patrol
  /fastbot_1/scan ──► patrol ──► /fastbot_1/cmd_vel

Service-based patrol
  /fastbot_1/scan ──► patrol_client ──(GetDirection request)──► direction_service
                           │                                           │
                           └──────── /fastbot_1/cmd_vel ◄──(direction)─┘

Go to pose
  goal (x, y, θ) ──► /go_to_pose action server ──► /fastbot_1/cmd_vel
                           ▲
              /fastbot_1/odom (pose + yaw)
```

## Getting Started

### Requirements
- Ubuntu with ROS 2 (Jazzy)
- A FastBot robot or simulation publishing `/fastbot_1/scan`, `/fastbot_1/odom` and subscribing to `/fastbot_1/cmd_vel`
- `rviz2`, `tf2`, `rclcpp_action` (included in a standard ROS 2 desktop install)

### Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/jarrebear/citylab_project.git
cd ~/ros2_ws
colcon build --packages-select custom_interfaces robot_patrol
source install/setup.bash
```

### Run

**Standalone patrol**
```bash
ros2 launch robot_patrol start_patrolling.launch.py
```

**Service-based patrol**
```bash
ros2 launch robot_patrol main.launch.py
```

**Test the direction service on its own**
```bash
ros2 launch robot_patrol start_direction_service.launch.py
ros2 launch robot_patrol start_test_service.launch.py
```

**Navigate to a pose** (x, y in metres, theta in degrees)
```bash
ros2 launch robot_patrol start_gotopose_action.launch.py
ros2 action send_goal /go_to_pose custom_interfaces/action/GoToPose \
  "{goal_pos: {x: 0.5, y: 0.5, theta: 90.0}}" --feedback
```

## Design Notes

- **Why both a standalone and a service version?** The standalone node is the simplest correct solution. The service version separates *deciding* (what direction is safest) from *acting* (publishing velocities), so the decision logic can be tested and reused independently.
- **Why a thread for the action goal?** Execution runs on its own thread so the callback returns quickly and the executor stays responsive to odometry and cancel requests.
- **Topic namespace:** topics are currently hard-coded to `/fastbot_1/...`. Remapping or a parameter would be needed for other robots.

## Possible Improvements

- Expose speeds, thresholds and the topic namespace as ROS 2 parameters
- Add obstacle avoidance to the `GoToPose` server (currently it drives straight to the goal)
- Replace the bang-bang turning with proportional control for smoother motion
- Add unit tests for the sector-sum logic (`ament_lint` / `gtest`)

## License

Released under the [MIT License](LICENSE).

## Author

**jarrebear**: [github.com/jarrebear](https://github.com/jarrebear)
