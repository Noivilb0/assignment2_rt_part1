# ROS Action client & custom (srv,msg,launch) files

## Overview
1. **Action Client Node**: A client node that interacts with the `/reaching_goal` action server to set a target goal (x, y) and move the robot to the target while providing feedback.
2. **Service to Track Last Target**: A service node that tracks the last target goal sent to the robot and responds with the last known target coordinates.
3. **Custom Message for Position and Velocity**: A subscriber listens to the robot's odometry data and publishes a custom message with the robot's position and velocity.
4. **Launch File**: A launch file that brings together multiple nodes for a coordinated execution.

## Nodes

### 1. `action_client.py`
- **Purpose**: The action client interacts with the `PlanningAction` server to set and send goals for the robot to reach. It publishes the robot's position and velocity and also handles canceling the goal if the user presses 'c'.
- **Topics**:
  - Subscribes to `/odom` to receive robot's odometry data.
  - Publishes to `/robot_pos_vel` to send the robot's position and velocity as a `Custom_msg`.
- **Services**: None

### 2. `last_target_service.py`
- **Purpose**: This service node tracks and returns the last target coordinates set for the robot. It listens to the `/reaching_goal/goal` topic and updates the stored coordinates.
- **Services**:
  - `Last_target`: Returns the last target coordinates as `float64` values for `x` and `y`.
- **Topics**:
  - Subscribes to `/reaching_goal/goal` to listen for new goals.

## Launch File
The launch file integrates the following nodes:
- `wall_follow_service.py`: A node for following a wall.
- `go_to_point_service.py`: A node for navigating to a target.
- `bug_as.py`: A node implementing bug-based navigation.
- `action_client.py`: The action client that sends goal coordinates to the robot and tracks its progress.
- `last_target_service.py`: A service node for querying the last set target coordinates.

### Launch command:
```bash
roslaunch assignment_2_2024 assignment1.launch
