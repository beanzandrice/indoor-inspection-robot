# Unitree Go2 Navigation and RViz Bridge

This repository contains the project-owned code used to run the Unitree Go2 navigation stack from a laptop through a Raspberry Pi 3 relay. It supports two robot modes:

- **Live mapping:** runs SLAM Toolbox online mapping, Nav2, Go2 base bringup, point cloud processing, and the RViz bridge.
- **Static map navigation:** runs Nav2 localization with the saved room map, AMCL, Nav2 navigation, Go2 base bringup, point cloud processing, and the RViz bridge.

The repo intentionally does **not** store SSH passwords, campus credentials, runtime logs, or local-only `bridge.env` files.

## System Architecture

```text
Laptop running RViz2
  |
  | SSH ProxyJump through hotspot or shared network
  v
Raspberry Pi 3 relay
  |
  | Wired robot-side network
  v
Unitree Go2
  |
  | ROS 2 Foxy, go2_core, go2_perception, Nav2, SLAM Toolbox
  v
Navigation stack and RViz TCP bridge
```

The TCP bridge exists because direct ROS 2 discovery between the laptop and the robot is unreliable across the Pi relay and hotspot route. The bridge relays the data RViz needs from the robot to the laptop and forwards RViz commands such as 2D goal poses back to the robot.

## Repository Layout

```text
config/
  bridge.env.example              Laptop/Pi/Go2 network template
scripts/
  install_go2_project.sh          Copies and rebuilds the GO2-side project
  start_tunnels.sh                Opens SSH tunnels through the Pi to the GO2
  start_laptop_rviz.sh            Starts laptop receiver and RViz2
  start_go2_nav_over_ssh.sh       Starts static-map navigation on the GO2
  start_go2_live_mapping_over_ssh.sh
                                   Starts live mapping navigation on the GO2
  laptop_rviz_receiver.py         Laptop-side bridge receiver and command forwarder
  go2_robot_model_publisher.py    Laptop-side Go2 URDF and neutral TF publisher
rviz/
  go2_navigation.rviz             RViz display configuration
robot_description/
  go2/urdf/go2.urdf               Go2 robot model used by RViz
  go2/dae/*.dae                   Go2 model meshes
go2_navigation/
  launch/                         One-command ROS launch files for GO2 navigation
  config/                         Nav2 and SLAM Toolbox parameters
  maps/                           Static map YAML and PGM
  go2_navigation/rviz_tcp_bridge.py
                                   GO2-side ROS-to-TCP bridge
go2_scripts/
  start_navigation_with_rviz_bridge.sh
  start_live_mapping_with_rviz_bridge.sh
```

## Requirements

### Laptop

- Ubuntu with ROS 2 installed. The scripts try `/opt/ros/humble/setup.bash` first and then `/opt/ros/foxy/setup.bash`.
- `rviz2`
- Python 3
- SSH access to the Raspberry Pi 3
- Network route from the Pi to the Go2

### Go2

- ROS 2 Foxy
- `unitree_ros2` Cyclone DDS workspace
- `go2_ros2_ws` or equivalent packages that provide:
  - `go2_core`
  - `go2_perception`
  - `nav2_bringup`
  - `slam_toolbox`
  - `tf2_ros`

The GO2 project path used by these scripts is:

```bash
/home/unitree/go2_navigation_project
```

## First-Time Setup

Clone this repository on the laptop:

```bash
git clone <repo-url>
cd go2-navigation-rviz-project
```

Create the local network config:

```bash
cp config/bridge.env.example config/bridge.env
nano config/bridge.env
```

Set `PI_HOST`, `PI_USER`, `GO2_HOST`, and `GO2_USER` for the current network. Do not put passwords in this file.

The default RViz display support enables the robot model and keeps Go2 video disabled unless the Go2 video publisher is installed:

```bash
GO2_VIDEO_ENABLE=false
GO2_VIDEO_FPS=10
GO2_RVIZ_IMAGE_MAX_HZ=5.0
GO2_RVIZ_POINTCLOUD_MAX_HZ=2.0
ENABLE_GO2_MODEL=true
```

The robot model URDF path used by default is:

```bash
robot_description/go2/urdf/go2.urdf
```

If you want to use a different local Go2 URDF, set `GO2_URDF=/absolute/path/to/go2.urdf` in `config/bridge.env`.

Install the GO2-side code from the laptop:

```bash
./scripts/install_go2_project.sh
```

This command copies `go2_navigation/` and `go2_scripts/` to the Go2 through the Pi, then runs:

```bash
colcon build --symlink-install --packages-select go2_navigation
```

on the Go2.

## Running Live Mapping Navigation

Use three laptop terminals.

### Terminal 1: open SSH tunnels

```bash
cd ~/go2-navigation-rviz-project
./scripts/start_tunnels.sh
```

Leave this terminal open.

### Terminal 2: start laptop RViz receiver

```bash
cd ~/go2-navigation-rviz-project
./scripts/start_laptop_rviz.sh
```

This starts `laptop_rviz_receiver.py` and opens RViz2 with `rviz/go2_navigation.rviz`.
It also starts `go2_robot_model_publisher.py`, which publishes `/robot_description` and a neutral Go2 TF model from `robot_description/go2/urdf/go2.urdf`.

### Terminal 3: start live mapping on the Go2

```bash
cd ~/go2-navigation-rviz-project
./scripts/start_go2_live_mapping_over_ssh.sh
```

This launches:

- `go2_core` base bringup
- `go2_perception` point cloud processing
- Go2 video publishing on `/camera/image_raw`
- static transform `base_link -> base_footprint`
- SLAM Toolbox online async mapping
- Nav2 navigation
- GO2 RViz TCP bridge

In RViz, use `map` as the fixed frame. Wait until the map, TF, scan, point cloud, odometry, camera, and costmap displays start updating before sending goals.

## Running Static Map Navigation

Use the same first two terminals as live mapping.

### Terminal 1

```bash
cd ~/go2-navigation-rviz-project
./scripts/start_tunnels.sh
```

### Terminal 2

```bash
cd ~/go2-navigation-rviz-project
./scripts/start_laptop_rviz.sh
```

### Terminal 3

```bash
cd ~/go2-navigation-rviz-project
./scripts/start_go2_nav_over_ssh.sh
```

The static launch file publishes the saved initial pose and performs the map server deactivate/activate cycle automatically. If the robot starts somewhere different from the saved pose, update the launch arguments or publish a new initial pose before sending goals.

## Sending Goals From RViz

1. Confirm RViz fixed frame is `map`.
2. Confirm TF contains `map`, `odom`, `base_link`, and `base_footprint`.
3. Confirm `/scan`, `/trans_cloud`, `/camera/image_raw`, `/odom`, and costmap displays are updating.
4. Use the RViz **2D Goal Pose** tool to send a Nav2 goal.
5. Watch the local costmap and robot footprint before allowing the robot to move near obstacles.

Only run one navigation mode at a time. Do not run live mapping and static map localization together.

## RViz Displays

The included RViz config enables these main displays:

| Display | Topic or Source | Message Type | Notes |
| --- | --- | --- | --- |
| Map | `/map` | `nav_msgs/OccupancyGrid` | Static map or live SLAM map. |
| RobotModel | `/robot_description` | `std_msgs/String` URDF | Published locally by `scripts/go2_robot_model_publisher.py`. |
| LaserScan | `/scan` | `sensor_msgs/LaserScan` | 2D scan used by Nav2. |
| PointCloud | `/trans_cloud` | `sensor_msgs/PointCloud2` | Accumulated Go2 lidar point cloud. |
| Go2Camera | `/camera/image_raw` | `sensor_msgs/Image` | Go2 video stream from `go2_core`. |
| Odometry | `/odom` | `nav_msgs/Odometry` | Disabled by default in RViz but available. |
| TF | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Relayed from the robot plus local model TF. |

The underlying Go2 toolbox also exposes the raw deskewed lidar point cloud as `/utlidar/cloud_deskewed`. This repo displays `/trans_cloud` by default because it is the accumulated cloud already used by the navigation stack.

The camera and point cloud are high-bandwidth topics. They are relayed through the SSH tunnel with default limits of 5 Hz for images and 2 Hz for the point cloud. Lower these values in `config/bridge.env` if the hotspot becomes unstable.

## Stopping the Stack

Stop in this order:

1. Press `Ctrl+C` in the GO2 navigation terminal.
2. Press `Ctrl+C` in the RViz/laptop receiver terminal.
3. Press `Ctrl+C` in the tunnel terminal.

## Common Troubleshooting

### `Missing config/bridge.env`

Create it from the template:

```bash
cp config/bridge.env.example config/bridge.env
nano config/bridge.env
```

### SSH cannot reach the Go2

Test each hop:

```bash
ssh ${PI_USER}@${PI_HOST}
ssh -J ${PI_USER}@${PI_HOST} ${GO2_USER}@${GO2_HOST}
```

If the Pi IP changed, update `PI_HOST` in `config/bridge.env`.

### `connect_to 127.0.0.1 port 16000: failed`

The GO2 bridge is trying to send data through the reverse tunnel before the laptop receiver is listening. Start terminal 2 with:

```bash
./scripts/start_laptop_rviz.sh
```

The warning should stop once the receiver is running.

### RViz says `No tf data` or `Frame [map] does not exist`

Check that all three processes are running:

- tunnel terminal
- laptop RViz receiver terminal
- GO2 navigation terminal

Then check the GO2 terminal for bridge status lines such as:

```text
data=connected, commands=waiting, relayed: /tf=...
```

For static map navigation, AMCL cannot publish `map -> odom` until an initial pose is accepted. The static launch publishes the saved initial pose automatically after startup, but if the robot is not physically near that pose, publish a new initial pose from RViz or adjust the launch arguments.

### Robot model does not appear

Check the laptop terminal for:

```text
Go2 robot model PID: ...
Go2 URDF: ...
```

The default URDF path is:

```bash
robot_description/go2/urdf/go2.urdf
```

If that file is missing or you want a different model, set `GO2_URDF` in `config/bridge.env`. The model publisher also creates a neutral TF tree under `base_link`; the robot will not animate individual leg joints unless real joint state publishing is added later.

### Camera feed does not appear

The wrappers default to video disabled because some Go2 installs do not include `video_stream_node.py` in the `go2_core` package:

```bash
GO2_VIDEO_ENABLE=false
GO2_VIDEO_FPS=10
```

Only set `GO2_VIDEO_ENABLE=true` after confirming the Go2 can run the `go2_core` video publisher. Then verify the bridge status includes `/camera/image_raw`. If the hotspot is overloaded, lower the image relay rate:

```bash
GO2_RVIZ_IMAGE_MAX_HZ=2.0
```

### Point cloud does not appear

The RViz display uses `/trans_cloud`. Confirm the GO2 terminal shows `cloud_accumulation` and `pointcloud_to_laserscan_node` running, and check bridge status for `/trans_cloud`. If the point cloud causes lag, lower:

```bash
GO2_RVIZ_POINTCLOUD_MAX_HZ=1.0
```

### RViz data appears but goals do not move the robot

Confirm the GO2 bridge is receiving commands. The bridge opens the command channel on `127.0.0.1:16001` through the SSH tunnel. Also confirm Nav2 lifecycle nodes are active in the GO2 terminal.

### The robot pose slides in the map while turning

Small corrections are normal when localization or SLAM reconciles leg odometry with scan data. Large sliding during turns usually means one of these is off:

- scan frame timing or TF timing
- AMCL motion model tuning
- insufficient scan matching features during turns
- starting pose mismatch
- wheel/leg odometry drift during spin recovery

Use live mapping in feature-rich areas when possible, reduce aggressive spin recovery speeds, and avoid sending goals before TF and scans are stable.

### `AMENT_TRACE_SETUP_FILES: unbound variable`

This happens when `set -u` is active before sourcing ROS setup files. The included scripts source ROS first and enable `set -u` afterward on the GO2-side wrappers. If this appears in a modified script, move `set -u` after the ROS `source` commands.

### `Message Filter dropping message: EmptyFrameID`

This can happen briefly during point cloud processing startup. If it persists, check the Go2 perception launch and confirm the point cloud has a valid frame before conversion to `/scan`.

### `Timed out waiting for transform from base_link to map`

For static map mode, wait for AMCL to accept the initial pose. For live mapping mode, wait for SLAM Toolbox to start publishing `map -> odom`. Do not send goals until the transform tree is complete.

### Live mapping map does not update

Confirm SLAM Toolbox is installed on the Go2:

```bash
ros2 pkg prefix slam_toolbox
```

Confirm `/scan` is publishing and that TF includes the scan source frame, `base_link`, `base_footprint`, and `odom`.

## Notes

- This repository stores project-owned glue code and configuration, not third-party dependencies.
- Keep `config/bridge.env` local.
- Keep passwords in your password manager or enter them interactively at SSH prompts.
- The current static map is included under `go2_navigation/maps/`.

## Credits and Third-Party Dependencies

This project was built on top of the ROS 2 and Unitree Go2 ecosystem. The main third-party projects used by this navigation workflow are credited in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Open source code still has license requirements. This repository does not store full third-party source trees, but any upstream files copied or adapted into this project should retain their original notices and follow their upstream licenses.
