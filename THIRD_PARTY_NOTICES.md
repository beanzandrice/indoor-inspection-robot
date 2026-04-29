# Third-Party Credits and Notices

This repository contains project-owned launch files, bridge scripts, configuration, and documentation for Team 10's Unitree Go2 navigation workflow. It does not vendor full third-party source trees. The stack depends on and was developed around the projects listed below.

Open source does not mean attribution is unnecessary. Each dependency remains under its own license, and any copied or modified upstream files should retain the upstream copyright and license notices.

## Core Robot and ROS Projects

- Unitree Robotics, `unitree_ros2`: https://github.com/unitreerobotics/unitree_ros2
- Unitree Robotics, `unitree_sdk2`: https://github.com/unitreerobotics/unitree_sdk2
- Zhuo An, `go2_ros2_toolbox`: https://github.com/andy-zhuo-02/go2_ros2_toolbox
- ROS 2 Foxy documentation: https://docs.ros.org/en/foxy/index.html
- ROS 2 Navigation2 / Nav2: https://github.com/ros-navigation/navigation2
- Nav2 documentation: https://docs.nav2.org/
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- Eclipse Cyclone DDS: https://github.com/eclipse-cyclonedds/cyclonedds
- RViz2 documentation: https://docs.ros.org/en/ros2_packages/rolling/api/rviz2/index.html
- ROS 2 `tf2_ros` static transform publisher documentation: https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
- Teddy Liao, `walk-these-ways-go2`: https://github.com/Teddy-Liao/walk-these-ways-go2

## Project-Specific Notes

- The GO2 navigation workflow assumes `go2_core` and `go2_perception` packages from the Go2 ROS 2 toolbox environment.
- The `go2_navigation` package in this repository is a project copy used for Team 10 testing and documentation. It includes project modifications for one-command launch, live mapping, static-map navigation, and RViz TCP bridging.
- The included static room map under `go2_navigation/maps/` was generated during Team 10 testing and is project data rather than third-party code.
- The Go2 URDF and DAE mesh files under `robot_description/go2/` were copied from `walk-these-ways-go2` for RViz visualization. The upstream MIT license copy is stored at `robot_description/go2/licenses/walk-these-ways-go2-LICENSE`.

## License Reminder

Before publishing this repository publicly or distributing it outside the team, review the upstream licenses for any files copied or adapted from the projects above. If a file was copied from an upstream repository, keep the original license header when present and follow that repository's license requirements.
