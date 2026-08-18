# Third-Party Credits and Notices

This repository combines project-specific bridge scripts, launch/configuration changes, and documentation with software and visualization assets derived from upstream projects. It does not vendor complete third-party source trees. The relevant provenance and runtime dependencies are documented below.

Open source does not mean attribution is unnecessary. Each dependency remains under its own license, and any copied or modified upstream files should retain the upstream copyright and license notices.

## Core Robot and ROS Projects

- Unitree Robotics: [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) and [`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2)
- Zhuo An: [`go2_ros2_toolbox`](https://github.com/andy-zhuo-02/go2_ros2_toolbox)
- [ROS 2 Foxy documentation](https://docs.ros.org/en/foxy/index.html)
- [Navigation2 / Nav2](https://github.com/ros-navigation/navigation2) and [Nav2 documentation](https://docs.nav2.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds)
- [RViz2 documentation](https://docs.ros.org/en/ros2_packages/rolling/api/rviz2/index.html)
- [ROS 2 `tf2_ros` documentation](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html)
- Teddy Liao: [`walk-these-ways-go2`](https://github.com/Teddy-Liao/walk-these-ways-go2)

## Project-Specific Notes

- The navigation workflow assumes `go2_core` and `go2_perception` packages from the Go2 ROS 2 toolbox environment.
- The local `go2_navigation/` package is derived in substantial part from the upstream `go2_navigation` package in `go2_ros2_toolbox`. In particular, `CMakeLists.txt`, `package.xml`, `config/nav2_params.yaml`, and the launch-file foundation were adapted and extended for one-command launch, live mapping, saved-map navigation, and RViz TCP bridging. The [upstream MIT notice](third_party/licenses/go2_ros2_toolbox-LICENSE) is retained locally.
- The included static room map under `go2_navigation/maps/` is project-generated test data rather than third-party code.
- The Go2 URDF and DAE meshes under `robot_description/go2/` were copied from `walk-these-ways-go2` for RViz visualization. The [upstream MIT license](robot_description/go2/licenses/walk-these-ways-go2-LICENSE) is retained locally.

## License Reminder

No repository-wide license has been declared for the project-owned code. Before redistributing or reusing it, obtain or add an explicit license from the repository owner. Files copied or adapted from upstream projects must retain their original notices and follow their respective license terms.
