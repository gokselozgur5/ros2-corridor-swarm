# ros2-corridor-swarm

A one-week ROS 2 Jazzy project on Jetson Thor, fully simulated.

Vehicles patrol a mission corridor (the KML/GeoJSON polygon from map-writer), stay inside a geofence, and report to ATAK over CoT. Foxglove on the Mac shows the ROS side.

## Environment

ROS 2 Jazzy is a source build at `~/.goske/ros2_jazzy` on Thor: `source ~/.goske/ros2_jazzy/install/setup.bash`. Extra packages (foxglove_bridge, ros_gz, Nav2) are built in this workspace. Mac: Foxglove app, connect to `ws://192.168.88.229:8765`.

## Week

1. Workspace, foxglove_bridge from source, first vehicle sim node, first topic seen from the Mac.
2. Vehicle sim: lat/lon kinematics, odometry, tf, NavSatFix; the corridor polygon loaded from GeoJSON.
3. Patrol action server and geofence against the corridor polygon.
4. CoT bridge: vehicles appear in ATAK (FreeTAKServer in Docker, ATAK in an emulator).
5. End to end: ATAK marker becomes a vehicle waypoint.
6. Several vehicles, QoS, lifecycle nodes, launch.
7. Demo video and this README.
