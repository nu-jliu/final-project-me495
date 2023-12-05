# Write Letters
**Author**: Allen Liu
This package takes the input of the coordinates of each letter and send the command to the robot via `moveit` to write a entire phrase on the whiteboard.

## How to launch
To launch the package in the fake mode:
```
ros2 launch write_letters writer.launch.xml hardware_type:=fake
```

To launch the package in real mode:
```
ros2 launch write_letters writer.launch.xml hardware_type:=real
```
## Nodes
 - `parser`: Parse the coordinate of the letters to actual waypoinits command
 - `writer`: Take different request and send the corresponding command for robot to move to.

## Services
 - `write [polygotbot_interfaces/srv/Write]`: Write the letters on the whiteboard with the letter
 ```
 ros2 service call write polyglotbot_interfaces/srv/Write '{characters: [points: [{x: 1.0, y: 1.0, z: 1.0}]]}'
 ```
 - `load_path [polygotbot_interfaces/srv/Path]`: Send the command for robot to move following resquested waypoints
 ```
 ros2 service call load_path polyglotbot_interfaces/srv/Path '{points: [{x: 1.0, y: 2.0, z: 1.0}]}'
 ```
 - `calibrate [std_srvs/srv/Empty]`: Command the robot to go to the calibrate pose
 ```
 ros2 service call calibrate std_srvs/srv/Empty
 ```
 - `homing [std_srvs/srv/Empty]`: Command the robot to go to the home pose
  ```
 ros2 service call homing std_srvs/srv/Empty
 ```
 - `grab_pen [polygotbot_interfaces/srv/GrabPen]`: Command the robot to grab a pen at a specific location
 ```
 ros2 service call grab_pen polyglotbot_interfaces/srv/GrabPen '{position: {x: 0.5, y: 0.0, z: 0.1}}'
 ```
 - `put_back`: Command the robot to put the pen back to the pen case
 ```
 ros2 service call put_back std_srvs/srv/Trigger
 ```