# carla_control.py
1. run carla
  ```bash
  /CarlaUE4.sh
  ```

2. spawn ego vehicle (rolename can be either `hero` or `ego_vehicle`
  ```bash
  python3 /carla-root-dir/PythonAPI/examples/manual_control.py
  ```

3. run g29-force-feedback node

4. run example code (rospy node, carla_ff_node)
  ```bash
  conda deactivate
  cd /ROS2/ros2_g29/
  . install/setup.sh
  python3 src/ros-g29-force-feedback/examples/carla_control.py
  ```
  
5. control ego vehicle
 
