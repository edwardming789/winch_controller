# Winch Controller
## Installing
### ROS1 Winch Controller Steps
1. Create a catkin workspace ([Tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). We’ll call it `catkin_ws` for this example.
2. Clone the winch_controller package to `catkin_ws` with the command `git clone https://github.com/edwardming789/winch_controller.git` (If you want to use the simulator, you should also clone this [package](https://github.com/edwardming789/sensor_sim)).
3. Compile your workspace using the command `catkin_make` from the directory `catkin_ws/` (you may need to also use `source /opt/ros/<distro>setup.bash` if you haven’t already).
4. In a different terminal window, run `source /opt/ros/<distro>/setup.bash && roscore` (or make sure it’s already running).
5. Run the command `source devel/setup.bash` and `rosrun winch_controller controller`
6. Use the command `rosservice call simulator/set_depth` to set the target depth for the sensor
### Arduino Winch Controller Steps
1. Follow this [tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) to setup Arduino IDE
2. Upload the `winch_controller_arduino.ino` from the directory `winch_controller_arduino.ino` to your Arduino
3. Make sure the `roscore` is running and run this command `rosrun rosserial_python serial_node.py /dev/ttyUSB0` to activite the ROS node on Arduino
4. You can use these commands to manually control the winch
    * `rosservice call winch/winch_up` to retrive the sensor
    * `rosservice call winch/winch_down` to deploy the sensor
    * `rosservice call winch/winch_stop` to stop the winch
