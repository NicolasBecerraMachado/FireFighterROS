#!/bin/bash
roscore &
rostopic pub /autonomous_control_on std_msgs/Bool "data: true" &
chmod 666 /dev/ttyACM0
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=1000000 &
rosrun FireFighterRos pan_tilt_control.py &
rosrun FireFighterRos pan_tilt_control.py &
python3 headless_vision_jetsson.py &
rosrun FireFighterRos mecanum_drive.py &
rosrun FireFighterRos autonomous_control.py &
