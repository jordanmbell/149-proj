#!/bin/bash
source catkin_ws/devel/setup.bash || exit -1
if [ "$1" == "-s" ]; then
	rosrun romi_emulator button_press -s
else
	rosrun romi_emulator button_press
fi
