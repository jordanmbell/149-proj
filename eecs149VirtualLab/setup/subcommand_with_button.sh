#!/bin/bash

function runController {
	START_TIME=$SECONDS

	while [ $(($SECONDS - $START_TIME)) -lt 5 ]
	do
	if lsof -Pi :11311 -sTCP:LISTEN -t; then
		sleep 2s
		rosrun romi_emulator romi_emulator & 
		rosrun romi_emulator button_press -s &
		wait
	fi

	done
	echo "CONTROLLER COULD NOT CONNECT TO GAZEBO... ABORT"
	exit 1
}

runController
