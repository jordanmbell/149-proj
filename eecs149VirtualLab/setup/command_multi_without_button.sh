#!/bin/bash

# trap "exit" INT TERM
# trap "kill 0" EXIT

filename="$1"

function runTest {
	file_name="$1"
	echo "file_name: "
	echo $file_name
	# roslaunch kobuki_gazebo kobuki_empty_world.launch world_file:=$file_name & ./setup/subcommand_without_button.sh #; PID=$!; echo $PID; [ $? -gt 0 ] && kill -9 $PID) &
	roslaunch romi_emulator kobuki_test.launch world_file:=$file_name
	# & ./setup/subcommand_without_button.sh #; PID=$!; echo $PID; [ $? -gt 0 ] && kill -9 $PID) &
	echo "exit status"
	echo $?

 	# (roslaunch kobuki_gazebo kobuki_empty_world.launch world_file:=$file_name; [ "$?" -gt 0 ] && kill "$$"); exit_status2=$? & #(./subcommand.sh; [ "$?" -gt 0 ] && kill "$$"; exit_status1=$?)
	
	# if [ "$exit_status1" -gt "0" ] && [ "$exit_status2" -gt "0" ]; then
	# 	echo "command.sh caught error"
	# 	exit 1
	# else
	# 	echo "command.sh no error"
	# 	exit 0
	# fi
}

runTest "$filename"