#!/bin/bash

# trap 'exit' INT TERM
# trap 'kill 0' EXIT
trap 'kill -INT -$pid' INT

cd catkin_ws/
catkin_make || exit -1
source devel/setup.sh
cd ..
rm -r ~/.ros/log/*

test_type=$1
test_num="$(($2-1))"

if [ "$test_type" == "ramp" ]; then
	TEST_FILES=(~/eecs149VirtualLab/tests_and_configs/RampTests/*.world)
	TEST_FOLDERS=(~/eecs149VirtualLab/tests_and_configs/ramp/Test*)
elif [ "$test_type" == "obs" ]; then
	TEST_FILES=(~/eecs149VirtualLab/tests_and_configs/ObsTests/*.world)
	TEST_FOLDERS=(~/eecs149VirtualLab/tests_and_configs/obstacle_avoidance/Test*)
elif [ "$test_type" == "empty" ]; then
	TEST_FILES=(~/eecs149VirtualLab/tests_and_configs/EmptyWorld/*.world)
	test_num=0
else
	echo "ERROR: USER DID NOT PROVIDE A PROPER TEST TYPE. PROGRAM EXITING..."
	echo "A proper command execution format:   ./run_tesh.sh  TEST_TYPE  TEST_NUMBER"
	exit 1
fi

killall -w -9 gazebo & killall -w -9 gzserver & killall -w -9 gzclient & 
killall -w -9 rosmaster & killall -w -9 robot_state_publisher & killall -w -9 rosout &
killall -w -9 nodelet

timeout -s SIGKILL 120s bash ./setup/command_with_button.sh ${TEST_FILES[$test_num]} &
pid=$!
wait

killall -w -9 gazebo & killall -w -9 gzserver & killall -w -9 gzclient & 
killall -w -9 rosmaster & killall -w -9 robot_state_publisher & killall -w -9 rosout &
killall -w -9 nodelet

# latest_log=`ls ~/.ros/log/*.log -t | head -1`
latest_folder=`ls ~/.ros/log -tr | head -1`
echo "LATEST LOG FOLDER"
echo $latest_folder
cp ~/.ros/log/$latest_folder/rosout.log ${TEST_FOLDERS[$test_num]}/simulation.log

echo "Test File: "
echo ${TEST_FILES[$test_num]}

echo "Test Config:"
echo ${TEST_FOLDERS[$test_num]}/*.config

source ./setup/activate_conda.sh
conda activate monitors
python monitors/main.py ${TEST_FOLDERS[$test_num]}/simulation.log ${TEST_FOLDERS[$test_num]}/*.config --log_type cpp
conda deactivate

echo "RAMP TESTS ARE COMPLETED"
