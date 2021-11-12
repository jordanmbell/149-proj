#!/bin/bash

# Change the publish rate of the kobuki
cp setup/robot.launch.xml /opt/ros/indigo/share/kobuki_gazebo/launch/includes/
cp setup/.bashrc ~/.bashrc

# Create conda env for monitor
source ./setup/activate_conda.sh
conda install -c anaconda python=3.8
conda create -n monitors python=3.8
conda activate monitors
pip install metric-temporal-logic datetime argparse
conda deactivate monitors

source ~/.bashrc

# build catkin workspace
# cd catkin_ws
# catkin_make
# cd ..

# source ~/.bashrc
