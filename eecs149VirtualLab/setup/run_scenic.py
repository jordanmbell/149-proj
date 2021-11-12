#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
import scenic
import scenic.simulators.gazebo.interface as interface
import os


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('-l', '--host', default='localhost', help='The IP where DeepGTAV is running')
    parser.add_argument('-p', '--port', default=8000, help='The port where DeepGTAV is running')
    parser.add_argument('-d', '--dataset_path', default='dataset.pz', help='Place to store the dataset')
    parser.add_argument('-i', '--index', default='5')
    parser.add_argument('-m', '--l_r', default='0')
    parser.add_argument('-n', '--f_b', default='10')
    parser.add_argument('-end', '--end', default='100')
    parser.add_argument('-start', '--start', default='0')
    parser.add_argument('-sc', '--sc_file', default='examples/gazebo/test.sc')
    parser.add_argument('-world', '--outFile', default='examples/gazebo/outputs/out.world')
    parser.add_argument('-template', '--world_template', default='basic_world')
    args = parser.parse_args()

    # name = "examples/gazebo/simpleBox.sc"
    scenic_source_file = args.sc_file
    outFile = args.outFile

    # construct scenario
    scenario = scenic.scenarioFromFile(scenic_source_file)
    scene, _ = scenario.generate()

    print("scene generated: ", scene)

    """ 
    Generate about 10 ~ 20 scenes & create corresponding world files 
    For each scene, log the variables varied 
    
    """

    my_cfg = interface.Gazebo.parse(scenario)
    print(my_cfg)

    interface.Gazebo.write(scenario)

    output = interface.Gazebo.config(scene, "basic_world")

    with open(outFile, 'w+') as fileObj:
        fileObj.write(output)