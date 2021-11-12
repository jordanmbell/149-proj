# Repository for CS149 Project

## Setting up the Simulation

**Sudo pass is Scenicgazebo!**
### Download VMware Workstation 16 Player

Windows: https://www.vmware.com/go/getplayer-win  
Linux: https://www.vmware.com/go/getplayer-linux

### Download the VM:

Link: https://drive.google.com/file/d/1ZCfnf85vl3LoPmM0LkYoq5p_OE5wWfZx/view?usp=sharing

### Setting up VM:

Click 'Open a Virtual Machine'  
Navigate to downloaded ova file  
Set name and storage path to desired  
Double click to launch

### Setting Git up

We need to setup git in the vm so that we can pull the repository and commit the code.  

Setup global git config

    git config --global user.name "John Doe"
    git config --global user.email johndoe@example.com 

Create SSH keys

    ssh-keygen -t ed25519 -C "your_email@example.com"
    # Accept default location, chose pass if desired
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_ed25519

Add SSH key to github:  
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account

    cd ~
    git clone git@github.com:jordanmbell/149-proj.git

### Final steps

The first time we run the simulator, we must set it up.

    cd ~/149-proj/eecs149VirtualLab
    sudo bash setup/setup.sh
    # Sudo pass is Scenicgazebo!
    # Type 'y' when prompted

### Running the Simulator

To start the simulation run:

    cd ~/149-proj/eecs149VirtualLab
    ./run_multi.sh emptpy

To press a button run:

    cd ~/149-proj/eecs149VirtualLab
    ./button_press.sh
    # Type 1-3 when prompted to add to that robot

## Adding Robots

To add additional robots, a few files need changes

###  catkin_ws/src/romi_emulator/src/button_press.cpp

Add additional NodeHandle and Publisher vars for each robot
Add the robot the the if-else block at the end

###  catkin_ws/src/romi_emulator/src/globals.h

Update NUM_ROBOTS

###  catkin_ws/src/romi_emulator/src/eomi_emulator_ros.cpp

Add a processR function for the new robot  
Add a position subscriber for the new robot

###  catkin_ws/src/romi_emulator/launch/kobuki_test.launch

Copy a new \<group/> block for the robot  
Give the group ns="r\<num>"  
Update the "robot_name" arg to have a unique value  
Update x,y, and z to be away from other robots
