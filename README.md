# This Install Guid

## Installation

To install, use

	$ sudo apt-get install python-wstool	

or when that is not possible, fall back to pip:

	$ sudo pip install -U wstool	

## Create a catkin Workspace with wstool
	$ mkdir ~/ros_catkin_ws 	

    $ cd ~/ros_catkin_ws 

## Initialize the Workspace from a rosinstall File

    $ wstool init src 

add the ```final_project.rosinstall``` file to add to the workspace, proceed to Merge in Additional rosinstall Files below. 

cd to the src folder
    cd src

    wstool merge final_project.rosinstall

## Updating the Workspace

    wstool up

This will update/download all the repositories


## Running The Project


Go up a directorry and build the project to ```ros_catkin_ws```

    cd ..
    catkin_make

sorce the devel 😈

    source devel/setup.bash

run the launch file

    roslaunch dummy_planner go_to_goal.launch 

![](./image.png)

![Alt text](image.png?raw=false "Title")

![alt text](https://gitlab.com/utahstate/20-fall-mobile-robotics/student_groups/allred_christopher/final_project/-/blob/master/image.png?raw=true)

![alt text](https://gitlab.com/utahstate/20-fall-mobile-robotics/student_groups/allred_christopher/final_project/-/blob/master/image.png)
