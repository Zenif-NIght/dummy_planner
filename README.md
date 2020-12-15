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

     wstool merge -t src final_project.rosinstall

## Updating the Workspace

    wstool up

This will update/download all the repositories