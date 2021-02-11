# SpikeCoding


Install ROS Noetic (for Ubuntu Focal 20.04 LTS) using the following tutorial: <br>http://wiki.ros.org/noetic/Installation/Ubuntu 

Install *catkin* python tools: 

	sudo apt-get install python3-catkin-tools 

Then create your workspace:

	cd~ 
	mkdir -p /catkin_ws/src
	cd /catkin/src
	git clone https://github.com/catkin/catkin_simple.git
	cd ..
	catkin_make
	source ~/catkin_ws/devel/setup.bash

Download this repository, and copy the *spike_encoder* package to your `~/catkin_ws/src/` folder. Then compile your project: 

	cd /catkin/
	catkin_make
	source ~/catkin_ws/devel/setup.bash
