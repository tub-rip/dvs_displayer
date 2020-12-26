# dvs_displayer

Introduction to event processing in ROS C++. In this exercise, we start from a minimum working version of a ROS package, called `dvs_displayer`, and will incrementally add new functionalities to it.

## Documentation and video
The goal of this exercise is to get familiarized with ROS (in C++) for event processing. We will learn how to read events, process them to produce some output and visualize it.
We also learn how to use dynamic reconfigure in order to interaction with the program (ROS node) during execution by means of a GUI (Graphical User Interface).

The task consists of building a simple visualizer that can read events and frames from a DAVIS camera (event messages and image messages from two topics) and then displays them as a published image in ROS.

- [Problem statement](https://drive.google.com/file/d/1YJuOIG2zOKnvFksZ7xKJYTK5PKNv4Gf-/view)
- [Video of running example](https://youtu.be/_ksxzPwhE_Y)

## Input / Output
**Input**:
- Events (topic)
- Grayscale images (DAVIS frames) (optional topic)

**Output**:
- Composite image: event image (rendered events) overlaid on a grayscale frame.

**Parameters** (dynamic reconfigure):
- Parameter to select the color map.
- Parameter to enable / disable alpha blending between grayscale image and event image.
- Parameter to control the amount of alpha blending.

### Solution

This repository has approximately one commit per question in the [problem statement](https://drive.google.com/file/d/1YJuOIG2zOKnvFksZ7xKJYTK5PKNv4Gf-/view) (the first 8 commits):
- [Add parameter in launch file to display events in red and blue](https://github.com/tub-rip/dvs_displayer/commit/c66aab349d8b8b2c2bda93e758b55295f9003230)
- [Add subscriber to read grayscale images. Plot red-blue events overlaid](https://github.com/tub-rip/dvs_displayer/commit/1ccf8805a2da77585cc2d608da06f1b1849ee551)
- [Add an OpenCV colormap to visualize events](https://github.com/tub-rip/dvs_displayer/commit/5cb65e46e968547056d22f904a2fee1283ddaecd)
- [Add custom colormap](https://github.com/tub-rip/dvs_displayer/commit/c2adf8ddb7717f31f36580b26ebda91e822546f8)
- [Add first time Dynamic Reconfigure of parameters](https://github.com/tub-rip/dvs_displayer/commit/bef3f3c0131c6fa4d805fa3139d409b4c7fd9545)
- [Dynamic reconfigure of alpha blending](https://github.com/tub-rip/dvs_displayer/commit/ea7d3668e658a544d3bfbf62abe0c08d6d50c33e)


## Dependencies

### Install ROS

- [Installing ROS](http://wiki.ros.org/ROS/Installation)

### Create a catkin workspace

Create a catkin workspace (if there is none yet). For example, from your home folder:

	cd
	mkdir -p catkin_ws/src
	cd catkin_ws
	catkin config --init --mkdirs --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

Depending on the ROS distribution you installed, you might have to use `kinetic` instead of `melodic` in the previous command.

### Add packages to the catkin workspace

Clone this repository into the `src` folder of your catkin workspace.

The catkin package dependencies are:
- [catkin simple](https://github.com/catkin/catkin_simple)
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))

The above dependencies are specified in the [dependencies.yaml](dependencies.yaml) file. They can be installed with the following commands from the `src` folder of your catkin workspace:

	cd catkin_ws/src
	sudo apt-get install python3-vcstool
	vcs-import < dvs_displayer/dependencies.yaml

The previous command should clone the repositories into folders *catkin_simple* and *rpg_dvs_ros* inside the src/ folder of your catkin workspace, at the same level as this repository *dvs_displayer*. They should NOT be inside the *dvs_displayer* folder.

## Compile

**Preliminaries**:
There are instructions in [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) to install the DVS / DAVIS camera driver. Such instructions also guide you through the process of seting up a catkin workspace. You may use the same catkin workspace for the DVS / DAVIS camera driver and this repository.

First, build `catkin simple` and the `davis_ros_driver`. The most important commands are:

	catkin build catkin_simple
	catkin build davis_ros_driver

Then, incorporate the packages to the path, so that ROS finds them:

	source ~/catkin_ws/devel/setup.bash

**Compile this package**:

	catkin build dvs_displayer --force-cmake

The flag `--force-cmake` is optional.
After building, at least the first time, remember to run:

	source ~/catkin_ws/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/dvs_displayer/

An alternative command to start from scratch (cleaning all catkin packages) is (to be used with *caution*): `catkin clean`


### Run example
Download a ROS bag dataset, for example [slider_depth](http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag) to play it in a terminal, that is, to use it as source of data (as if an event camera was connected to the computer).

Every time that you open a terminal, you may need to run:

	source ~/catkin_ws/devel/setup.bash

to have access to the ROS commands.

Run in a terminal (you may need to "source" first):

	roscore

In another terminal play the bag with the event data:

	rosbag play -l path_to_file/slider_depth.bag

Then, in another terminal run the visualizer node:

	roslaunch dvs_displayer display_monocular.launch

In another terminal open the dynamic reconfigure and play around with the parameters in the window named `dvs_displayer_one`

	rosrun rqt_reconfigure rqt_reconfigure

Using the same roscore during execution of multiple runs of the dvs_displayer allows the dynamic parameters to persist even if the nodes are killed. The next time they are started, they configuration parameters will be set using the ones from the previous session.

End the program execution with `Ctrl + C` keyboard shortcut.


## (Optional) Possible extensions:
- Currently we are displaying the events as they are arranged in the ROS messages. Add a parameter that allows to control the rate at which the reconstructed image is published. Use, for example a fixed number of events or a fixed time interval.
- Allow interaction, to change the number of events or the size of the time slice
- Add visualization of time surfaces

## References
- The [RPG dvs_renderer](https://github.com/uzh-rpg/rpg_dvs_ros/tree/master/dvs_renderer) package.
- Gallego et al., *[Event-based Vision: A Survey](https://arxiv.org/pdf/1904.08405)*, IEEE Trans. Pattern Anal. Machine Intell. (TPAMI), 2020. *Section 3.1*
