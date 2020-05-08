# dvs_displayer

Bare minimum event processing in ROS C++.

At this point, the visualizer only reads event messages and displays them as a (published) grayscale image.

### Dependencies

Requires [catkin_simple](https://github.com/catkin/catkin_simple) and [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) driver

### Compile

**Preliminaries**:
First, build `catkin simple` and the `davis_ros_driver`. The most important commands are:

	catkin build catkin_simple
	catkin build davis_ros_driver

Then, incorporate the packages to the path, so that ROS finds them:
	
	source ~/ros/catkin_ws_evis/devel/setup.bash
	
**Compile this package**:
	
	catkin build dvs_displayer --force-cmake
	
The flag `--force-cmake` is optional.	
After building, at least the first time, remember to run:

	source ~/ros/catkin_ws_evis/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/dvs_displayer/
	
An alternative command to start from scratch (cleaning all catkin pakages) is (to be used with *caution*): `catkin clean`


### Run example
Download a ROS bag dataset, for example [slider_depth](http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag) to play it in a terminal, that is, to use it as source of data (as if an event camera was connected to the computer).

Every time that you open a terminal, you may need to run:

	source ~/ros/catkin_ws_evis/devel/setup.bash

to have access to the ROS commands.

Run in a terminal (you may need to "source" first):

	roscore
	
In another terminal play the bag with the event data:

	rosbag play -l path_to_file/slider_depth.bag
	
Then, in another terminal play run the visualizer node:
	
	roslaunch dvs_displayer display_monocular.launch

End the program execution with `Ctrl + C` keyboard shortcut. 