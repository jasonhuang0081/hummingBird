## Introduction

This package contains the necessary rviz and urdf files, plugin, and nodes to run a rviz visualization of the hummingbird hardware.
Currently, this package assumes that you are running Ubuntu 18.04 with the ros-melodic toolchain. Note: this
package will not run on Ubuntu 14.04 with Indigo due to Qt4 vs Qt5 discrepancies.


## Dependencies
- [hb_msgs](https://magiccvs.byu.edu/gitlab/hummingbird/hb_msgs) package: Contains all of the custom hummingbird messages to be used by other packages.

## Setup

If you currently do not have a catkin workspace, start by creating one.

```
mkdir -p <catkin_ws_name>/src
cd <catkin_ws_name>
catkin_make
```

Clone the [hb_msgs](https://magiccvs.byu.edu/gitlab/hummingbird/hb_msgs), and the hb\_viz packages source folder of your catkin workspace, build your workspace, and add your packages to the
ROS path by sourcing the *setup.bash* file.

```
cd src
git clone https://magiccvs.byu.edu/gitlab/hummingbird/hb_msgs.git
git clone https://magiccvs.byu.edu/gitlab/hummingbird/hb_viz.git
cd ..
catkin_make
source devel/setup.bash
```

or if you have keys setup you can run the following

```
cd src
git clone git@magiccvs.byu.edu:hummingbird/hb_msgs.git
git clone git@magiccvs.byu.edu:hummingbird/hb_viz.git
cd ..
catkin_make
source devel/setup.bash
```

## Running the Vizualization

The dynamics project has one ROS launch file.
- dynamics_viz.launch

This launch file will run the visualization node and the dynamics node. To run it

```
roslaunch hb_viz visualize.launch
```

The simulation should now render and you should be able to see an rviz panel in the upper left corner called Hummingbird. You should see adjustable widgets such as sliders, drop down menu, and a button.
The sliders will change the values being published, the drop down menu is used to indicate the topic to publish, and the button indicates if the panel should be enabled or not. Note: if disabled,
topics will no longer be published.


Note: these commands will only bring up rviz and thus, the hummingbird dynamics will not be implemented. To implement the
hummingbird dynamics, either the simulator or the hardware needs to be run.

## Panel
The plugin creates a panel in the rviz gui that allows users to interact with additional widgets. Currently the panel consists of three different layouts which corresponds to
three different topics: 'hb\_states' which uses hb\_msgs::State messages, 'hb\_reference_state' which uses hb\_msgs::ReferenceState messages,
and 'hb\_command' which uses hb\_msgs::Command messages. You can switch between which topics are being published by adjusting the "Layout Type" on the panel. Note that there is also an enable button. Only when enabled will the topics be published and the values be changeable.
