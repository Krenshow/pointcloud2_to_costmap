# pointcloud2_to_costmap

#### ABOUT

This repository contains pointcloud2_to_costmap node that takes PointCloud2 and decomposes it to classes related to [Class Definitions](https://www.cityscapes-dataset.com/dataset-overview/). Then, each group is sent to its own PointCloud2 topic for the costmap layer ([costmap_2d](https://github.com/ros-planning/navigation/tree/noetic-devel/costmap_2d)).

![test](https://user-images.githubusercontent.com/29725931/134464525-d1a0e4dd-ad29-49eb-8ee1-e777a898e375.gif)

You can configure the segmentation of data to several groups in the _config file_. The number of groups and their composition might be chosen on behalf of the task and related to the data consumer. There is a [_config file_](config/config.yaml) structure example:

```
types:
    - {name:   sidewalk,
       value:  1,
       group:  example_road}

    - {name:   human,
       value:  11,
       group:  example_human}

    - ...
```
* name is a description of the class by [Class Definitions](https://www.cityscapes-dataset.com/dataset-overview/).
* value is the numeric description of the class.
* group is the name of the group you can set by yourself.


#### HOW TO INSTALL

You have to have installed ROS1 (The software was written for ROS Melodic) and contains such packages as [navigation](https://github.com/ros-planning/navigation)[/costmap_2d](https://github.com/ros-planning/navigation/tree/noetic-devel/costmap_2d) and [RViz](http://wiki.ros.org/rviz).

Copy this repository to your cantkin_ws/src directory and do catkin_make command:
```
~ cd catkin_ws/src/
~ git clone https://github.com/Krenshow/pointcloud2_to_costmap.git
~ cd .. && catkin_make
```


#### HOW TO RUN

To run the node you have to run pointcloud2_to_costmap node + run costmap_2d modul + rviz to see the result. As well as the rosbag file or the other source of data has to be started.

pointcloud2_to_costmap node
```
roslaunch pointcloud2_to_costmap pointcloud2_to_costmap.launch 
```
costmap_2d module + rviz
```
roslaunch pointcloud2_to_costmap costmap.launch
```
rosbag file
```
cd path_to_rosbag_file/
rosbag play file_name
```


#### CONFIGURE

To configure the poincloud2_to_costmap node you can edit [config file](config/config.yaml). Also, [params file](config/params.yaml) might be changed to set the costmap layer. You can change observation sources (choose groups of PointCloud2 you want to see at the costmap) or set other parameters of the [costmap](http://wiki.ros.org/costmap_2d).

#### RVIZ

If you run [costmap.launch](launch/costmap.lunch) RViz is started automatically. You should see footprint, costmap and /example_human PointCloud2 data. You can switch on other PointCloud2 groups (topics) just by putting a tick in the boxes in the left corner.


