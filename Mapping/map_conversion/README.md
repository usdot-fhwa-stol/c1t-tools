# SLAM map to PCD conversion
This repository contains multiple ROS2 packages and some Python scripts to facilitate converting from a SLAM map to a PCD file for use with the CARMA 1tenth scaled-down cooperative driving automation program.

The ROS2-based Autoware.auto does not appear to include the **NDT Mapping** from Autoware.ai.
Additionally, stakeholders may desire to generate their HD maps using a non-C1T vehicle (or a C1T vehicle running some remote-controlled driving platform) and need a solution to generate the PCD maps that are required for **NDT Matching** and localization in the C1T HD Map and scaled-down world.

## Assumptions and Limitations

### Assumptions
The code in this repository assumes that SLAM has already been used to generate an Occupancy Grid of the test environment that has been mapped, and that the map that was made was recorded in a ros2 bag file (.db3).

The currently supported ROS2-SLAM library is [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and can be used for Occupancy Grid map generation.

### Limitations
A key component of this process to convert from an occupancy grid to a PCD file is the [pcl_ros](http://wiki.ros.org/pcl_ros) ROS2 package.

The pcl_ros package includes a [bag_to_pcd](http://wiki.ros.org/pcl_ros#bag_to_pcd) node that converts a PointCloud2 message in a rosbag to a PCD file.

At the time of writing this README, the pcl_ros package has not been ported from ROS to ROS2 - and the time and effort to port would be prohibitive to the CARMA 1tenth program and out of scope. A workaround to convert from a PointCloud2 in a ros2 bag to a PCD exists, where the ros2 bag can first be converted to a rosbag and then pcl_ros can be run in ROS Noetic.

As such: at this time, conversion from occupancy grids to PCD files requires the installation of ROS Noetic and the perception_pcl package.

## Dependent Repos
This repository depends on the **ros2_numpy** package to serialize/deserialize
messages into numpy arrays. You must first clone this package from the [Box-Robotics](https://github.com/Box-Robotics/ros2_numpy) repo and build it with colcon.

The conversion process also depends on the rosbag_converting python package to convert from a ros2 .db3 file to a ros .bag file.
Install this package using pip in the terminal:
```
pip install rosbags
```

## What's within this repository?
This C1T_map_conversion repository includes:

1. A ROS2 package that converts an OccupancyGrid to a PointCloud2
    1. Uses the forked **ros2_numpy** package to serialize/deserialize messages into numpy arrays.
    2. Build this package by navigating to the ./occgrid_to_pcd2 directory and running the following command in the terminal:
    ```
    colcon build --packages-select occupancy_grid_to_pcd2
    ```

The **occupancy_grid_to_pcd2** relies on manipulating numpy arrays, and is computationally expensive and slow.
This package, and the conversion from an OccupancyGrid to a PointCloud2 message should occur offline from the SLAM map-collection effort.
The node is not yet optimized for conversion of specific OccupancyGrid messages, and will seek to convert any OccupancyGrid message published to the /map topic.
As such, current conversion relies on a user "timing" the running of the conversion node to be before the desired OccupancyGrid message.
This issue can be avoided by only recording the desired OccupancyGrid, and not recording many occupancy grids in the same bag.
This would also function to reduce the size of the bag, reducing storage requirements.

## Process
The overall process for PCD map generation is as follows:

0. SLAM OccupancyGrid Map Generation
    1. run using slam_toolbox and record rosbag2
1. OccupancyGrid to PointCloud2 Conversion:
    0. play the rosbag2 by running `ros2 bag play -l <bag_name>` where `<bag_name>` is the path of the rosbag2
    1. convert the SLAM map to a PointCloud2 message by using:
        ```
        ros2 run occupancy_grid_to_pcd2 occ_grid_to_pcd2 
        ```
        This step:
        1. Serializes the OccupancyGrid message into a numpy array using the ros2_numpy package
        2. Converts the OccupancyGrid array to an array of points, by using the resolution and origin to convert occupied grids to points
        3. Deserializes the array of points into a PointCloud2 message using the ros2_numpy package
        4. Publishes the PointCloud2 message of the map of the test environment to a topic and record rosbag2 of the PointCloud2 topic
    2. record the PointCloud2 message with `ros2 bag record <topic_name>` where `<topic_name>` is the name of the topic the PointCloud2 message is being published to
2. Convert the rosbag2 to a rosbag using `rosbags` python package and by running:
    ```
    rosbags-convert <rosbag2_path>
    ```
    in Python where `<rosbag2_path>` is the path of the ros2 bag
3. Use pcl_ros in ROS Noetic to convert the PointCloud2 message to a .pcd file for use as the geometric layer of the HD map. With ROS Noetic installed, run pcl_ros by:

    1. sourcing ROS Noetic in a terminal and then running `roscore`
    2. sourcing ROS Noetic in another terminal and then running `rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>` where:
        - <input_file.bag> is the bag file name to read
        - <topic> is the topic in the bag file containing messages to save
        - <output_directory> is the directory on disk in which to create PCD files from the point cloud messages

    More information on the pcl_ros package can be found in the ros wiki [here](https://wiki.ros.org/pcl_ros#bag_to_pcd)

    If you do not know what topic the PointCloud2 message is published in, find it using `rostopic find sensor_msgs/PointCloud2`