# DeepRacer USB Monitor Package

## Overview

The DeepRacer USB Monitor ROS package creates the *usb_monitor_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/awsdeepracer/aws-deepracer-launcher).

This node monitors the connection/disconnection of the USB drive and provides functionality to watch for specific files/folders in it. It provides services and functions to subscribe to be notified if a particular file/folder is found in USB, publisher to broadcast details of the found files/folders and mount point management functions.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the usb_monitor_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The usb_monitor_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

* *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the usb_monitor_pkg on the DeepRacer device:

        git clone https://github.com/awsdeepracer/aws-deepracer-usb-monitor-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-usb-monitor-pkg
        rosws update

1. Resolve the usb_monitor_pkg dependencies:

        cd ~/deepracer_ws/aws-deepracer-usb-monitor-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the usb_monitor_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/aws-deepracer-usb-monitor-pkg && colcon build --packages-select usb_monitor_pkg deepracer_interfaces_pkg

## Usage

The usb_monitor_node provide basic system level functionality for the AWS DeepRacer application to work. Although the node is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built usb_monitor_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-usb-monitor-pkg/install/setup.bash

1. Launch the usb_monitor_node using the launch script:

        ros2 launch usb_monitor_pkg usb_monitor_pkg_launch.py

## Launch Files

The  usb_monitor_pkg_launch.py is also included in this package that gives an example of how to launch the nodes independently from the core application.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='usb_monitor_pkg',
                namespace='usb_monitor_pkg',
                executable='usb_monitor_node',
                name='usb_monitor_node'
            )
        ])

## Node Details

### usb_monitor_node

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/usb_monitor_pkg/usb_file_system_notification|USBFileSystemNotificationMsg|Publisher to broadcast the notification messages when any of the files/folders that are in the watchlist are found in the USB drive.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|usb_file_system_subscribe|USBFileSystemSubscribeSrv|Service that is called to add files/folders to the watchlist so that when they are found in the USB drive, a file system notification will be triggered.|
|usb_mount_point_manager|USBMountPointManagerSrv|Service exposing  the functionality to safely increment/decrement the reference counter for the mount points.|

