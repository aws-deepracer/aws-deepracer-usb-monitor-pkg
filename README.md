# AWS DeepRacer USB monitor package

## Overview

The AWS DeepRacer USB monitor ROS package creates the `usb_monitor_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the [aws-deepracer-launcher repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node monitors the connection and disconnection of the USB drive and watches for specific files and folders in it. It provides services and functions to subscribe to notify you if a particular file or folder is found on the USB drive, a publisher to broadcast details of the found files and folders, and mount point management functions.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer USB monitor package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `usb_monitor_pkg`. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `usb_monitor_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

* `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `usb_monitor_pkg` on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-usb-monitor-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-usb-monitor-pkg
        rosws update

1. Resolve the `usb_monitor_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-usb-monitor-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `usb_monitor_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-usb-monitor-pkg && colcon build --packages-select usb_monitor_pkg deepracer_interfaces_pkg

## Usage

The `usb_monitor_node` provides basic system-level functionality for the AWS DeepRacer application to work. Although the node is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `usb_monitor_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-usb-monitor-pkg/install/setup.bash

1. Launch the `usb_monitor_node` using the launch script:

        ros2 launch usb_monitor_pkg usb_monitor_pkg_launch.py

## Launch files

The `usb_monitor_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the nodes independently from the core application.

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

## Node details

### `usb_monitor_node`

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`usb_monitor_pkg`/`usb_file_system_notification`|`USBFileSystemNotificationMsg`|Publisher that broadcasts notification messages when any of the files or folders that are in the watchlist are found in the USB drive.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`usb_file_system_subscribe`|`USBFileSystemSubscribeSrv`|Service that is called to add files or folders to the watchlist so that when they are found in the USB drive, they initiate a file system notification.|
|`usb_mount_point_manager`|`USBMountPointManagerSrv`|Service exposing  the functionality to safely increment or decrement the reference counter for the mount points.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)

