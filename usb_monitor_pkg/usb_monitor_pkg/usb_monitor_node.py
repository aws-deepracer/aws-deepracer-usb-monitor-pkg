#!/usr/bin/env python

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
usb_monitor_node.py

This module creates the usb_monitor_node which monitors the connection/disconnection
of the USB drive and provides functionality to watch for specific files/folders in it.
It provides services and functions to subscribe to be notified if a particular file/folder
is found in USB, publisher to broadcast details of the found files/folders and mount
point management functions.

The node defines:
    usb_file_system_notification_publisher: A publisher to broadcast the
                                            notification messages when any of the
                                            files/folders that are in the watchlist are
                                            found in the USB drive.
    usb_file_system_subscriber_service: A service to add files/folders to the watchlist
                                        so that when they are found in the USB drive,
                                        a file system notification will be triggered.
    usb_mount_point_manager_service: A service exposing the functionality to safely
                                     increment/decrement the reference counter for
                                     the mount points.
"""

import os
import threading
import queue
import pyudev
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from deepracer_interfaces_pkg.srv import (USBFileSystemSubscribeSrv,
                                          USBMountPointManagerSrv)
from deepracer_interfaces_pkg.msg import USBFileSystemNotificationMsg
from usb_monitor_pkg import (constants,
                             mount_point_tracker)

#########################################################################################
# USB event monitor.


class USBMonitorNode(Node):
    """Node responsible for managing and monitoring USB connections and notify if a
       watched file/folder is present.
    """

    def __init__(self):
        """Create a USBMonitorNode.
        """
        super().__init__("usb_monitor_node")
        self.get_logger().info("usb_monitor_node started")
        self.queue = queue.Queue()
        self.stop_queue = threading.Event()
        self.mount_point_map = dict()
        self.subscribers = list()

        # Service to add files/folders to the watchlist (subscribers).
        self.file_system_service_cb_group = ReentrantCallbackGroup()
        self.usb_file_system_subscriber_service = \
            self.create_service(USBFileSystemSubscribeSrv,
                                constants.USB_FILE_SYSTEM_SUBSCRIBE_SERVICE_NAME,
                                self.usb_file_system_subscribe_cb,
                                callback_group=self.file_system_service_cb_group)

        # Service exposing the functionality to safely increment/decrement the refernce
        # counter for the mount point.
        self.usb_mpm_cb_group = ReentrantCallbackGroup()
        self.usb_mount_point_manager_service = \
            self.create_service(USBMountPointManagerSrv,
                                constants.USB_MOUNT_POINT_MANAGER_SERVICE_NAME,
                                self.usb_mount_point_manager_cb,
                                callback_group=self.usb_mpm_cb_group)

        # Publisher to broadcast the notification messages.
        self.usb_file_system_notif_cb = ReentrantCallbackGroup()
        self.usb_file_system_notification_publisher = \
            self.create_publisher(USBFileSystemNotificationMsg,
                                  constants.USB_FILE_SYSTEM_NOTIFICATION_TOPIC,
                                  10,
                                  callback_group=self.usb_file_system_notif_cb)

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("USB Monitor node successfully created")

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement.

        Returns:
           USBMonitorNode : self object returned.
        """
        self.thread = threading.Thread(target=self.processor)
        self.thread.start()

        self.pd_context = pyudev.Context()
        try:
            monitor = pyudev.Monitor.from_netlink(self.pd_context)
            monitor.filter_by(subsystem="block")
            self.observer = pyudev.MonitorObserver(monitor, callback=self.block_device_monitor)

        except Exception as ex:
            self.get_logger().info(f"Failed to create UDEV monitor: {ex}")
            self.observer = None

        # Start USB event monitor.
        self.start()
        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.stop()
        self.stop_queue.set()
        self.queue.put(None)
        self.thread.join()

    def usb_mount_point_manager_cb(self, req, res):
        """Callback for the usb_mount_point_manager service. Executes increment/decrement
           actions for the mount point node name passed.

        Args:
            req (USBMountPointManagerSrv.Request): Request object with node_name(str)
                                                   identifying the mount point and the
                                                   action(int) flag.
            res (USBMountPointManagerSrv.Response): Response object with error(int) flag
                                                    to indicate if the service call was
                                                    successful.

        Returns:
            USBMountPointManagerSrv.Response: Response object with error(int) flag to
                                              indicate if the service call was successful.
        """
        node_name = req.node_name
        action = req.action
        res.error = 0
        self.get_logger().info(f"USB mount point manager callback: {node_name} {action}")
        if node_name in self.mount_point_map:
            if action == 0:
                self.mount_point_map[node_name].ref_dec()
            elif action == 1:
                self.mount_point_map[node_name].ref_inc()
            else:
                res.error = 1
                self.get_logger().error(f"Incorrect action value: {action}")
        else:
            res.error = 1
            self.get_logger().error(f"Node name not found: {node_name}")
        return res

    def usb_file_system_subscribe_cb(self, req, res):
        """Callback for the usb_file_system_subscribe service. Adds the file/folder details
           to the subscriber list.

        Args:
            req (USBFileSystemSubscribeSrv.Request): Request object with file_name(str),
                                                     callback_name(str) and
                                                     verify_name_exists(bool) flag to be
                                                     added to the subscriber list.
            res (USBFileSystemSubscribeSrv.Response): Response object with error(int) flag
                                                      to indicate if the service call was
                                                      successful.

        Returns:
            USBFileSystemSubscribeSrv.Response: Response object with error(int) flag to
                                                indicate if the service call was successful.
        """
        self.subscribers.append((req.file_name, req.callback_name, req.verify_name_exists))
        res.error = 0
        self.get_logger().info("USB File system subscription : "
                               f"{req.file_name} "
                               f"{req.callback_name} "
                               f"{req.verify_name_exists}")
        return res

    def processor(self):
        """Main daemon thread that is checking for USB connection and processing its contents.
        """
        while not self.stop_queue.isSet():
            device = self.queue.get()
            if device is None:
                continue

            # Look for block devices.
            if device.subsystem != "block":
                continue

            # Look for partition/disk devices.
            if device.device_type not in ("partition", "disk"):
                continue

            # Look for SCSI disk devices (0-15).
            if device["MAJOR"] != "8":
                continue

            self.get_logger().info(f"Detected file system {device.device_node}")
            self.process(device.device_node)
            self.get_logger().info(f"After processing : {device.device_node}")

    def process(self, filesystem, post_action=None):
        """Helper function to parse the contents of the USB drive and publish notification
           messages for watched files/folders.

        Args:
            filesystem (str): Filesystem path where teh device is mounted.
            post_action (function, optional): Any function that needs to be executed
                                              while decrementing the reference for the
                                              filesystem after completing processing the
                                              file/folder.
                                              Defaults to None.

        Returns:
            bool: True if successfully processed the filesystem else False.
        """
        # Mount the filesystem and get the mount point.
        mount_point = self.get_mount_point(filesystem, post_action)
        if mount_point.name == "":
            return False

        # Check if any of the watched files are present.
        for file_name, callback_name, verify_name_exists in self.subscribers:
            full_name = os.path.join(mount_point.name, file_name)

            if verify_name_exists and \
               (not os.path.isdir(full_name)
                    and not os.path.isfile(full_name)):
                self.get_logger().info("verify_name_exists : "
                                       f"{full_name} "
                                       f"{os.path.isdir(full_name)} "
                                       f"{full_name}")
                continue

            mount_point.ref_inc()
            notification_msg = USBFileSystemNotificationMsg()
            notification_msg.path = mount_point.name
            notification_msg.file_name = file_name
            notification_msg.callback_name = callback_name
            notification_msg.node_name = filesystem
            self.get_logger().info("USB File system notification:"
                                   f" {notification_msg.path}"
                                   f" {notification_msg.file_name}"
                                   f" {notification_msg.node_name}"
                                   f" {notification_msg.callback_name}")

            self.usb_file_system_notification_publisher.publish(notification_msg)

        # Dereference mount point.
        mount_point.ref_dec()
        return True

    def block_device_monitor(self, device):
        """Function callback passed to pyudev.MonitorObserver to monitor USB connection.

        Args:
            device (pyudev.Device): USB Device with attached attributes and properties.
        """
        # Only care about new insertions.
        if device.action == "add":
            self.queue.put(device)

    def start(self):
        """Initializing function to start monitoring.
        """
        # Enumerate and process already connected media before enabling the monitor.
        for device in self.pd_context.list_devices(subsystem="block"):
            self.queue.put(device)

        if self.observer is not None:
            self.observer.start()

    def stop(self):
        """Function to stop monitoring.
        """
        if self.observer is not None:
            self.observer.stop()

    def get_mount_point(self, node_name, post_action=None):
        """Return a MountPoint object from the cache or create one if it does not exist.

        Args:
            node_name (str): Name of the node mounted.
            post_action (function, optional): Optional post action function.
                                              Defaults to None.

        Returns:
            MountPoint: Mount point tracker object.
        """
        try:
            mount_point = self.mount_point_map[node_name]
            if not mount_point.is_valid():
                raise Exception("Not a valid mount point.")

            self.get_logger().info(f"{node_name} is previosuly mounted")
            mount_point.ref_inc()
            mount_point.add_post_action(post_action)

        except Exception:
            self.get_logger().info(f"Creating a mount point for {node_name}")
            mount_point = mount_point_tracker.MountPoint(node_name,
                                                         self.get_logger(),
                                                         post_action)
            self.mount_point_map[node_name] = mount_point

        return mount_point


def main(args=None):
    rclpy.init(args=args)
    with USBMonitorNode() as usb_monitor_node:
        executor = MultiThreadedExecutor()
        rclpy.spin(usb_monitor_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
    usb_monitor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
