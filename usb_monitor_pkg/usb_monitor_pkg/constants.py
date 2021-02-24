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

import os

USB_FILE_SYSTEM_NOTIFICATION_TOPIC = "usb_file_system_notification"
USB_FILE_SYSTEM_SUBSCRIBE_SERVICE_NAME = "usb_file_system_subscribe"
USB_MOUNT_POINT_MANAGER_SERVICE_NAME = "usb_mount_point_manager"

# Base mount point.
BASE_MOUNT_POINT = os.path.join(os.sep, "mnt", "aws-mount")
