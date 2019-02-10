# Copyright 2019 Ruffin White.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rclpy.node import Node

from bbr_msgs.msg import Checkpoint
from bbr_msgs.srv import CreateRecord
from std_msgs.msg import String


class Bridge(Node):

    def __init__(self, name):
        super().__init__(name)
        self.checkpoint_subscription = self.create_subscription(
            Checkpoint,
            '_checkpoint',
            self.checkpoint_callback)

        self.create_record_server = self.create_service(
            CreateRecord, '_create_record', self.create_record_callback)
        self.checkpoint_subscription  # prevent unused variable warning

    def checkpoint_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg.stamp))

    def create_record_callback(self, request, response):
        response.success = True
        self.get_logger().info('request: %s' % request.name)
        return response
