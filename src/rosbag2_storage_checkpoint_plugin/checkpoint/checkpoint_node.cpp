// Copyright 2019, Ruffin White.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosbag2_storage_default_plugins/checkpoint/checkpoint_node.hpp"


namespace rosbag2_storage_plugins
{

CheckpointNode::CheckpointNode(const std::string & node_name)
    : rclcpp::Node(node_name), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("_checkpoint");
}

void CheckpointNode::publish_checkpoint(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  auto msg = std_msgs::msg::String();
//  message.data = "Hello, world! " + std::to_string(count_++);
  msg.data = "Topic: " + message->topic_name + " Time: " + std::to_string(message->time_stamp);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
  publisher_->publish(msg);
}

}  // namespace rosbag2_storage_plugins
