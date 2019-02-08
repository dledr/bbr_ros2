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

#include "rosbag2_storage_bbr_plugin/bbr/bbr_node.hpp"


namespace rosbag2_storage_plugins
{

BbrNode::BbrNode(const std::string & node_name)
    : rclcpp::Node(node_name), count_(0)
{
  publisher_ = this->create_publisher<bbr_msgs::msg::Checkpoint>("_checkpoint");
}

void BbrNode::publish_checkpoint(
    std::shared_ptr<rcutils_uint8_array_t> hash,
    std::shared_ptr<rcutils_uint8_array_t> nonce,
    std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  auto msg = bbr_msgs::msg::Checkpoint();
  msg.stamp = rclcpp::Time(message->time_stamp);
  msg.hash.data = std::vector<uint8_t>(hash->buffer, hash->buffer + hash->buffer_length);
  msg.nonce.data = std::vector<uint8_t>(nonce->buffer, nonce->buffer + nonce->buffer_length);
  RCLCPP_INFO(this->get_logger(), "Publishing checkpoint: '%s'", message->topic_name.c_str());
  publisher_->publish(msg);
}

}  // namespace rosbag2_storage_plugins
