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
: rclcpp::Node(node_name)
{
  checkpoint_publisher_ = this->create_publisher<bbr_msgs::msg::Checkpoint>("_checkpoint");
  record_client_ = this->create_client<bbr_msgs::srv::CreateRecord>("_create_record");
  while (!record_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "record client interrupted while waiting for service to appear.");
      throw std::runtime_error(
          "Failed to connect to record service.");
    }
    RCLCPP_INFO(this->get_logger(), "waiting for record service to appear...");
  }
}

void BbrNode::create_record(
    std::shared_ptr<rcutils_uint8_array_t> nonce,
    const rosbag2_storage::TopicMetadata & topic)
{
  auto request = std::make_shared<bbr_msgs::srv::CreateRecord::Request>();

  request->name = topic.name;
  request->nonce.data = std::vector<uint8_t>(nonce->buffer, nonce->buffer + nonce->buffer_length);
  request->serialization_format = topic.serialization_format;
  request->stamp = this->now();
  request->type = topic.type;
  auto result_future = record_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "record call failed: '%s'", topic.name.c_str());
    throw std::runtime_error(
        "Failed to confirm creation from record service.");
  }
  auto result = result_future.get();
  if (not result->success){
    RCLCPP_ERROR(this->get_logger(), "record was not created: '%s'", topic.name.c_str());
    throw std::runtime_error(
        "Failed to create record from record service.");
  }
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
  checkpoint_publisher_->publish(msg);
}

}  // namespace rosbag2_storage_plugins
