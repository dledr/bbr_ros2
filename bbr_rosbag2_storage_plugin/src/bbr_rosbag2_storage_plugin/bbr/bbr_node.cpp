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

#include "bbr_rosbag2_storage_plugin/bbr/bbr_node.hpp"


namespace rosbag2_storage_plugins
{

BbrNode::BbrNode(const std::string & node_name)
: rclcpp::Node(node_name)
{
  checkpoints_publisher_ =
    this->create_publisher<bbr_msgs::msg::CheckpointArray>("checkpoints", 10);
  records_client_ = this->create_client<bbr_msgs::srv::CreateRecords>("create_records");
  while (!records_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(), "record client interrupted while waiting for service to appear.");
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
  rcutils_time_point_value_t time_stamp;
  int error = rcutils_system_time_now(&time_stamp);
  if (error != RCUTILS_RET_OK) {
    throw std::runtime_error(
            "Error getting current time.");
  }

  auto checkpoint = bbr_msgs::msg::Checkpoint();
  checkpoint.hash.data = std::vector<uint8_t>(
    nonce->buffer, nonce->buffer + nonce->buffer_length);
  checkpoint.stamp = time_stamp;

  auto checkpoint_array = bbr_msgs::msg::CheckpointArray();
  checkpoint_array.checkpoints.push_back(checkpoint);
  checkpoint_array.uid.data = checkpoint.hash.data;

  auto record = bbr_msgs::msg::Record();
  record.checkpoint_array = checkpoint_array;
  record.topic_name = topic.name;
  record.message_type = topic.type;
  record.serialization_format = topic.serialization_format;

  auto record_array = bbr_msgs::msg::RecordArray();
  record_array.records.push_back(record);

  auto request = std::make_shared<bbr_msgs::srv::CreateRecords::Request>();
  request->record_array = record_array;

  auto result_future = records_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "record call failed: '%s'", topic.name.c_str());
    throw std::runtime_error(
            "Failed to confirm creation from record service.");
  }
  auto result = result_future.get();
  if (not result->success) {
    RCLCPP_ERROR(this->get_logger(), "record was not created: '%s'", topic.name.c_str());
    throw std::runtime_error(
            "Failed to create record from record service.");
  }
}

void BbrNode::publish_checkpoint(
  std::shared_ptr<rcutils_uint8_array_t> nonce,
  std::shared_ptr<rcutils_uint8_array_t> hash,
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  auto msg = bbr_msgs::msg::CheckpointArray();
  // msg.stamp = rclcpp::Time(message->time_stamp);
  // FIXME: This time_stamp may be younger than gensius stamp from create_record

  auto checkpoint = bbr_msgs::msg::Checkpoint();
  checkpoint.hash.data = std::vector<uint8_t>(
    hash->buffer, hash->buffer + hash->buffer_length);
  checkpoint.stamp = message->time_stamp;

  auto checkpoint_array = bbr_msgs::msg::CheckpointArray();
  checkpoint_array.checkpoints.push_back(checkpoint);
  checkpoint_array.uid.data = std::vector<uint8_t>(
    nonce->buffer, nonce->buffer + nonce->buffer_length);

  RCLCPP_INFO(this->get_logger(), "Publishing checkpoint: '%s'", message->topic_name.c_str());
  checkpoints_publisher_->publish(checkpoint_array);
}

}  // namespace rosbag2_storage_plugins
