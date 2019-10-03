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

#ifndef BBR_SAWTOOTH_BRIDGE__BBR__NODE_HPP_
#define BBR_SAWTOOTH_BRIDGE__BBR__NODE_HPP_

#include <zmqpp/context.hpp>
#include <zmqpp/socket.hpp>

#include "bbr_msgs/msg/checkpoint.hpp"
#include "bbr_msgs/msg/checkpoint_array.hpp"
#include "bbr_msgs/msg/record.hpp"
#include "bbr_msgs/msg/record_array.hpp"
#include "bbr_msgs/srv/create_records.hpp"

#include "bbr_sawtooth_bridge/bridge_signer.hpp"

#include "rclcpp/rclcpp.hpp"

// #include "std_msgs/msg/string.hpp"
// #include "rosbag2_storage/serialized_bag_message.hpp"
// #include "rosbag2_storage/topic_metadata.hpp"

namespace bbr_sawtooth_bridge
{

class Bridge
  : public rclcpp::Node
{
public:
  explicit Bridge(
    const std::string & node_name,
    const std::string & signer_key_path,
    const std::string & batcher_key_path);
  ~Bridge() override = default;

private:
  void checkpoints_callback(
    const bbr_msgs::msg::CheckpointArray::SharedPtr msg);

  void create_records_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<bbr_msgs::srv::CreateRecords::Request> request,
    const std::shared_ptr<bbr_msgs::srv::CreateRecords::Response> response);

  std::string path_to_key(std::string key_path);

  rclcpp::Subscription<bbr_msgs::msg::CheckpointArray>::SharedPtr checkpoints_subscription_;
  rclcpp::Service<bbr_msgs::srv::CreateRecords>::SharedPtr create_records_server_;

  std::shared_ptr<Signer> batcher_;
  std::shared_ptr<Signer> signer_;

  std::shared_ptr<Poco::Crypto::DigestEngine> deigest_engine_;

  zmqpp::context context_;
  zmqpp::socket socket_;
};

}  // namespace bbr_sawtooth_bridge

#endif  // BBR_SAWTOOTH_BRIDGE__BBR__NODE_HPP_
