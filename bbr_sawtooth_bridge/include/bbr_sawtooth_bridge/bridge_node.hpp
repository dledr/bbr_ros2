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

#include "bbr_msgs/msg/checkpoint.hpp"
#include "bbr_msgs/srv/create_record.hpp"
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
  explicit Bridge(const std::string & node_name);
  ~Bridge() override = default;

  void create_record_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<bbr_msgs::srv::CreateRecord::Request> request,
    const std::shared_ptr<bbr_msgs::srv::CreateRecord::Response> response);

  void checkpoint_callback(
      const bbr_msgs::msg::Checkpoint::SharedPtr msg);

 private:
  rclcpp::Subscription<bbr_msgs::msg::Checkpoint>::SharedPtr checkpoint_subscription_;
  rclcpp::Service<bbr_msgs::srv::CreateRecord>::SharedPtr create_record_server_;
  std::shared_ptr<Signer> signer_;
};

}  // namespace bbr_sawtooth_bridge

#endif  // BBR_SAWTOOTH_BRIDGE__BBR__NODE_HPP_
