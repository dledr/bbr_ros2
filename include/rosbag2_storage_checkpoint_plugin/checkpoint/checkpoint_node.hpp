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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__CHECKPOINT__CHECKPOINT_NODE_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__CHECKPOINT__CHECKPOINT_NODE_HPP_

#include "rosbag2_storage_default_plugins/visibility_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"

using namespace std::chrono_literals;

namespace rosbag2_storage_plugins
{

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC CheckpointNode
    : public rclcpp::Node
{
 public:
  explicit CheckpointNode(const std::string & node_name);
  ~CheckpointNode() override = default;

  void publish_checkpoint(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message);

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__CHECKPOINT__CHECKPOINT_NODE_HPP_
