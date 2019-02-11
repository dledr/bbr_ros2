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

#include <inttypes.h>
#include <memory>

#include "bbr_sawtooth_bridge/bridge_node.hpp"
#include "bbr_sawtooth_protobuf/proto/Record.pb.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace bbr_sawtooth_bridge
{

Bridge::Bridge(const std::string & node_name)
: rclcpp::Node(node_name)
{
  checkpoint_subscription_ = this->create_subscription<Checkpoint>(
    "_checkpoint", std::bind(&Bridge::checkpoint_callback, this, _1));
  create_record_server_ = this->create_service<CreateRecord>(
    "_create_record", std::bind(&Bridge::create_record_callback, this, _1, _2, _3));

  auto record = Record::proto::Record();
}

void Bridge::create_record_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<CreateRecord::Request> request,
  const std::shared_ptr<CreateRecord::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    this->get_logger(),
    "request: %s", request->name.c_str());
  response->success = true;
}

void Bridge::checkpoint_callback(
    const Checkpoint::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "I heard: %d", msg->stamp);
}

}  // namespace bbr_sawtooth_bridge
