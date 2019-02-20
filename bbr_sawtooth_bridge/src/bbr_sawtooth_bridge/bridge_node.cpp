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

#include "bbr_protobuf/proto/sawtooth/batch.pb.h"
#include "bbr_protobuf/proto/sawtooth/property.pb.h"
#include "bbr_protobuf/proto/sawtooth/record.pb.h"
#include "bbr_protobuf/proto/sawtooth/transaction.pb.h"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace bbr_sawtooth_bridge
{

Bridge::Bridge(
    const std::string & node_name,
    const std::string & signer_privkey,
    const std::string & batcher_privkey)
: rclcpp::Node(node_name),
  batcher_(),
  signer_(),
  deigest_engine_()
{
  checkpoint_subscription_ = this->create_subscription<bbr_msgs::msg::Checkpoint>(
    "_checkpoint", std::bind(&Bridge::checkpoint_callback, this, _1));
  create_record_server_ = this->create_service<bbr_msgs::srv::CreateRecord>(
    "_create_record", std::bind(&Bridge::create_record_callback, this, _1, _2, _3));
  signer_ = std::make_shared<Signer>(signer_privkey);
  batcher_ = std::make_shared<Signer>(batcher_privkey);
  deigest_engine_ = std::make_shared<Poco::Crypto::DigestEngine>("SHA512");
}

void Bridge::create_record_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<bbr_msgs::srv::CreateRecord::Request> request,
  const std::shared_ptr<bbr_msgs::srv::CreateRecord::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    this->get_logger(),
    "request: %s", request->name.c_str());
  response->success = true;
}

void Bridge::checkpoint_callback(
    const bbr_msgs::msg::Checkpoint::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "I heard: %d", msg->stamp);

//  auto property = Property();
//  auto associated_agent = Record::AssociatedAgent();
//  associated_agent.set_agent_id(signer_->pubkey_str);
//  associated_agent.set_timestamp(msg->stamp);
//  auto record = Record();

  std::string txn_payload("foo");

  auto txn_header = TransactionHeader();
//  TODO: Update family_name to "bbr"
  txn_header.set_family_name("intkey");
  txn_header.set_family_version("1.0");
  auto input = txn_header.add_inputs();
  input->assign("1cf1266e282c41be5e4254d8820772c5518a2c5a8c0c7f7eda19594a7eb539453e1ed7");

  auto output = txn_header.add_outputs();
  output->assign("1cf1266e282c41be5e4254d8820772c5518a2c5a8c0c7f7eda19594a7eb539453e1ed7");

  txn_header.set_signer_public_key(signer_->pubkey_str);
  txn_header.set_batcher_public_key(batcher_->pubkey_str);
//  transaction.set_dependencies();

  deigest_engine_->reset();
  deigest_engine_->update(txn_payload);
  auto digest = deigest_engine_->digest();
  txn_header.set_payload_sha512(
      Poco::DigestEngine::digestToHex(digest));


  BatchList batch_list;
  auto batch = batch_list.add_batches();
  auto transaction = batch->add_transactions();

  std::string txn_header_bytes;
  txn_header.SerializeToString(&txn_header_bytes);
  auto txn_header_signature = encodeToHex(signer_->sign(txn_header_bytes));

  transaction->set_header(txn_header_bytes);
  transaction->set_header_signature(txn_header_signature);
  transaction->set_payload(txn_payload);

  BatchHeader batch_header;
  batch_header.set_signer_public_key(batcher_->pubkey_str);
  auto transaction_id = batch_header.add_transaction_ids();
  transaction_id->assign(transaction->header_signature());

  std::string batch_header_bytes;
  batch_header.SerializeToString(&batch_header_bytes);
  auto batch_header_signature = encodeToHex(batcher_->sign(batch_header_bytes));

  batch->set_header(batch_header_bytes);
  batch->set_header_signature(batch_header_signature);

  std::string batch_list_bytes;
  batch_list.SerializeToString(&batch_list_bytes);

}

}  // namespace bbr_sawtooth_bridge
