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

#include "bbr_rosbag2_storage_plugin/bbr/bbr_helper.hpp"

//#include <iostream>

#include "rosbag2_storage/ros_helper.hpp"

namespace rosbag2_storage_plugins
{

BbrHelper::BbrHelper()
{
  deigest_engine_ = std::make_shared<Poco::Crypto::DigestEngine>(DIGEST_ENGINE_NAME);
}

std::shared_ptr<rcutils_uint8_array_t> BbrHelper::createNonce()
{
  char nonce[NONCE_SIZE];
  Poco::RandomInputStream rnd;
  rnd.read(nonce, NONCE_SIZE);
//  char seed[] = { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
//                  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
//                  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
//                  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,};
//  std::string str_nonce(nonce);
//  std::cout << "Nonce: " << str_nonce;

  return rosbag2_storage::make_serialized_message(nonce, NONCE_SIZE);;
}


std::shared_ptr<rcutils_uint8_array_t> BbrHelper::computeGenesis(
    std::shared_ptr<rcutils_uint8_array_t> nonce,
    const rosbag2_storage::TopicMetadata & topic)
{
//  TODO: Properly hash a list of multiple items, e.g via a hash list
//  https://en.wikipedia.org/wiki/Hash_list
//  https://crypto.stackexchange.com/questions/10058/how-to-hash-a-list-of-multiple-items
  deigest_engine_->reset();
  deigest_engine_->update(nonce->buffer, nonce->buffer_length);
  deigest_engine_->update(topic.name);
  deigest_engine_->update(topic.type);
  deigest_engine_->update(topic.serialization_format);
  Poco::DigestEngine::Digest digest = deigest_engine_->digest();
  char* hash = reinterpret_cast<char*>(digest.data());

  return rosbag2_storage::make_serialized_message(hash, NONCE_SIZE);;
}


std::shared_ptr<rcutils_uint8_array_t> BbrHelper::computeHash(
    std::shared_ptr<rcutils_uint8_array_t> nonce,
    std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
//  TODO: Properly hash a list of multiple items, e.g via a hash list
  deigest_engine_->reset();
  deigest_engine_->update(nonce->buffer, nonce->buffer_length);
  deigest_engine_->update(message->time_stamp);
  deigest_engine_->update(message->serialized_data->buffer, message->serialized_data->buffer_length);
  Poco::DigestEngine::Digest digest = deigest_engine_->digest();
  char* hash = reinterpret_cast<char*>(digest.data());

  return rosbag2_storage::make_serialized_message(hash, NONCE_SIZE);;
}

}  // namespace rosbag2_storage_plugins
