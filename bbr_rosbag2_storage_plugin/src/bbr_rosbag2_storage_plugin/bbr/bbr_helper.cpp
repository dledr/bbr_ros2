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

#include <iostream>
#include <sstream>

#include "rosbag2_storage/ros_helper.hpp"

#include "Poco/HMACEngine.h"
#include "Poco/MD5Engine.h"
#include "Poco/DigestStream.h"
#include "Poco/RandomStream.h"
#include "Poco/StreamCopier.h"

#include "bbr_protobuf/proto/bbr/hash.pb.h"

class SHA256Engine
  : public Poco::Crypto::DigestEngine
{
public:
  enum
  {
    BLOCK_SIZE = 64,
    DIGEST_SIZE = 32
  };

  SHA256Engine()
  : DigestEngine("SHA256")
  {}
};

namespace rosbag2_storage_plugins
{

BbrHelper::BbrHelper()
{}

std::shared_ptr<rcutils_uint8_array_t> BbrHelper::createNonce()
{
  char nonce[SHA256Engine::DIGEST_SIZE];
  Poco::RandomInputStream rnd;
  rnd.read(nonce, SHA256Engine::DIGEST_SIZE);
//  char seed[] = { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
//                  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
//                  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
//                  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,};
//  std::string str_nonce(nonce);
//  std::cout << "Nonce: " << str_nonce;

  return rosbag2_storage::make_serialized_message(nonce, SHA256Engine::DIGEST_SIZE);
}


std::shared_ptr<rcutils_uint8_array_t> BbrHelper::computeTopicDigest(
  std::shared_ptr<rcutils_uint8_array_t> nonce,
  const rosbag2_storage::TopicMetadata & topic)
{
  std::string nonce_passphrase(
    reinterpret_cast<char *>(nonce->buffer),
    nonce->buffer_length);

  std::string topic_format_str;
  auto topic_format = TopicFormat();
  topic_format.set_type(topic.type);
  topic_format.set_serialization_format(topic.serialization_format);
  topic_format.SerializeToString(&topic_format_str);

  std::istringstream topic_format_istr(topic_format_str);
  auto topic_digest = computeHMAC(nonce_passphrase, topic_format_istr);

  std::string topic_passphrase(
    reinterpret_cast<char *>(topic_digest.data()),
    topic_digest.size());

  char * hash = reinterpret_cast<char *>(topic_digest.data());
  return rosbag2_storage::make_serialized_message(hash, SHA256Engine::DIGEST_SIZE);
}

std::shared_ptr<rcutils_uint8_array_t> BbrHelper::computeTopicNonce(
  std::shared_ptr<rcutils_uint8_array_t> nonce,
  const rosbag2_storage::TopicMetadata & topic)
{
  std::string topic_passphrase(
    reinterpret_cast<char *>(nonce->buffer),
    nonce->buffer_length);

  std::string topic_info_str;
  auto topic_info = TopicInfo();
  topic_info.set_name(topic.name);
  topic_info.SerializeToString(&topic_info_str);

  std::istringstream topic_info_istr(topic_info_str);
  auto topic_nonce = computeHMAC(topic_passphrase, topic_info_istr);

  char * hash = reinterpret_cast<char *>(topic_nonce.data());
  return rosbag2_storage::make_serialized_message(hash, SHA256Engine::DIGEST_SIZE);
}


std::shared_ptr<rcutils_uint8_array_t> BbrHelper::computeMessageDigest(
  std::shared_ptr<rcutils_uint8_array_t> nonce,
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  std::string message_passphrase(
    reinterpret_cast<char *>(nonce->buffer),
    nonce->buffer_length);

  std::string message_info_str;
  auto message_info = MessageInfo();
  message_info.set_stamp(message->time_stamp);
  message_info.SerializeToString(&message_info_str);

  std::string message_data_str(
    reinterpret_cast<char *>(message->serialized_data->buffer),
    message->serialized_data->buffer_length);

  std::istringstream topic_info_istr(message_info_str);
  std::istringstream topic_data_istr(message_data_str);
  auto message_digest = computeHMAC(message_passphrase, topic_info_istr, topic_data_istr);

  char * hash = reinterpret_cast<char *>(message_digest.data());
  return rosbag2_storage::make_serialized_message(hash, SHA256Engine::DIGEST_SIZE);
}

Poco::DigestEngine::Digest BbrHelper::computeHMAC(
  std::string passphrase,
  std::istringstream & istr)
{
  //TODO: rework to allow utilize protobuf SerializeToOstream
  Poco::HMACEngine<SHA256Engine> hmac(passphrase);
  Poco::DigestOutputStream dos(hmac);
  Poco::StreamCopier::copyStream(istr, dos);
  dos.close();
  return hmac.digest();
}

Poco::DigestEngine::Digest BbrHelper::computeHMAC(
  std::string passphrase,
  std::istringstream & istr1,
  std::istringstream & istr2)
{
  //TODO: rework to allow utilize protobuf SerializeToOstream
  Poco::HMACEngine<SHA256Engine> hmac(passphrase);
  Poco::DigestOutputStream dos(hmac);
  Poco::StreamCopier::copyStream(istr1, dos);
  Poco::StreamCopier::copyStream(istr2, dos);
  dos.close();
  return hmac.digest();
}

}  // namespace rosbag2_storage_plugins
