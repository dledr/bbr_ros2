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

#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__BBR__BBR_HELPER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__BBR__BBR_HELPER_HPP_

#include "rosbag2_storage_default_plugins/visibility_control.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "Poco/Crypto/DigestEngine.h"

namespace rosbag2_storage_plugins
{

const size_t NONCE_SIZE = 32;
const std::string DIGEST_ENGINE_NAME = "SHA256";

class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC BbrHelper
{
 public:
  BbrHelper();

  std::shared_ptr<rcutils_uint8_array_t> createNonce();

  std::shared_ptr<rcutils_uint8_array_t> computeTopicDigest(
      std::shared_ptr<rcutils_uint8_array_t> nonce,
      const rosbag2_storage::TopicMetadata &topic);

  std::shared_ptr<rcutils_uint8_array_t> computeTopicNonce(
      std::shared_ptr<rcutils_uint8_array_t> nonce,
      const rosbag2_storage::TopicMetadata &topic);

  std::shared_ptr<rcutils_uint8_array_t> computeMessageDigest(
      std::shared_ptr<rcutils_uint8_array_t> nonce,
      std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message);

 private:

  Poco::DigestEngine::Digest computeHMAC(
      std::string passphrase,
      std::istringstream & istr);
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_DEFAULT_PLUGINS__BBR__BBR_HELPER_HPP_
