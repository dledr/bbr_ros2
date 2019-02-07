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

#include "rosbag2_storage_checkpoint_plugin/checkpoint/checkpoint_helper.hpp"

namespace rosbag2_storage_plugins
{

CheckpointHelper::CheckpointHelper()
{}

std::string CheckpointHelper::createNonce()
{
  char seed[NONCESIZE];
  Poco::RandomInputStream rnd;
  rnd.read(seed, sizeof(seed));
  std::string nonce(seed);

  return nonce;
}

}  // namespace rosbag2_storage_plugins
