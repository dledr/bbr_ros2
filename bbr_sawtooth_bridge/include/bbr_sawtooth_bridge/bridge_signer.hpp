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

#ifndef BBR_SAWTOOTH_BRIDGE__BBR__SIGNER_HPP_
#define BBR_SAWTOOTH_BRIDGE__BBR__SIGNER_HPP_

#include <secp256k1.h>

#include "bbr_sawtooth_protobuf/proto/transaction.pb.h"

#include "Poco/Crypto/DigestEngine.h"


namespace bbr_sawtooth_bridge
{

std::string encodeToHex(const std::string& str);
std::string decodeFromHex(const std::string& str);

class Signer
{
 public:
  Signer(std::string privkey_str);

  std::string sign(const std::string& message);

  std::string privkey;
  std::string pubkey;

  secp256k1_context const* context_;
};

}  // namespace bbr_sawtooth_bridge

#endif  // BBR_SAWTOOTH_BRIDGE__BBR__SIGNER_HPP_
