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
#include <iostream>
#include <memory>
#include "bbr_sawtooth_bridge/bridge_signer.hpp"

int main(int argc, char * argv[])
{

  std::string KEY1_PRIV_HEX = "2f1e7b7a130d7ba9da0068b3bb0ba1d79e7e77110302c9f746c3c2a63fe40088";
//  std::string KEY1_PUB_HEX = "026a2c795a9776f75464aa3bda3534c3154a6e91b357b1181d3f515110f84b67c5";

  std::string privkey = bbr_sawtooth_bridge::decodeFromHex(KEY1_PRIV_HEX);
  std::cout << "\nKEY1_PRIV_HEX" << "\n";
  std::cout << bbr_sawtooth_bridge::encodeToHex(privkey) << "\n";

  auto signer = bbr_sawtooth_bridge::Signer(privkey);

  std::cout << "\nKEY1_PUB_HEX" << "\n";
  std::cout << bbr_sawtooth_bridge::encodeToHex(signer.pubkey) << "\n";

  std::string MSG1 = "test";
  std::string signature = signer.sign(MSG1);

//  std::string MSG1_KEY1_SIG = "5195115d9be2547b720ee74c23dd841842875db6eae1f5da8605b050a49e"
//                              "702b4aa83be72ab7e3cb20f17c657011b49f4c8632be2745ba4de79e6aa0"
//                              "5da57b35";

  std::cout << "\nMSG1_KEY1_SIG" << "\n";
  std::cout << bbr_sawtooth_bridge::encodeToHex(signature) << "\n";

  return 0;
}
