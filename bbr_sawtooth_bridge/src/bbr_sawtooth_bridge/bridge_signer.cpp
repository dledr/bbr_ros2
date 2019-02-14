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

#include <assert.h>
#include <iostream>
#include <inttypes.h>
#include <memory>

#include <secp256k1.h>

#include "bbr_sawtooth_bridge/bridge_signer.hpp"


namespace bbr_sawtooth_bridge
{

Signer::Signer()
{

//  char beef[] = "foo";
//  std::string payload_bytes(beef);
//  Poco::Crypto::DigestEngine sha512("SHA512");
//  sha512.reset();
//  sha512.update(payload_bytes);
//  auto digest = sha512.digest();

//  auto transaction_header = TransactionHeader();
//  transaction_header.set_family_name("intkey");
//  transaction_header.set_family_version("1.0");
//  transaction_header.set_inputs(0, "1cf1266e282c41be5e4254d8820772c5518a2c5a8c0c7f7eda19594a7eb539453e1ed7");
//  transaction_header.set_outputs(0, "1cf1266e282c41be5e4254d8820772c5518a2c5a8c0c7f7eda19594a7eb539453e1ed7");
//  transaction_header.set_signer_public_key("034a2176fedd3945d922a26a83495e5ff9328d55d8c8f0fa432f9f4ff267aebf4a");
//  transaction_header.set_batcher_public_key("034a2176fedd3945d922a26a83495e5ff9328d55d8c8f0fa432f9f4ff267aebf4a");
////  transaction.set_dependencies();
//  transaction_header.set_payload_sha512("sha512(payload_bytes).hexdigest()");
//
//  std::string txn_header_bytes;
//  transaction_header.SerializeToString(&txn_header_bytes);

  unsigned int ALL_FLAGS = SECP256K1_FLAGS_BIT_CONTEXT_SIGN|SECP256K1_CONTEXT_VERIFY;
  secp256k1_context* context = secp256k1_context_create(ALL_FLAGS);

  std::string KEY1_PRIV_HEX = "2f1e7b7a130d7ba9da0068b3bb0ba1d79e7e77110302c9f746c3c2a63fe40088";
  std::string MSG1 = "test";
  std::string MSG1_KEY1_SIG = "5195115d9be2547b720ee74c23dd841842875db6eae1f5da8605b050a49e"
                   "702b4aa83be72ab7e3cb20f17c657011b49f4c8632be2745ba4de79e6aa0"
                   "5da57b35";

  Poco::Crypto::DigestEngine sha256("SHA256");
  sha256.reset();
  sha256.update(MSG1);
  auto digest = sha256.digest();

  std::istringstream keyStream(KEY1_PRIV_HEX.c_str());
  Poco::HexBinaryDecoder hexDnc(keyStream);
  std::string priv_key_str;
//  hexDnc >> priv_key_str;
  int c = hexDnc.get();
  while (c != -1) { priv_key_str += char(c); c = hexDnc.get(); }

  std::ostringstream str;
  Poco::HexBinaryEncoder encoder(str);
  encoder << priv_key_str;
  encoder.close();
  std::cout << "\nKEY1_PRIV_HEX" << "\n";
  std::cout << str.str() << "\n";

  secp256k1_ecdsa_signature *raw_sig = new secp256k1_ecdsa_signature;
  const unsigned char *msg32 = digest.data();
  const unsigned char *private_key = (unsigned char*) priv_key_str.c_str();
  secp256k1_nonce_function nonce_fn = NULL;
  const void *nonce_data = NULL;

  secp256k1_ecdsa_sign(context, raw_sig, msg32, private_key, nonce_fn, nonce_data);

  unsigned char output64[64];
  secp256k1_ecdsa_signature_serialize_compact(context, output64, raw_sig);
//  assert(signed == 1);
  std::string signature((char *)output64, 64);
//  std::string signature((char *)private_key, 64);

  std::ostringstream ostr;
  Poco::HexBinaryEncoder sig_encoder(ostr);
  sig_encoder << signature;
  sig_encoder.close();
  std::cout << "\nMSG1_KEY1_SIG" << "\n";
  std::cout << ostr.str() << "\n";
  
}


}  // namespace bbr_sawtooth_bridge
