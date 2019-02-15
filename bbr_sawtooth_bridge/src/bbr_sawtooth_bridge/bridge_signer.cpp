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

#include "bbr_sawtooth_bridge/bridge_signer.hpp"

#include "Poco/StreamCopier.h"


secp256k1_context const* getCtx()
{
  static std::unique_ptr<secp256k1_context, decltype(&secp256k1_context_destroy)> s_ctx{
      secp256k1_context_create(SECP256K1_CONTEXT_SIGN | SECP256K1_CONTEXT_VERIFY),
      &secp256k1_context_destroy
  };
  return s_ctx.get();
}

namespace bbr_sawtooth_bridge
{


Signer::Signer() {
  context_ = getCtx();
  demo();
}

void Signer::demo(){


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


  std::string KEY1_PRIV_HEX = "2f1e7b7a130d7ba9da0068b3bb0ba1d79e7e77110302c9f746c3c2a63fe40088";
  std::string KEY1_PUB_HEX = "026a2c795a9776f75464aa3bda3534c3154a6e91b357b1181d3f515110f84b67c5";
  std::string MSG1 = "test";
  std::string MSG1_KEY1_SIG = "5195115d9be2547b720ee74c23dd841842875db6eae1f5da8605b050a49e"
                   "702b4aa83be72ab7e3cb20f17c657011b49f4c8632be2745ba4de79e6aa0"
                   "5da57b35";

  std::string privkey_str;
  privkey_str = decodeFromHex(KEY1_PRIV_HEX);
  std::cout << "\nKEY1_PRIV_HEX" << "\n";
  std::cout << encodeToHex(privkey_str) << "\n";

  const unsigned char *privkey_ptr = (unsigned char*) privkey_str.c_str();
  secp256k1_pubkey *pubkey_ptr = new secp256k1_pubkey;
  int created = secp256k1_ec_pubkey_create(
      context_, pubkey_ptr, privkey_ptr);
  assert(created == 1);

  std::array<uint8_t, 33> pubkey_bytes;
  size_t serializedPubkeySize = pubkey_bytes.size();
  secp256k1_ec_pubkey_serialize(
      context_, pubkey_bytes.data(), &serializedPubkeySize, pubkey_ptr, SECP256K1_EC_COMPRESSED);
  std::string pubkey_str((char*) pubkey_bytes.data());

  std::cout << "\nKEY1_PUB_HEX" << "\n";
  std::cout << encodeToHex(pubkey_str) << "\n";


  Poco::Crypto::DigestEngine sha256("SHA256");
  sha256.reset();
  sha256.update(MSG1);
  auto digest = sha256.digest();

  secp256k1_ecdsa_signature *raw_sig = new secp256k1_ecdsa_signature;
  const unsigned char *msg32 = digest.data();
  secp256k1_nonce_function nonce_fn = NULL;
  const void *nonce_data = NULL;

  unsigned char output64[64];
  secp256k1_ecdsa_sign(
      context_, raw_sig, msg32, privkey_ptr, nonce_fn, nonce_data);
  secp256k1_ecdsa_signature_serialize_compact(
      context_, output64, raw_sig);
  std::string signature((char *)output64, 64);

  std::string signature_hex;
  signature_hex = encodeToHex(signature);
  std::cout << "\nMSG1_KEY1_SIG" << "\n";
  std::cout << signature_hex << "\n";
  
}

std::string Signer::encodeToHex(const std::string& str)
{
  std::istringstream source(str);
  std::ostringstream sink;

  Poco::HexBinaryEncoder encoder(sink);
  Poco::StreamCopier::copyStream(source, encoder);
  encoder.close();

  return sink.str();
}

std::string Signer::decodeFromHex(const std::string& str)
{
  std::istringstream source(str);
  std::ostringstream sink;

  Poco::HexBinaryDecoder decoder(source);
  Poco::StreamCopier::copyStream(decoder, sink);

  return sink.str();
}

}  // namespace bbr_sawtooth_bridge
