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
#include <sstream>

#include "bbr_sawtooth_bridge/bridge_signer.hpp"

#include "Poco/DigestStream.h"
#include "Poco/HexBinaryDecoder.h"
#include "Poco/HexBinaryEncoder.h"
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


Signer::Signer(const std::string & privkey_str) {
  context_ = getCtx();
  privkey = privkey_str;

  const unsigned char *privkey_ptr = (unsigned char*) privkey.c_str();
  std::unique_ptr<secp256k1_pubkey> pubkey_ptr(new secp256k1_pubkey);
  int pubkey_created = secp256k1_ec_pubkey_create(
      context_, pubkey_ptr.get(), privkey_ptr);
  assert(pubkey_created == 1);


  std::array<uint8_t, 33> pubkey_bytes;
  size_t serializedPubkeySize = pubkey_bytes.size();
  int pubkey_serialize = secp256k1_ec_pubkey_serialize(
      context_, pubkey_bytes.data(), &serializedPubkeySize, pubkey_ptr.get(), SECP256K1_EC_COMPRESSED);
  assert(pubkey_serialize == 1);
  pubkey = std::string((char*) pubkey_bytes.data());
  pubkey_str = encodeToHex(pubkey);
}


std::string Signer::sign(const std::string& message){

  std::istringstream source(message);
  Poco::Crypto::DigestEngine sha256("SHA256");
  sha256.reset();
  Poco::DigestOutputStream digest_output_stream(sha256);
  Poco::StreamCopier::copyStream(source, digest_output_stream);
  digest_output_stream.close();
  auto digest = sha256.digest();

//  Poco::Crypto::DigestEngine sha256("SHA256");
//  sha256.reset();
//  sha256.update(message);
//  auto digest = sha256.digest();

  return this->_sign(digest);

}

std::string Signer::_sign(const std::vector<unsigned char>& digest){

  std::unique_ptr<secp256k1_ecdsa_signature> raw_sig(new secp256k1_ecdsa_signature);
  const unsigned char *msg32 = digest.data();
  secp256k1_nonce_function nonce_fn = NULL;
  const void *nonce_data = NULL;

  const unsigned char *privkey_ptr = (unsigned char*) privkey.c_str();
  secp256k1_ecdsa_sign(
      context_, raw_sig.get(), msg32, privkey_ptr, nonce_fn, nonce_data);

  std::array<uint8_t, 64> output64;
  secp256k1_ecdsa_signature_serialize_compact(
      context_, output64.data(), raw_sig.get());
  std::string signature((char *)output64.data(), output64.size());

  return signature;

}


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



std::string encodeToHex(const std::string& str)
{
  std::istringstream source(str);
  std::ostringstream sink;

  Poco::HexBinaryEncoder encoder(sink);
  Poco::StreamCopier::copyStream(source, encoder);
  encoder.close();

  return sink.str();
}

std::string decodeFromHex(const std::string& str)
{
  std::istringstream source(str);
  std::ostringstream sink;

  Poco::HexBinaryDecoder decoder(source);
  Poco::StreamCopier::copyStream(decoder, sink);

  return sink.str();
}

}  // namespace bbr_sawtooth_bridge
