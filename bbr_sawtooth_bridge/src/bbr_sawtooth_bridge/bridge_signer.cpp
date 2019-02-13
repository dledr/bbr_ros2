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

#include <iostream>
#include <inttypes.h>
#include <memory>

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


  std::string private_pem = "-----BEGIN EC PARAMETERS-----\n"
                            "BgUrgQQACg==\n"
                            "-----END EC PARAMETERS-----\n"
                            "-----BEGIN EC PRIVATE KEY-----\n"
                            "MHQCAQEEIHSL071lsUCnE7G1kmHE0tmMgtuFKSwnyFaU4hYO38EjoAcGBSuBBAAK\n"
                            "oUQDQgAES/58IaP2xljOS0XreQVUbj03VFSG5qOHvjxn6GRsCepu4WiXHa0lRjRH\n"
                            "xqhVFvuA0/6PqJph5QFy55TP/vlp3Q==\n"
                            "-----END EC PRIVATE KEY-----" ;

  std::istringstream iPriv(private_pem);
  Poco::Crypto::ECKey ec_key(NULL, &iPriv, "");
  Poco::Crypto::ECDSADigestEngine ecdsa_digest_engine(ec_key, "SHA256");

  ecdsa_digest_engine.reset();
  ecdsa_digest_engine.update("foo");
  auto spam = ecdsa_digest_engine.digest();
  auto digest = ecdsa_digest_engine.signature();
  assert(ecdsa_digest_engine.verify(digest) == true);


//  std::ostringstream strPubE;
//  std::ostringstream strPrivE;
//  ec_key.save(&strPubE, &strPrivE, "");
//  std::string pubKeyE = strPubE.str();
//  std::string privKeyE = strPrivE.str();
//  std::cout << privKeyE << "\n";


  std::ostringstream hashStream;
  Poco::HexBinaryEncoder hexEnc(hashStream);
  hexEnc.write(reinterpret_cast<char*>(digest.data()), digest.size());
  hexEnc.close();
  std::string hashHex = hashStream.str();
  std::cout << hashHex << "\n";


}


}  // namespace bbr_sawtooth_bridge
