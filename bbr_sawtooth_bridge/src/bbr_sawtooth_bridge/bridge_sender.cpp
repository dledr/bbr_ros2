// Copyright 2019, Gianluca Caiazza.
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

#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/StreamCopier.h>
#include <Poco/Path.h>
#include <Poco/URI.h>
#include <Poco/Exception.h>

#include "bbr_sawtooth_bridge/bridge_sender.hpp"

#include <inttypes.h>
#include <iostream>
#include <sstream>
#include <memory>
#include <fstream>


using namespace Poco::Net;
using namespace Poco;
using namespace std;

 namespace bbr_sawtooth_bridge
 {

 std::string post_batches(std::string& url, std::string& contents)
 {
  
  try
  { 

    
    // prepare session
    URI uri(url);
        
    HTTPClientSession session(uri.getHost(), uri.getPort());

        
    // send request
    HTTPRequest req(HTTPRequest::HTTP_POST, "/batches", HTTPMessage::HTTP_1_1);
    req.setContentType("application/octet-stream");


    // Set the request body
    req.setContentLength( contents.length() );
        
    // sends request, returns open stream
    std::ostream& os = session.sendRequest(req);
    os << contents;  // sends the body
    //req.write(std::cout); // print out request
        
    // get response
    HTTPResponse res;
    std::cout << res.getStatus() << " " << res.getReason() << '\n';
        
    std::istream &is = session.receiveResponse(res);
    std::stringstream ss;
    StreamCopier::copyStream(is, ss);
    //std::cout << ss.str() << '\n';
    return ss.str();
  }
  catch (Exception &ex)
  {
    std::cerr << ex.displayText() << endl;
    //return -1;
    return ex.displayText();
  }


  //return 0;
 }
 
}  // namespace bbr_sawtooth_bridge
