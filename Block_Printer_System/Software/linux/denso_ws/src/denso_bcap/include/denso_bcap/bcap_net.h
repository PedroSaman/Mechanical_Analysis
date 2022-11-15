/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 * Based on Denso b-Cap example
 */

#ifndef BCAP_SERIAL_H_
#define BCAP_SERIAL_H_

#include <denso_bcap/bcap_base.h>
#include <boost/asio.hpp>

class BCapNet : public BCap
{
public:
  enum ConnectingMode
  {
    BCAP_TCP,
    BCAP_UDP
  };
  BCapNet(const std::string& address, const std::string& port, ConnectingMode mode = BCAP_TCP);
  ~BCapNet();
protected:
  BCAP_HRESULT SendBinary(uint8_t *pBuff, uint32_t lSize);
  uint8_t *ReceivePacket();

private:
  uint8_t pRcvBuffer[LOCALRECBUFFER_SZ];
  ConnectingMode mode_;
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket* tcp_socket_;
  boost::asio::ip::udp::socket* udp_socket_;
  boost::asio::ip::udp::endpoint host_endpoint_;
};




#endif /* BCAP_SERIAL_H_ */
