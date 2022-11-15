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

#include <denso_bcap/bcap_net.h>
using namespace boost;
using namespace boost::asio::ip;

BCapNet::BCapNet(const std::string& address, const std::string& port, ConnectingMode mode)
  : BCap(false), mode_(mode), tcp_socket_(0), udp_socket_(0)
{
  if(mode_ == BCAP_TCP)
  {
    tcp::resolver resolver(io_service_);
    tcp::resolver::query query(tcp::v4(), address, port);
    tcp::resolver::iterator iterator = resolver.resolve(query);
    tcp_socket_ = new tcp::socket(io_service_);
    tcp_socket_->connect(*iterator);
  }
  else
  {
    udp_socket_ = new udp::socket(io_service_, udp::endpoint(udp::v4(), 0));

    udp::resolver resolver(io_service_);
    udp::resolver::query query(udp::v4(), address, port);
    udp::resolver::iterator iterator = (resolver.resolve(query));
    host_endpoint_ = *iterator;
    udp_socket_->connect(host_endpoint_);
  }
}

BCapNet::~BCapNet()
{
  if(tcp_socket_)
  {
    tcp_socket_->close();
    delete tcp_socket_;
  }
  if(udp_socket_)
  {
    udp_socket_->close();
    delete udp_socket_;
  }
}

BCAP_HRESULT BCapNet::SendBinary(uint8_t *pBuff, uint32_t lSize)
{
  int lSendLen = 0;
  try
  {
    if(mode_ == BCAP_TCP)
    {
      lSendLen = asio::write(*tcp_socket_, asio::buffer(pBuff, lSize));
    }
    else
    {
      lSendLen = udp_socket_->send_to(asio::buffer(pBuff, lSize), host_endpoint_);
    }

    if (m_show_debug_packet)
    {
      printf("\n-------------- TX --------------\n");
      int count = 0;
      for (int i = 0; i < lSendLen; i++)
      {

        printf("%02x ", (unsigned char)pBuff[i]);
        count++;
        if (count == 16)
        {
          count = 0;
          printf("\n");
        }

      }
      printf("\n--------------------------------\n");
    }

  }
  catch (boost::system::system_error & error)
  {
    return BCAP_E_FAIL;
  }
  return BCAP_S_OK;
}

uint8_t * BCapNet::ReceivePacket()
{
  uint8_t *pTop;
  uint8_t *pPacketBuff = NULL;
  uint32_t lRecvSize;
  int lRecLen = 0;

  int count = 0;

  lRecvSize = 0;
  if (m_show_debug_packet)
  {
    printf("\n-------------- RX --------------\n");
  }

  if(mode_ == BCAP_TCP)
  {
    lRecLen = asio::read(*tcp_socket_, asio::buffer(&(pRcvBuffer[lRecvSize]), BCAP_HEADER_SIZE));
  }
  else
  {
    udp::endpoint sender_endpoint;
    lRecLen = udp_socket_->receive_from(asio::buffer(&(pRcvBuffer[lRecvSize]), LOCALRECBUFFER_SZ), sender_endpoint);
  }

  if (m_show_debug_packet)
  {
    for (int i = 0; i < lRecLen; i++)
    {
      printf("%02x ", (unsigned char)pRcvBuffer[lRecvSize + i]);
      count++;
      if (count == 16)
      {
        count = 0;
        printf("\n");
      }
    }
  }
  lRecvSize += lRecLen; /* add read bytes */

  pTop = (uint8_t *)memchr((const void *)pRcvBuffer, BCAP_SOH, lRecvSize);
  if (pTop == NULL)
  { /* Is there SOH ? */
    lRecvSize = 0; /* If No SOH, then all read data are discarded */
  }
  else
  {
    if (pTop != pRcvBuffer)
    { /* if (pTop == pRcvBuffer) then SOH is already in the top. */
      lRecvSize = lRecvSize - (pTop - pRcvBuffer); /* exclude before SOH  */
      memmove(pRcvBuffer, pTop, lRecvSize);
    }
  }

  uint32_t lPacketSize;
  uint32_t lRemainSize;

  copyValue(&lPacketSize, &(pRcvBuffer[1]), BCAP_SIZE_LEN);
  lRemainSize = lPacketSize - lRecvSize;

  //printf("\nlPacketSize: %d lRecvSize: %d lRemainSize: %d \n", lPacketSize, lRecvSize, lRemainSize);


  //lRecLen = asio::read(serial_port_, asio::buffer(&(pRcvBuffer[lRecvSize]), lRemainSize));

  if(mode_ == BCAP_TCP)
  {
    lRecLen = asio::read(*tcp_socket_, asio::buffer(&(pRcvBuffer[lRecvSize]), lRemainSize));
  }
  else
  {
    lRecLen = lRemainSize;
  }


  if (m_show_debug_packet)
  {
    for (int i = 0; i < lRecLen; i++)
    {
      printf("%02x ", (unsigned char)pRcvBuffer[lRecvSize + i]);
      count++;
      if (count == 16)
      {
        count = 0;
        printf("\n");
      }
    }
    printf("\n-------------- RX --------------\n");
  }

  lRecvSize += lRecLen; /* add read bytes */

  if (pRcvBuffer[lPacketSize - 1] != BCAP_EOT)
  {
    return NULL;
  }

  pPacketBuff = (unsigned char *)bMalloc(lPacketSize);
  if (pPacketBuff != NULL)
  {
    memcpy(pPacketBuff, pRcvBuffer, lRecvSize);
    return pPacketBuff;
  }
  else
  {
    return NULL;
  }
}
