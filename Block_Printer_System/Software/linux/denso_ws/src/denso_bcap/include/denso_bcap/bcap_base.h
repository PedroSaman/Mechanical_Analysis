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


#ifndef _BCAP_BASE_
#define _BCAP_BASE_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

/**
 * @enum BCAP_HRESULT
 * @brief        BCAP_HRESULT values
 */
typedef enum BCAP_HRESULT
{
  BCAP_S_OK = 0, /*      OK                                      */
  BCAP_E_NOTIMPL = 0x80004001, /*      Not implemented function is called      */
  BCAP_E_ABORT = 0x80004004, /*      Function aborted                        */
  BCAP_E_FAIL = 0x80004005, /*      Function failed                         */
  BCAP_E_UNEXPECTED = 0x8000FFFF, /*      Fatal Error occurred                    */
  BCAP_E_INVALIDRCVPACKET = 0x80010001, /*      Invalid packet is received.             */

  /* When this error is occurred, robot controller disconnect from client immediately.*/
  /* Please make sure the packet that you sent. */

  BCAP_E_INVALIDSNDPACKET = 0x80010002, /*      Invalid packet is sent                  */
  BCAP_E_INVALIDARGTYPE = 0x80010003, /*      Invalid argument type                   */
  BCAP_E_ROBOTISBUSY = 0x80010004, /*      Robot is busy (Wait for a while)        */
  BCAP_E_INVALIDCOMMAND = 0x80010005, /*      Invalid command string is received      */

  BCAP_E_PACKETSIZEOVER = 0x80010011, /*      Received packet size over ( > 16Mbytes) */

  BCAP_E_ARGSIZEOVER = 0x80010012, /*      An argument size over of the received packet. ( > 16Mbytes) */
  BCAP_E_ACCESSDENIED = 0x80070005, /*      Access denied                           */
  BCAP_E_HANDLE = 0x80070006, /*      Invalid handle                          */
  BCAP_E_OUTOFMEMORY = 0x8007000E, /*      Out of memory                           */
  BCAP_E_INVALIDARG = 0x80070057 /*      Invalid argument                        */

} BCAP_HRESULT;

/* b-CAP Type id */
#define VT_EMPTY                                0                       /* (0Byte) */
#define VT_NULL                                 1                       /* (0Byte) */
#define VT_ERROR                                10                      /* (2Byte) */
#define VT_UI1                                  17                      /* (1Byte) */
#define VT_I2                                   2                       /* (2Byte) */
#define VT_UI2                                  18                      /* (2Byte) */
#define VT_I4                                   3                       /* (4Byte) */
#define VT_UI4                                  19                      /* (4Byte) */
#define VT_R4                                   4                       /* (4Byte) */
#define VT_R8                                   5                       /* (8Byte) */
#define VT_CY                                   6                       /* (8Byte) */
#define VT_DATE                                 7                       /* (8Byte) */
#define VT_BOOL                                 11                      /* (2Byte) */
#define VT_BSTR                                 8                       /* (ASCII string length *2 + 4 Byte) */
/* Double bytes per character */
#define VT_VARIANT                              12                      /* Variant */
#define VT_ARRAY                                0x2000                  /* Array */

/* b-CAP Utility macros */
#ifndef SUCCEEDED
#define SUCCEEDED(Status) ((BCAP_HRESULT)(Status) == 0)
#endif

#ifndef FAILED
#define FAILED(Status) ((BCAP_HRESULT)(Status) != 0)
#endif

/* length of temporary string buffer */
#define LOCALBUFFER_SZ                  256
/* length of temporary recieve buffer (must be >= 16bytes) */
#define LOCALRECBUFFER_SZ               256

/* b-CAP packet constant */
#define BCAP_SOH                        0x01                    /* size of packet header(SOH) */
#define BCAP_EOT                        0x04                    /* size of packet terminator(EOT) */
#define BCAP_SIZE_SOH                   1                       /* size of header(SOH)   */
#define BCAP_SIZE_EOT                   1                       /* size of terminator(EOT)  */
#define BCAP_SIZE_LEN                   4                       /* size of message size */
#define BCAP_SIZE_SERIAL                2                       /* size of serial number */
#define BCAP_SIZE_RESERVE               2                       /* size of reserved */
#define BCAP_SIZE_FUNCID                4                       /* size of FunctionID */
#define BCAP_SIZE_ARGNUM                2                       /* size of Args */

/* b-CAP Packet base size */
#define BCAP_SIZE_BASE  (BCAP_SIZE_SOH + BCAP_SIZE_EOT + \
                          BCAP_SIZE_LEN + BCAP_SIZE_SERIAL + \
                          BCAP_SIZE_RESERVE + BCAP_SIZE_FUNCID + \
                          BCAP_SIZE_ARGNUM)

#define BCAP_HEADER_SIZE (BCAP_SIZE_SOH + BCAP_SIZE_LEN + \
                           BCAP_SIZE_SERIAL + BCAP_SIZE_RESERVE + \
                           BCAP_SIZE_FUNCID + BCAP_SIZE_ARGNUM)

/* b-CAP Argument structure */
/**
 * @struct       BCAP_ARG
 * @brief        BCAP_ARG
 */
typedef struct BCAP_ARG
{
  uint32_t lLength;

  uint16_t iType;
  uint32_t lArrays;
  void *data;

  struct BCAP_ARG *pNextArg; /* pointer to the next argument  */
} BCAP_ARG;

/* b-CAP Packet structure */
/**
 * @struct       BCAP_PACKET
 * @brief        BCAP_PACKET
 */
typedef struct BCAP_PACKET
{

  uint32_t lMsgLength;

  uint16_t iSerialNo;
  uint16_t iReserved;

  uint32_t lFuncID;

  uint16_t iArgs;

  struct BCAP_ARG *pArg; /* pointer to the first argument */
} BCAP_PACKET;

class BCap
{

public:
  BCap(bool need_crc = false);
  virtual ~BCap() { }

  void set_debug_packet(bool show)
  {
    m_show_debug_packet = show;
  }

  BCAP_HRESULT ServiceStart();
  BCAP_HRESULT ServiceStop();

  /* b-CAP Controller Functions */
  BCAP_HRESULT ControllerConnect(const std::string& pStrCtrlname, const std::string& pStrProvName,
                                 const std::string& pStrPcName, const std::string& pStrOption, uint32_t* plhController);
  BCAP_HRESULT ControllerDisconnect(uint32_t lhController);

  BCAP_HRESULT ControllerGetRobot(uint32_t lhController, const std::string& pStrRobotName,
                                  const std::string& pStrOption, uint32_t *lhRobot);
  BCAP_HRESULT ControllerGetVariable(uint32_t lhController, char *pVarName, char *pstrOption, uint32_t *plhVar);
  BCAP_HRESULT ControllerGetTask(uint32_t lhController, const std::string& pTskName, const std::string& pstrOption,
                                 uint32_t *plhVar);
  BCAP_HRESULT ControllerExecute(uint32_t lhController, char *pStrCommand, char *pStrOption, long *plResult);
  BCAP_HRESULT ControllerExecute2(uint32_t lhController, const std::string& pStrCommand, uint16_t iType,
                                  uint32_t lArrays, void *pVntValue, void *pVntReturn);

  /* b-CAP Robot Functions */
  BCAP_HRESULT RobotRelease(uint32_t lhRobot);
  BCAP_HRESULT RobotGetVariable(uint32_t lhRobot, const std::string& pVarName, const std::string& pStrOption,
                                uint32_t *lhVar);
  BCAP_HRESULT RobotExecute(uint32_t lhRobot, char *pStrCommand, char *pStrOption, long *plResult);
  BCAP_HRESULT RobotExecute2(uint32_t lhRobot, const std::string& pStrCommand, uint16_t iType, uint32_t lArrays,
                             void *pVntValue, void *pVntReturn);
  BCAP_HRESULT RobotChange(uint32_t lhRobot, char *pStrCommand);
  BCAP_HRESULT RobotMove(uint32_t lhRobot, long lComp, const std::string& pStrPose, const std::string& pStrOption);

  /* b-CAP Task Functions */
  BCAP_HRESULT TaskRelease(uint32_t lhTask);
  BCAP_HRESULT TaskGetVariable(uint32_t lhTask, char *pVarName, char *pstrOption, uint32_t *plhVar);
  BCAP_HRESULT TaskStart(uint32_t lhTask, long lMode, const std::string& pStrOption);
  BCAP_HRESULT TaskStop(uint32_t lhTask, long lMode, const std::string& pStrOption);

  /* b-CAP Variable Functions */
  BCAP_HRESULT VariableRelease(uint32_t lhVar);
  BCAP_HRESULT VariableGetValue(uint32_t lhVar, void *pVntValue);
  BCAP_HRESULT VariablePutValue(uint32_t lhVar, uint16_t iType, uint32_t lArrays, void *pVntValue);

protected:
  /* module socket utility functions */
  BCAP_HRESULT PacketSend(BCAP_PACKET *pPacket);
  BCAP_HRESULT SendAndReceive(BCAP_PACKET *pSndPacket, BCAP_PACKET *pRecPacket);
  virtual BCAP_HRESULT SendBinary(uint8_t *pBuff, uint32_t lSize) = 0;
  virtual uint8_t *ReceivePacket() = 0;

  /* packet functions */
  BCAP_PACKET *Packet_Create(uint32_t lFuncID);
  void Packet_Release(BCAP_PACKET *pPacket); /* Release allocated packet and the arguments */
  BCAP_HRESULT Packet_Serialize(BCAP_PACKET *pPacket, void *pBinData); /* struct ---> bin */
  BCAP_HRESULT Packet_Deserialize(void *pBinData, BCAP_PACKET *pPacket); /* bin ---> struct  */
  BCAP_HRESULT Packet_AddArg(BCAP_PACKET *pPacket, BCAP_ARG *pNewArg);
  //BCAP_HRESULT      Packet_AddCRC(BCAP_PACKET *pPacket);

  /* argument functions */
  BCAP_ARG *Arg_Create(uint16_t iType, uint32_t lArrays, uint32_t lLength, void *data);
  void Arg_Release(BCAP_ARG *pArg); /* free the allocated argument */
  BCAP_ARG **Packet_GetLastArgHandle(BCAP_PACKET *pPacket);

  /* module utility functions */
  uint32_t sizeOfVarType(uint16_t iType);
  uint32_t copyValue(void *pDst, void *pVntValue, uint32_t lLength);
  uint32_t copyToBSTR(uint8_t *pbDstPtr, const char *pbSrcPtr);
  uint32_t copyFromBSTR(void *pDstAsciiPtr, void *pSrcBstrPtr);
  void *bMalloc(size_t size);
  void bFree(void *pPtr);

  /*
   *   Memory allocation counter
   */
  int32_t m_lAllocCount;
  int32_t m_lAllocSize;

  //packet serial number: cyclic from 0x0001 to 0xFFFF
  uint16_t m_iSerialNo;

  //need CRC-CCITT for serial communication
  bool m_do_crc;
  bool m_show_debug_packet;
};

#endif
