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

/* OS Switching */
#ifdef WIN32
/* Windows */
#include <winsock2.h>
#pragma comment(lib, "wsock32.lib")

#else
/* Other */
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>

#endif

/* b-CAP functions */
#include <denso_bcap/bcap_base.h>
#include <denso_bcap/crc16.h>

/* ENDIAN switching */
#if !defined(__LITTLE_ENDIAN__)
#if !defined(__BIG_ENDIAN__)

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define __LITTLE_ENDIAN__
#elif __BYTE_ORDER == __BIG_ENDIAN
#define __BIG_ENDIAN__
#endif

#endif
#endif

/* DEBUG */
// #define      DEBUG                                   1       // Set to debug mode
#ifdef DEBUG
#define DebugPrint( message )   fprintf( stderr, message )
#define DebugPrint2( f, a, b )  fprintf( stderr, f, a, b )
#else
#define DebugPrint( message )
#define DebugPrint2( f, a, b )
#endif

/* b-CAP argument constant */
#define BCAP_SIZE_ARGLEN                4               /* size of length  */
#define BCAP_SIZE_ARGTYPE               2               /* size of type */
#define BCAP_SIZE_ARGARRAYS             4               /* size of arrays */
#define BCAP_SIZE_ARGBASE               (BCAP_SIZE_ARGLEN+BCAP_SIZE_ARGTYPE+BCAP_SIZE_ARGARRAYS)
/* b-CAP Arg header size  */
#define BCAP_SIZE_ARGSTRLEN             4               /* size of string length */

#define BCAP_MAX_PACKET_SIZE            0x1000000       /* max packet size (bytes) */
#define BCAP_MAX_ARG_SIZE               0x1000000       /* max arg size (bytes) */

/* b-CAP function IDs */
#define BCAP_FUNC_Service_Start                 1
#define BCAP_FUNC_Service_Stop                  2
#define BCAP_FUNC_Controller_Connect            3
#define BCAP_FUNC_Controller_Disconnect         4
#define BCAP_FUNC_Controller_GetRobot           7
#define BCAP_FUNC_Controller_GetTask            8
#define BCAP_FUNC_Controller_GetVariable        9
#define BCAP_FUNC_Controller_Execute            17

#define BCAP_FUNC_Robot_GetVariable             62
#define BCAP_FUNC_Robot_Execute                 64
#define BCAP_FUNC_Robot_Change                  66
#define BCAP_FUNC_Robot_Move                    72
#define BCAP_FUNC_Robot_Release                 84

#define BCAP_FUNC_Task_GetVariable              85
#define BCAP_FUNC_Task_Start                    88
#define BCAP_FUNC_Task_Stop                     89
#define BCAP_FUNC_Task_Release                  99

#define BCAP_FUNC_Variable_GetValue             101
#define BCAP_FUNC_Variable_PutValue             102
#define BCAP_FUNC_Variable_Release              111

/*--------------------------------------------------------------------
 b-Cap library public functions
 --------------------------------------------------------------------*/

BCap::BCap(bool do_crc) :
    m_lAllocCount(0),
    m_lAllocSize(0),
    m_iSerialNo(1),
    m_do_crc(do_crc),
    m_show_debug_packet(false)
{

}

/**     Start b-Cap service
 *
 * Start b-Cap servic
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::ServiceStart()
{
  BCAP_PACKET *pPacket;
  BCAP_PACKET *pRecPacket;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pPacket = Packet_Create(BCAP_FUNC_Service_Start); /* Service_Start */
  if (pPacket != NULL)
  {

    {
      pRecPacket = Packet_Create(BCAP_S_OK); /* storing a new packet from RC    */
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pPacket, pRecPacket);
      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pPacket);
  }

  return hr;

}

/**     Stop b-Cap service
 *
 * Stop b-Cap service
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::ServiceStop()
{
  BCAP_PACKET *pPacket;
  BCAP_PACKET *pRecPacket;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pPacket = Packet_Create(BCAP_FUNC_Service_Stop); /* alloc new packet:Service_Stop */
  if (pPacket != NULL)
  {

    {
      pRecPacket = Packet_Create(BCAP_S_OK); /*  alloc new packet storing a packet from RC    */
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pPacket, pRecPacket);

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pPacket);
  }

  return hr;

}

/**     Controller_Connect
 *
 * Controller_Connect
 *
 *      @param  iSockFd         :       [in]  Socket descriptor
 *      @param  pStrCtrlname    :       [in]  CtrlName in AsciiZ
 *      @param  pStrProvName    :       [in]  ProvName in AsciiZ
 *      @param  pStrPcName      :       [in]  PCName in AsciiZ
 *      @param  pStrOption      :       [in]  Option string in AsciiZ
 *      @param  plhController   :       [out]  handle of the controller that returned from the robot controller.
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::ControllerConnect(const std::string& pStrCtrlname, const std::string& pStrProvName,
                                          const std::string& pStrPcName, const std::string& pStrOption,
                                          uint32_t* plhController)
{
  BCAP_PACKET *pPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pPacket = Packet_Create(BCAP_FUNC_Controller_Connect); /* Controller_Connect */
  if (pPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      lLen = copyToBSTR(buff, pStrCtrlname.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pPacket, pArg);
      }
    }
    {
      lLen = copyToBSTR(buff, pStrCtrlname.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pPacket, pArg);
      }
    }
    {
      lLen = copyToBSTR(buff, pStrPcName.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pPacket, pArg);
      }
    }
    {
      lLen = copyToBSTR(buff, pStrOption.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          copyValue(plhController, pRecPacket->pArg->data, 4);
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pPacket);
  }

  return hr;

}

/**     Disconnect b-Cap
 *
 * Controller_Disconnect
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhController            :       [in]  controller handle to disconnect
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::ControllerDisconnect(uint32_t lhController)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Controller_Disconnect); /* Controller_Disconnect */
  if (pSndPacket != NULL)
  {

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhController);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK); /* storing a new packet from RC    */
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;

}

/**     Controller_GetRobot
 *
 * Controller_GetRobot
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhController            :       [in]  controller handle
 *      @param  pStrRobotName           :       [in]  Robot name string in AsciiZ
 *      @param  pStrOption                      :       [in]  Option string in AsciiZ
 *      @param  lhRobot                         :       [out]  robot handle
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::ControllerGetRobot(uint32_t lhController, const std::string& pStrRobotName, const std::string& pStrOption,
                                                 uint32_t *plhRobot)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  uint8_t buff[LOCALBUFFER_SZ];
  uint32_t lLen;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Controller_GetRobot); /* BCAP_FUNC_Controller_GetRobot */
  if (pSndPacket != NULL)
  {

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhController);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }
    {
      lLen = copyToBSTR(buff, pStrRobotName.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }
    {
      lLen = copyToBSTR(buff, pStrOption.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }
    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          copyValue(plhRobot, pRecPacket->pArg->data, 4);
        }
        else
        {
          /* NO Argument */
          hr = BCAP_E_FAIL;
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Controller_GetVariable
 *
 * Controller_GetVariable
 *
 *      @param  iSockFd                 :       [in]  Socket descriptor
 *      @param  lhController    :       [in]  controller handle
 *      @param  pVarName                :       [in]  Variable name string in AsciiZ
 *      @param  pstrOption              :       [in]  Option string in AsciiZ
 *      @param  plhVar                  :       [out]  variable handle
 *      @retval BCAP_HRESULT
 *
 */

BCAP_HRESULT BCap::ControllerGetVariable(uint32_t lhController, char *pVarName, char *pstrOption,
                                                    uint32_t *plhVar)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Controller_GetVariable); /* BCAP_FUNC_Controller_GetVariable */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhController);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pVarName);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pstrOption);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          if (pRecPacket->iArgs >= 1)
          {
            copyValue(plhVar, pRecPacket->pArg->data, 4);
          }
          else
          {
            /* NO Argument */
            hr = BCAP_E_FAIL;
          }
        }
      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Controller_GetTask
 *
 * Controller_GetTask
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhController            :       [in]  controller handle
 *      @param  pTskName                :       [in]  task name string in AsciiZ
 *      @param  pstrOption              :       [in]  option string in AsciiZ
 *      @param  plhVar          :       [out]  task handle
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::ControllerGetTask(uint32_t lhController, const std::string& pTskName, const std::string& pstrOption,
                                                uint32_t *plhVar)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Controller_GetTask); /* BCAP_FUNC_Controller_GetTask */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhController);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pTskName.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pstrOption.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          if (pRecPacket->iArgs >= 1)
          {
            copyValue(plhVar, pRecPacket->pArg->data, 4);
          }
          else
          {
            /* NO Argument */
            hr = BCAP_E_FAIL;
          }
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Controller_Execute
 *
 * Controller_Execute
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhController            :       [in]  controller handle
 *      @param  pStrCommand             :       [in]  command string in AsciiZ
 *      @param  pstrOption              :       [in]  option string in AsciiZ
 *      @param  plResult                :       [out]  result value
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::ControllerExecute(uint32_t lhController, char *pStrCommand, char *pStrOption,
                                                long *plResult)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Controller_Execute); /* BCAP_FUNC_Controller_Execute */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhController);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrCommand);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrOption);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          if (pRecPacket->iArgs >= 1)
          {
            copyValue(plResult, pRecPacket->pArg->data, 4);
          }
          else
          {
            /* NO Argument */
            hr = BCAP_E_FAIL;
          }
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

BCAP_HRESULT BCap::ControllerExecute2(uint32_t lhController,
                                           const std::string& pStrCommand,
                                           uint16_t iType, uint32_t lArrays,
                                           void *pVntValue,
                                           void *pVntReturn)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Controller_Execute); /* BCAP_FUNC_Controller_Execute */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhController);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrCommand.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      uint32_t lDataLen = 0;
      uint32_t lLen;
      lLen = sizeOfVarType((uint16_t)(iType & ~VT_ARRAY));
      lDataLen = lLen * lArrays;
      //if (lDataLen != 0) <-- removed to support empty argument
      {
        //printf("aa\n");
        uint32_t i;
        uint8_t *pSrcData = (uint8_t *)pVntValue;
        uint8_t *pDstData = (uint8_t *)bMalloc(lDataLen);
        if (pDstData != NULL)
        {
          uint8_t *pDstPtr = pDstData;
          for (i = 0; i < lArrays; i++)
          {

            copyValue(pDstPtr, pSrcData, lLen);
            pDstPtr += lLen;
            pSrcData += lLen;
          }

          pArg = Arg_Create(iType, lArrays, lDataLen, pDstData);
          if (pArg != NULL)
          {
            Packet_AddArg(pSndPacket, pArg);
          }
          bFree(pDstData);
        }
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          //printf("arg no. %d\n", pRecPacket->iArgs);
          if (pRecPacket->iArgs >= 1)
          {
            { /* Copy values */
              uint32_t i;
              uint32_t lSize;
              uint16_t iType;
              BCAP_ARG *pArgValue = pRecPacket->pArg;

              iType = (pArgValue->iType) & ~VT_ARRAY; /* Mask "Array" */

              //switch(iType)
              //printf("iType: 0x%x %d\n", iType, iType);

              if (iType == VT_BSTR)
              {
                //printf("a\n");
                uint8_t *pDstAscii = (uint8_t *)pVntReturn;
                uint8_t *pSrcBstr = (uint8_t *)pArgValue->data;

                for (i = 0;i < pArgValue->lArrays;i++)
                {
                  lSize = copyFromBSTR(pDstAscii, pSrcBstr);
                  pDstAscii += lSize;
                  pSrcBstr += BCAP_SIZE_ARGSTRLEN + ((lSize -1) * 2); /* lSize include Terminator,so (lSize -1) * 2) */
                }
              }
              else
              {

                //printf("b\n");
                lSize = sizeOfVarType((uint16_t)(pArgValue->iType));
                //printf("%d %d %d %x\n", lSize, pArgValue->lArrays, pArgValue->iType, pArgValue->iType);

                if(pArgValue->iType & VT_ARRAY)
                {
                  //printf("b1\n");
                  if (lSize != 0)
                  {

                    uint8_t *pDst = (uint8_t *)pVntReturn;
                    uint8_t *pSrc = (uint8_t *)pArgValue->data;

                    for (i = 0;i < pArgValue->lArrays;i++)
                    {
                      copyValue(pDst, pSrc, lSize);
                      pDst += lSize;
                      pSrc += lSize;
                    }
                  }
                }
                else
                {
                  //printf("b2\n");
                  uint8_t *pDst = (uint8_t *)pVntReturn;
                  uint8_t *pSrc = (uint8_t *)pArgValue->data;
                  copyValue(pDst, pSrc, lSize);
                }
              }
            }
          }
        }
      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Robot_Release
 *
 * Robot_Release
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhRobot         :       [in]  robot handle
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::RobotRelease(uint32_t lhRobot)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Robot_Release); /* FuncID:RobotRelease */
  if (pSndPacket != NULL)
  {

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhRobot);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg); /* Add 1st argument */
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK); /* Alloc for storing received packet */
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);

      }
      Packet_Release(pRecPacket); /* Release recieved packet */
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Robot_GetVariable
 *
 * Robot_GetVariable
 *
 *      @param  iSockFd                 :       [in]  Socket descriptor
 *      @param  lhRobot                 :       [in]  robot handle
 *      @param  pVarName                :       [in]  Variable name string in AsciiZ
 *      @param  pstrOption              :       [in]  Option string in AsciiZ
 *      @param  plhVar                  :       [out]  result value = variable handle
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::RobotGetVariable(uint32_t lhRobot, const std::string& pVarName, const std::string& pstrOption, uint32_t *plhVar)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Robot_GetVariable); /* BCAP_FUNC_Robot_GetVariable */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhRobot);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pVarName.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pstrOption.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          if (pRecPacket->iArgs >= 1)
          {
            copyValue(plhVar, pRecPacket->pArg->data, 4);
          }
          else
          {

          }
        }
      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Robot_Execute
 *
 * Robot_Execute
 *
 *      @param  iSockFd                 :       [in]  Socket descriptor
 *      @param  lhRobot                 :       [in]  robot handle
 *      @param  pStrCommand             :       [in]  command string in AsciiZ
 *      @param  pstrOption              :       [in]  option string in AsciiZ
 *      @param  plhVar                  :       [out]  result value
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::RobotExecute(uint32_t lhRobot, char *pStrCommand, char *pStrOption, long *plResult)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Robot_Execute); /* BCAP_FUNC_Robot_Execute */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhRobot);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrCommand);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrOption);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          if (pRecPacket->iArgs >= 1)
          {
            copyValue(plResult, pRecPacket->pArg->data, 4);
          }
          else
          {
            /* NO Argument */
            hr = BCAP_E_FAIL;
          }
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }
  return hr;
}

BCAP_HRESULT BCap::RobotExecute2(uint32_t lhRobot,
                                      const std::string& pStrCommand,
                                      uint16_t iType,
                                      uint32_t lArrays,
                                      void *pVntValue,
                                      void *pVntReturn)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Robot_Execute); /* BCAP_FUNC_Robot_Execute */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhRobot);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrCommand.c_str());
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      uint32_t lDataLen = 0;
      uint32_t lLen;
      lLen = sizeOfVarType((uint16_t)(iType & ~VT_ARRAY));
      lDataLen = lLen * lArrays;

      //printf("aa\n");
      uint32_t i;
      uint8_t *pSrcData = (uint8_t *)pVntValue;
      uint8_t *pDstData = (uint8_t *)bMalloc(lDataLen);
      if (pDstData != NULL)
      {
        uint8_t *pDstPtr = pDstData;
        for (i = 0; i < lArrays; i++)
        {

          copyValue(pDstPtr, pSrcData, lLen);
          pDstPtr += lLen;
          pSrcData += lLen;
        }

        pArg = Arg_Create(iType, lArrays, lDataLen, pDstData);
        if (pArg != NULL)
        {
          Packet_AddArg(pSndPacket, pArg);
        }
        bFree(pDstData);
      }

    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          //printf("arg no. %d\n", pRecPacket->iArgs);
          if (pRecPacket->iArgs >= 1)
          {
            { /* Copy values */
              uint32_t i;
              uint32_t lSize;
              uint16_t iType;
              BCAP_ARG *pArgValue = pRecPacket->pArg;

              iType = (pArgValue->iType) & ~VT_ARRAY; /* Mask "Array" */

              //switch(iType)
              //printf("iType: 0x%x %d\n", iType, iType);

              if (iType == VT_BSTR)
              {
                //printf("a\n");
                uint8_t *pDstAscii = (uint8_t *)pVntReturn;
                uint8_t *pSrcBstr = (uint8_t *)pArgValue->data;

                for (i = 0;i < pArgValue->lArrays;i++)
                {
                  lSize = copyFromBSTR(pDstAscii, pSrcBstr);
                  pDstAscii += lSize;
                  pSrcBstr += BCAP_SIZE_ARGSTRLEN + ((lSize -1) * 2); /* lSize include Terminator,so (lSize -1) * 2) */
                }
              }
              else
              {



                //printf("b\n");
                lSize = sizeOfVarType((uint16_t)(pArgValue->iType));
                //printf("%d %d %d %x\n", lSize,  pArgValue->lArrays, pArgValue->iType, pArgValue->iType);

                if(pArgValue->iType & VT_ARRAY)
                {
                  //printf("b1\n");
                  if (lSize != 0)
                  {

                    uint8_t *pDst = (uint8_t *)pVntReturn;
                    uint8_t *pSrc = (uint8_t *)pArgValue->data;

                    for (i = 0;i < pArgValue->lArrays;i++)
                    {
                      copyValue(pDst, pSrc, lSize);
                      pDst += lSize;
                      pSrc += lSize;
                    }
                  }
                }
                else
                {
                  //printf("b2\n");
                  uint8_t *pDst = (uint8_t *)pVntReturn;
                  uint8_t *pSrc = (uint8_t *)pArgValue->data;
                  copyValue(pDst, pSrc, lSize);
                }
              }
            }
          }
        }
      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Robot_Change
 *
 * Robot_Change
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhRobot         :       [in]  robot handle
 *      @param  pStrCommand             :       [in]  command string in AsciiZ
 *      @param  pstrOption              :       [in]  option string in AsciiZ
 *      @param  plhVar          :       [out]  result value
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::RobotChange(uint32_t lhRobot, char *pStrCommand)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Robot_Change); /* BCAP_FUNC_Robot_Change */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhRobot);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrCommand);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          ;
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Robot_Move
 *
 * Robot_Move
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhRobot         :       [in]  robot handle
 *      @param  lComp           :       [in]  completion parameter
 *      @param  pStrPose                :       [in]  Pose string in AsciiZ
 *      @param  pstrOption              :       [in]  Option string in AsciiZ
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::RobotMove(uint32_t lhRobot, long lComp, const std::string& pStrPose, const std::string& pStrOption)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Robot_Move); /* BCAP_FUNC_Robot_Move */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhRobot); /* Arg1 Handle of the robot */
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lComp); /* Arg2 Completion param */
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrPose.c_str()); /* Arg3 Pose param */
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrOption.c_str()); /* Arg4 option param */
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          ;
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Task_Release
 *
 * Task_Release
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhTask          :       [in]  task handle
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::TaskRelease(uint32_t lhTask)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Task_Release); /* BCAP_FUNC_Task_Release */
  if (pSndPacket != NULL)
  {

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhTask);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg); /* Arg1 lhTask */
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK); /* Alloc for storing recieved packet */
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);

      }
      Packet_Release(pRecPacket); /* Release recieved packet */
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Task_GetVariable
 *
 * Task_GetVariable
 *
 *      @param  iSockFd                 :       [in]  Socket descriptor
 *      @param  lhTask                  :       [in]  task handle
 *      @param  pVarName                :       [in]  Variable name string in AsciiZ
 *      @param  pstrOption              :       [in]  Option string in AsciiZ
 *      @param  plhVar                  :       [out]  result value = variable handle
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::TaskGetVariable(uint32_t lhTask, char *pVarName, char *pstrOption, uint32_t *plhVar)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Task_GetVariable); /* BCAP_FUNC_Task_GetVariable */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhTask);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pVarName);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pstrOption);
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          if (pRecPacket->iArgs >= 1)
          {
            copyValue(plhVar, pRecPacket->pArg->data, 4);
          }
          else
          {

          }
        }
      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Task_Start
 *
 * Task_Start
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhTask          :       [in]  task handle
 *      @param  lMode           :       [in]  start parameter
 *      @param  pstrOption              :       [in]  Option string in AsciiZ
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::TaskStart(uint32_t lhTask, long lMode, const std::string& pStrOption)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Task_Start); /* BCAP_FUNC_Task_Start */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhTask); /* Arg1 Handle of the task */
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lMode); /* Arg2 start param */
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrOption.c_str()); /* Arg3 option param */
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          ;
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Task_Stop
 *
 * Task_Stop
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhTask          :       [in]  task handle
 *      @param  lMode           :       [in]  stop parameter
 *      @param  pstrOption              :       [in]  Option string in AsciiZ
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::TaskStop(uint32_t lhTask, long lMode, const std::string& pStrOption)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Task_Stop); /* BCAP_FUNC_Task_Stop */
  if (pSndPacket != NULL)
  {

    uint8_t buff[LOCALBUFFER_SZ];
    uint32_t lLen;

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhTask); /* Arg1 Handle of the task */
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lMode); /* Arg2 stop param */
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      lLen = copyToBSTR(buff, pStrOption.c_str()); /* Arg3 option param */
      pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          ;
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Variable_Release
 *
 * Variable_Release
 *
 *      @param  iSockFd :       [in]  Socket descriptor
 *      @param  lhVariable              :       [in]  variable handle
 *      @retval BCAP_HRESULT
 *
 */
BCAP_HRESULT BCap::VariableRelease(uint32_t lhVar)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Variable_Release); /* BCAP_FUNC_Task_Release */
  if (pSndPacket != NULL)
  {

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhVar);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg); /* Arg1 lhVariable */
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK); /* Alloc for storing recieved packet */
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);

      }
      Packet_Release(pRecPacket); /* Release recieved packet */
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Variable_GetValue
 *
 * Variable_GetValue
 *
 *      @param  iSockFd                 :       [in]  Socket descriptor
 *      @param  lhVar                   :       [in]  robot handle
 *      @param  *pVntValue              :       [out]  value stored pointer
 *      @retval BCAP_HRESULT
 *      @detail Note:    This function write value into *pVntValue,
 *                                      So, Client program must allocate enough memory as *pVntValue.
 */
BCAP_HRESULT BCap::VariableGetValue(uint32_t lhVar, void *pVntValue)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Variable_GetValue); /* BCAP_FUNC_Variable_GetValue */
  if (pSndPacket != NULL)
  {

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhVar); /* Arg1: Variable handle */
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          if (pRecPacket->iArgs >= 1)
          {
            { /* Copy values */
              uint32_t i;
              uint32_t lSize;
              uint16_t iType;
              BCAP_ARG *pArgValue = pRecPacket->pArg;

              iType = (pArgValue->iType) & ~VT_ARRAY; /* Mask "Array" */
              if (iType == VT_BSTR)
              {
                uint8_t *pDstAscii = (uint8_t *)pVntValue;
                uint8_t *pSrcBstr = (uint8_t *)pArgValue->data;

                for (i = 0;i < pArgValue->lArrays;i++)
                {
                  lSize = copyFromBSTR(pDstAscii, pSrcBstr);
                  pDstAscii += lSize;
                  pSrcBstr += BCAP_SIZE_ARGSTRLEN + ((lSize -1) * 2); /* lSize include Terminator,so (lSize -1) * 2) */
                }
              }
              else
              {
                lSize = sizeOfVarType((uint16_t)(pArgValue->iType));
                if (lSize != 0)
                {
                  uint8_t *pDst = (uint8_t *)pVntValue;
                  uint8_t *pSrc = (uint8_t *)(uint8_t *)pArgValue->data;

                  for (i = 0;i < pArgValue->lArrays;i++)
                  {
                    copyValue(pDst, pSrc, lSize);
                    pDst += lSize;
                    pSrc += lSize;
                  }
                }
              }
            }
          }
          else
          {
            /* NO Argument */
            hr = BCAP_E_FAIL;
          }
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/**     Variable_PutValue
 *
 * Variable_PutValue
 *
 *      @param  iSockFd                 :       [in]  Socket descriptor
 *      @param  lhVar                   :       [in]  robot handle
 *      @param  iType                   :       [in]  variable type
 *      @param  lArrays                 :       [in]  array counter
 *      @param  *pVntValue              :       [in]  value stored pointer
 *      @retval BCAP_HRESULT
 */
BCAP_HRESULT BCap::VariablePutValue(uint32_t lhVar, uint16_t iType, uint32_t lArrays, void *pVntValue)
{
  BCAP_PACKET *pSndPacket;
  BCAP_PACKET *pRecPacket;
  BCAP_ARG *pArg;

  BCAP_HRESULT hr = BCAP_E_FAIL;

  pSndPacket = Packet_Create(BCAP_FUNC_Variable_PutValue); /* BCAP_FUNC_Variable_PutValue */
  if (pSndPacket != NULL)
  {

    {
      pArg = Arg_Create(VT_I4, 1, 4, &lhVar);
      if (pArg != NULL)
      {
        Packet_AddArg(pSndPacket, pArg);
      }
    }

    {
      uint32_t lDataLen = 0;

      if ((iType & ~VT_ARRAY) == VT_BSTR)
      { /* Mask "Array" *//* IMPL:Not Support String array now. */
        /* String data */

        uint8_t buff[LOCALBUFFER_SZ];
        uint32_t lLen;

        lLen = copyToBSTR(buff, (const char *)pVntValue);
        pArg = Arg_Create(VT_BSTR, 1, lLen, buff);
        if (pArg != NULL)
        {
          Packet_AddArg(pSndPacket, pArg);
        }
      }
      else
      {
        /* Not string data */

        uint32_t lLen;
        lLen = sizeOfVarType((uint16_t)(iType & ~VT_ARRAY));
        lDataLen = lLen * lArrays;
        if (lDataLen != 0)
        {
          uint32_t i;
          uint8_t *pSrcData = (uint8_t *)pVntValue;
          uint8_t *pDstData = (uint8_t *)bMalloc(lDataLen);
          if (pDstData != NULL)
          {
            uint8_t *pDstPtr = pDstData;
            for (i = 0; i < lArrays; i++)
            {

              copyValue(pDstPtr, pSrcData, lLen);
              pDstPtr += lLen;
              pSrcData += lLen;
            }

            pArg = Arg_Create(iType, lArrays, lDataLen, pDstData);
            if (pArg != NULL)
            {
              Packet_AddArg(pSndPacket, pArg);
            }
            bFree(pDstData);
          }
        }
        else
        { /* lDataLen = 0,then Unknown data type */
          Packet_Release(pSndPacket);
          return (BCAP_E_INVALIDARGTYPE);
        }
      }

    }

    {
      pRecPacket = Packet_Create(BCAP_S_OK);
      if (pRecPacket != NULL)
      {
        hr = SendAndReceive(pSndPacket, pRecPacket);
        if SUCCEEDED(hr)
        {
          ;
        }

      }
      Packet_Release(pRecPacket);
    }
    Packet_Release(pSndPacket);
  }

  return hr;
}

/*--------------------------------------------------------------------
 b-Cap library utility functions
 --------------------------------------------------------------------*/

/**     Send b-Cap packet
 *
 * Convert Send a b-Cap packet and Receive a Packet
 *
 *      @param  iSd     :       [in]  Socket descriptor
 *      @param  pPacket         :       [in]  bCap packet data to send
 *      @retval BCAP_HRESULT is returned.
 *
 */
BCAP_HRESULT BCap::PacketSend(BCAP_PACKET *pPacket)
{

  BCAP_HRESULT hr = BCAP_E_FAIL;

  void *pbSendData;
  uint32_t length = (m_do_crc) ? (pPacket->lMsgLength + 2) : (pPacket->lMsgLength);

  pbSendData = bMalloc(length);
  if (pbSendData != NULL)
  {
    if (Packet_Serialize(pPacket, pbSendData) == BCAP_S_OK)
    {

      hr = SendBinary((uint8_t *)pbSendData, length); /* Send packet  */
      if (hr == BCAP_S_OK)
      {
        ;
      }
    }
    bFree(pbSendData);
  }

  return hr;
}

/**     Send and Receive bCap Packet
 *
 * Send a b-Cap packet and Receive a Packet
 *
 *      @param  pSndPacket              :       [in]  Send packet pointer
 *      @param  pRecPacket              :       [out]  Received packet pointer
 *      @retval BCAP_HRESULT
 *
 */

BCAP_HRESULT BCap::SendAndReceive(BCAP_PACKET *pSndPacket, BCAP_PACKET *pRecPacket)
{

  BCAP_HRESULT hr = BCAP_E_FAIL;

  if ((pRecPacket != NULL) && (pSndPacket != NULL))
  {

    if (PacketSend(pSndPacket) == BCAP_S_OK)
    {
      //printf("-----1\n");
      uint8_t *pRec = ReceivePacket(); /* Receive packet and alloc memory for storing received binary */
      //printf("-----2\n");
      if (pRec != NULL)
      {
        hr = Packet_Deserialize(pRec, pRecPacket);
        if SUCCEEDED(hr)
        {
          hr = (BCAP_HRESULT)pRecPacket->lFuncID;
        }

        bFree(pRec); /* Free received packet */
      }
      else
      {
        /* allocation error */
        hr = BCAP_E_INVALIDRCVPACKET;
      }

    }
    else
    {
      /* send error  */
      hr = BCAP_E_UNEXPECTED;
    }
  }
  else
  {
    /* Null pointer is asigned */
    hr = BCAP_E_FAIL;
  }

  return hr;

}


/**     Packet_GetLastArgHandle
 *
 * Get last argument pointer in the packet
 *
 *      @param  *pPacket        :       [in]  packet pointer
 *      @retval The last arg pointer is returned.
 *
 */
BCAP_ARG **BCap::Packet_GetLastArgHandle(BCAP_PACKET *pPacket)
{
  BCAP_ARG **pLastArg = NULL;
  BCAP_ARG *pArg;

  if (pPacket != NULL)
  {

    pArg = pPacket->pArg; /* set base pointer in packet struct */
    pLastArg = &(pPacket->pArg); /* if this is NULL in this time, this packet has no argment.
     So, pPacket->pArg is set as the last arg. */

    /* search the last arg pointer */
    while (pArg != NULL)
    {
      pLastArg = &(pArg->pNextArg);

      pArg = pArg->pNextArg; /* set next arg pointer. it might be NULL */
    }
  }

  if (pLastArg != NULL)
  {
    return pLastArg;
  }
  else
  {
    return NULL;
  }
}

/**     Packet_AddArg
 *
 * Add the arg into the packet
 *
 *      @param  *pPacket        :       [in]  Packet pointer .
 *      @param  *pNewArg        :       [in]  Arg pointer should be added into pPacket.
 *      @retval BCAP_HRESULT is returned.
 *
 */
BCAP_HRESULT BCap::Packet_AddArg(BCAP_PACKET *pPacket, BCAP_ARG *pNewArg)
{

  BCAP_HRESULT hr = BCAP_S_OK;
  BCAP_ARG **pLastArg = NULL;

  if ((pPacket != NULL) && (pNewArg != NULL))
  {
    pLastArg = Packet_GetLastArgHandle(pPacket); /* get a pointer of the last arg. */
    if (pLastArg == NULL)
    {

      /* Bad pointer */
      hr = BCAP_E_FAIL;
    }
    else
    {
      /* Not NULL: then success to find the last arg */
      *pLastArg = pNewArg; /* Set the new arg pointer as the next arg pointer */
    }

    pPacket->iArgs++; /* Increment arg counter of this packet */

    pPacket->lMsgLength += (pNewArg->lLength + BCAP_SIZE_ARGLEN);
    /* Add arg size into Packet length */
    hr = BCAP_S_OK;
  }
  else
  {

    /* Bad pointer */
    hr = BCAP_E_FAIL;
  }

  return hr;
}

/**     Packet_Release
 *
 * Release packet and all args in recursive.
 *
 *      @param  *pPacket        :       [in]  Packet pointer .
 *      @retval void
 *
 */
void BCap::Packet_Release(BCAP_PACKET *pPacket)
{

  /* release poacket  */
  if (pPacket != NULL)
  {
    /* release all args  */
    Arg_Release(pPacket->pArg);

    /* release instance of the packet */
    bFree(pPacket);
  }
}

/**     Packet_Create
 *
 * Create and allocate new packet structure.
 *
 *      @param  lFuncID         :       [in]  Function ID.
 *      @retval New packet pointer is returned.
 *
 */
BCAP_PACKET *BCap::Packet_Create(uint32_t lFuncID)
{

  BCAP_PACKET *pNewPacket = NULL;

  pNewPacket = (BCAP_PACKET *)bMalloc(sizeof(BCAP_PACKET)); /* alloc packet */
  if (pNewPacket != NULL)
  {

    pNewPacket->iArgs = 0; /* args count */
    pNewPacket->iReserved = 0;
    pNewPacket->iSerialNo = m_iSerialNo;
    pNewPacket->lFuncID = lFuncID;

    pNewPacket->lMsgLength = BCAP_SIZE_BASE;

    pNewPacket->pArg = NULL;

    m_iSerialNo++; /* increment serial number */
    if (m_iSerialNo == 0)
    {
      m_iSerialNo = 1; /* Not use Zero as a serial number */
    }
  }

  return pNewPacket;
}

/**     Serialize from packet into byte arrays.
 *
 * Change from struct into binary array.
 *
 *      @param  *pSrcPacket             :       [in]  Packet pointer to send.
 *      @param  *pDstBinData            :       [out]  byte pointer to store packet
 *      @retval BCAP_HRESULT is returned.
 *
 */
BCAP_HRESULT BCap::Packet_Serialize(BCAP_PACKET *pSrcPacket, void *pDstBinData)
{

  BCAP_HRESULT hr = BCAP_S_OK;
  uint8_t *pDstPtr;

  pDstPtr = (uint8_t *)pDstBinData;

  /* SOH */
  *pDstPtr = BCAP_SOH;
  pDstPtr += BCAP_SIZE_SOH;
  /* Header */
  uint32_t length = (m_do_crc) ? (pSrcPacket->lMsgLength + 2) : (pSrcPacket->lMsgLength);
  pDstPtr += copyValue(pDstPtr, &(length), sizeof(length));
  pDstPtr += copyValue(pDstPtr, &(pSrcPacket->iSerialNo), sizeof(pSrcPacket->iSerialNo));
  pDstPtr += copyValue(pDstPtr, &(pSrcPacket->iReserved), sizeof(pSrcPacket->iReserved));
  pDstPtr += copyValue(pDstPtr, &(pSrcPacket->lFuncID), sizeof(pSrcPacket->lFuncID));
  pDstPtr += copyValue(pDstPtr, &(pSrcPacket->iArgs), sizeof(pSrcPacket->iArgs));

  { /* Copy args */
    unsigned int i, j;
    BCAP_ARG *pArgPtr;
    uint8_t *pbValue;
    pArgPtr = pSrcPacket->pArg;

    for (i = 0; i < pSrcPacket->iArgs; i++)
    {
      if (pArgPtr != NULL)
      {
        {
          pDstPtr += copyValue(pDstPtr, &(pArgPtr->lLength), sizeof(pArgPtr->lLength));
          pDstPtr += copyValue(pDstPtr, &(pArgPtr->iType), sizeof(pArgPtr->iType));
          pDstPtr += copyValue(pDstPtr, &(pArgPtr->lArrays), sizeof(pArgPtr->lArrays));

          pbValue = (uint8_t *)pArgPtr->data; /* value stored pointer is set */

          for (j = 0; j < pArgPtr->lArrays; j++)
          {
            uint32_t lValueSize;
            uint16_t iVarType = pArgPtr->iType & ~VT_ARRAY; /* Mask "Array" */
            switch (iVarType)
            {
              case VT_UI1:
              case VT_I2:
              case VT_UI2:
              case VT_I4:
              case VT_UI4:
              case VT_R4:
              case VT_R8:
              case VT_BOOL:
              {
                lValueSize = sizeOfVarType(iVarType);
                pDstPtr += copyValue(pDstPtr, pbValue, lValueSize);
                pbValue += lValueSize; /* value stored pointer is added */
              }
                break;

              case VT_BSTR:
              {
                uint32_t lStrLen;
                copyValue(&lStrLen, pbValue, BCAP_SIZE_ARGSTRLEN);
                pDstPtr += copyValue(pDstPtr, &lStrLen, BCAP_SIZE_ARGSTRLEN); /* Set String length (4 bytes) */
                pbValue += BCAP_SIZE_ARGSTRLEN; /* value stored pointer is added */

                pDstPtr += copyValue(pDstPtr, pbValue, lStrLen); /* Set string data */
                pbValue += lStrLen;
              }
                break;

              case VT_VARIANT:
                /* NOT_IMPLEMENT */
              {

              }

                break;

              default:
                break;
            }
          }

        }
        pArgPtr = pArgPtr->pNextArg; /* Set pointer to the next arg. */
      }
    }

  }

  if (m_do_crc)
  {
    *((unsigned short*)pDstPtr) = CRC16_BlockChecksum(((char*)pDstBinData) + 1, pSrcPacket->lMsgLength - 2);
    pDstPtr += 2;
  }

  /* EOT */
  *pDstPtr = BCAP_EOT;
  pDstPtr += BCAP_SIZE_EOT;

  return hr;
}

/** Deserialize from byte array into packet.
 *
 * Change from byte arrays into struct.
 *
 *      @param  *pSrcBinData            :       [in]  Byte pointer to send.
 *      @param  *pDstPacket                     :       [out]  packet pointer to store
 *      @retval BCAP_HRESULT is returned.
 *      @detail bin  ---> struct
 *                      Note:   If something wrong, then release this packet and included args.
 */

BCAP_HRESULT BCap::Packet_Deserialize(void *pSrcBinData, BCAP_PACKET *pDstPacket)
{

  BCAP_HRESULT hr = BCAP_S_OK;
  uint8_t *pSrcPtr;

  uint16_t iArgs;
  uint32_t lMsgLength;

  pSrcPtr = (uint8_t *)pSrcBinData;

  /* SOH */
  /* pDstPtr += copyValue( &(pDstPacket->Header), pDstPtr, BCAP_SIZE_SOH)); */
  pSrcPtr += BCAP_SIZE_SOH;

  /* Header */
  /* "pkt->lMsgLength" and "pkt->lArgs" are calcurated in Packet_AddArg()  automatically */
  pSrcPtr += copyValue(&lMsgLength, pSrcPtr, sizeof(pDstPacket->lMsgLength)); /*  calcurated in Packet_AddArg()  automatically */
  pSrcPtr += copyValue(&(pDstPacket->iSerialNo), pSrcPtr, sizeof(pDstPacket->iSerialNo));
  pSrcPtr += copyValue(&(pDstPacket->iReserved), pSrcPtr, sizeof(pDstPacket->iReserved));
  pSrcPtr += copyValue(&(pDstPacket->lFuncID), pSrcPtr, sizeof(pDstPacket->lFuncID));
  pSrcPtr += copyValue(&iArgs, pSrcPtr, sizeof(pDstPacket->iArgs)); /*  calcurated in Packet_AddArg()  automatically */

  { /* Copy args */
    unsigned int i;
    BCAP_ARG *pArgPtr;
    pArgPtr = pDstPacket->pArg;

    for (i = 0; i < iArgs; i++)
    {
      uint32_t lDataSize; /* size of "*data" */
      uint32_t lLength; /* size of argument block */
      uint16_t iType;
      uint32_t lArrays;

      pSrcPtr += copyValue(&lLength, pSrcPtr, sizeof(lLength)); /* size of a argument block  */
      pSrcPtr += copyValue(&iType, pSrcPtr, sizeof(iType));
      pSrcPtr += copyValue(&lArrays, pSrcPtr, sizeof(lArrays));

      lDataSize = lLength - BCAP_SIZE_ARGTYPE - BCAP_SIZE_ARGARRAYS; /* size of "*data" */

      pArgPtr = Arg_Create(iType, lArrays, lDataSize, pSrcPtr); /* Create new arg */
      if (pArgPtr != NULL)
      {
        pSrcPtr = pSrcPtr + lDataSize; /* skip "*data"  */
      }
      else
      {
        hr = BCAP_E_FAIL;
        break;
      }

      if (Packet_AddArg(pDstPacket, pArgPtr) != BCAP_S_OK)
      { /* Add new arg to packet */
        Arg_Release(pArgPtr); /* Fail to add, then release this arg. */
        hr = BCAP_E_FAIL;
        break;
      }
    }

  }

  if (m_do_crc)
  {
    unsigned short crc = CRC16_BlockChecksum(((char*)pSrcBinData) + 1, pDstPacket->lMsgLength - 2);
    unsigned short src_crc = *((unsigned short*)pSrcPtr);
    if (src_crc != crc)
    {
      hr = BCAP_E_INVALIDRCVPACKET;
    }
    pSrcPtr += 2;
  }

  /* EOT */
  if (hr == BCAP_S_OK)
  {
    if (*pSrcPtr != BCAP_EOT)
    { /* If end of the binnary is not EOT, then error */
      hr = BCAP_E_UNEXPECTED;
    }
  }
  pSrcPtr += BCAP_SIZE_EOT;

  return hr;
}

/**     Arg_Create
 *
 * Create and allocate b-Cap argument structure
 *
 *      @param  iType           :       [in]  Variable type
 *      @param  lArrays         :       [in]  Arrays
 *      @param  lDataSize       :       [in]  total byte of ( "*data" )
 *      @param   *data          :       [in]  value pointer
 *      @retval allocated pointer is returned
 *
 *      @detail Note:When BSTR is used, *data must be store "Length:4byte" + "DoubleByte String"
 *                                      See alose function CopyToBSTR().
 */
BCAP_ARG *BCap::Arg_Create(uint16_t iType, uint32_t lArrays, uint32_t lDataSize, void *data)
{

  BCAP_ARG *pNewArg = NULL;

  if ((0 < lDataSize) && (lDataSize < BCAP_MAX_ARG_SIZE))
  { /* (0 < ) has something data, (<BCAP_MAX_ARG_SIZE) not over size */

    pNewArg = (BCAP_ARG *)bMalloc(sizeof(BCAP_ARG)); /* alloc argument */
    if (pNewArg != NULL)
    {
      pNewArg->iType = iType;
      pNewArg->lArrays = lArrays;
      pNewArg->lLength = lDataSize + BCAP_SIZE_ARGTYPE + BCAP_SIZE_ARGARRAYS;
      pNewArg->pNextArg = NULL;
      pNewArg->data = NULL;

      pNewArg->data = bMalloc(lDataSize);
      if (pNewArg->data != NULL)
      {
        if (data != NULL)
        {
          memcpy(pNewArg->data, data, lDataSize);
        }
        else
        {
          /* Fail to copy from *data, then release this argumant.  */
          Arg_Release(pNewArg);
          pNewArg = NULL; /* return value (allocated pointer) */
        }

      }
      else
      { /* fail to alloc memory */
        /* Fail to alloc memory, then release this argumant.  */
        Arg_Release(pNewArg);
        pNewArg = NULL; /* return value (allocated pointer) */
      }
    }
  }

  return pNewArg;
}

/**     Arg_Release
 *
 *      Release all args in recursive
 *
 *      @param  *pArg           :       [in]  Arg pointer
 *      @retval void
 *
 */
void BCap::Arg_Release(BCAP_ARG *pArg)
{

  while (pArg != NULL)
  {
    BCAP_ARG *pNextArg;

    pNextArg = pArg->pNextArg; /* store next pointer in temporary */

    if (pArg->data != NULL)
    {
      bFree(pArg->data); /* free value of argument */
    }

    bFree(pArg); /* free argument */
    pArg = pNextArg; /* set next pointer */
  }
}

/**     get size of variable
 *
 * get size of variable
 *
 * @param       iType           :       [in]  Variable type
 * @retval      size of variable.
 * @detail      Note 1: If (iType = VT_BSTR), then this function returns BCAP_SIZE_ARGSTRLEN (= 4 bytes)
 *                      Note 2: Not support VT_VARIANT,VT_EMPTY,VT_NULL,VT_ERROR,VT_CY,VT_DATE
 */
uint32_t BCap::sizeOfVarType(uint16_t iType)
{

  uint32_t lValueSize = 0;

  switch (iType & ~VT_ARRAY)
  {
    case VT_EMPTY:
      lValueSize = 0;   //added 20120621
      break;
    case VT_UI1:
      lValueSize = 1;
      break;
    case VT_I2:
      lValueSize = 2;
      break;
    case VT_UI2:
      lValueSize = 2;
      break;
    case VT_I4:
      lValueSize = 4;
      break;
    case VT_UI4:
      lValueSize = 4;
      break;
    case VT_R4:
      lValueSize = 4;
      break;
    case VT_R8:
      lValueSize = 8;
      break;
    case VT_BOOL:
      lValueSize = 2;
      break;
    case VT_BSTR:
      /* In this function  */
      lValueSize = BCAP_SIZE_ARGSTRLEN;
      break;

    case VT_VARIANT: /* Not implement */
      break;

    default: /*  Not implement */
      /*      VT_NULL          */
      /*      VT_ERROR         */
      /*      VT_CY            */
      /*      VT_DATE          */
      break;
  }

  return lValueSize;
}

/**     Convert into BSTR from AsciiZ
 *
 * Convert Send a b-Cap packet and Receive a Packet
 *
 *      @param  pDstPtr :       [out]  BSTR pointer
 *      @param  pSrcPtr         :       [in]  String pointer in AsciiZ
 *      @retval total size of BSTR is returned.
 *
 */
uint32_t BCap::copyToBSTR(uint8_t *pDstBstrPtr, const char *pSrcAsciiPtr)
{
  uint8_t *pbDst = (uint8_t *)pDstBstrPtr;
  uint8_t *pbSrc = (uint8_t *)pSrcAsciiPtr;
  uint32_t lStrLen, lLen2;
  uint32_t i;

  lStrLen = strlen((const char *)pbSrc); /* length of source string (ascii) */
  lLen2 = lStrLen * 2;

  if (pbDst != NULL)
  {
    pbDst += copyValue(pbDst, &lLen2, BCAP_SIZE_ARGSTRLEN);

    for (i = 0; i < lStrLen; i++)
    {
      *pbDst++ = *pbSrc++;
      *pbDst++ = 0;
    }

  }
  return (lStrLen * 2 + BCAP_SIZE_ARGSTRLEN);
}

/**     Convert From BSTR into AsciiZ
 *
 * Convert Send a b-Cap packet and Receive a Packet
 *
 *      @param  pDstPtr         :       [out]  String pointer in AsciiZ
 *      @param  pSrcPtr         :       [in]  BSTR pointer
 *      @retval total size of BSTR is returned.
 *
 */
uint32_t BCap::copyFromBSTR(void *pDstAsciiPtr, void *pSrcBstrPtr)
{
  uint8_t *pbDst = (uint8_t *)pDstAsciiPtr;
  uint8_t *pbSrc = (uint8_t *)pSrcBstrPtr;
  uint32_t lStrLen, lLen2;
  uint32_t i;

  copyValue(&lStrLen, pbSrc, BCAP_SIZE_ARGSTRLEN); /* Get BStr length */
  pbSrc += BCAP_SIZE_ARGSTRLEN;

  lLen2 = lStrLen / 2;

  if (pbDst != NULL)
  {
    for (i = 0; i < lLen2; i++)
    {
      *pbDst = *pbSrc;
      pbDst += 1; /* Ascii */
      pbSrc += 2; /* BSTR */

    }
    *pbDst = 0; /* Zero termination */
    lLen2++; /* +1 = (Zero termination) */
  }
  return (lLen2);
}

/*
 *      copy value in the "Little Endian"
 *
 *              alternate of htonl()/ntohl()
 */
uint32_t BCap::copyValue(void *pDst, void *pVntValue, uint32_t lLength)
{

#if defined(__BIG_ENDIAN__)

  /* SPARC/MC68xxx */

  /* copy values inversion. b-CAP is based on little-endian */
  {
    uint32_t i;

    uint8_t *pbDst;
    uint8_t *pbSrc;

    pbSrc = (uint8_t *)(pVntValue) + lLength -1;
    pbDst = (uint8_t *)pDst;

    for (i = 0; i < lLength; i++)
    {
      *pbDst++ = *pbSrc--;
    }
  }
#else

  memcpy(pDst, pVntValue, lLength);
#endif

  return lLength; /* return copied bytes size */
}

/*
 *      alternative of Malloc()
 */
void *BCap::bMalloc(size_t size)
{

  void *pPtr;

  m_lAllocCount++;
  m_lAllocSize += size;

  pPtr = malloc(size);

#ifdef DEBUG
  printf("AllocCount:%d\n",m_lAllocCount);
#endif

  return pPtr;
}

/*
 *      alternative of Free()
 */
void BCap::bFree(void *pPtr)
{

  m_lAllocCount--;
#ifdef DEBUG
  printf("AllocCount:%d\n",m_lAllocCount);
#endif
  free(pPtr);
}
