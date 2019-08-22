/******************************************************************************

 @file  broadcaster.h

 @brief TI BLE GAP Broadcaster Role for for RTOS Applications

        This GAP profile only advertises.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2011-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/

#ifndef BROADCASTER_H
#define BROADCASTER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------------------------------------------------
 * INCLUDES
 */
#include "gap.h"
/*-------------------------------------------------------------------
 * CONSTANTS
 */

/** @defgroup GAPROLE_PROFILE_PARAMETERS GAP Role Parameters
 * @{
 */
#define GAPROLE_PROFILEROLE         0x300  //!< Reading this parameter will return GAP Role type. Read Only. Size is uint8_t.
#define GAPROLE_BD_ADDR             0x301  //!< Device's Address. Read Only. Size is uint8_t[B_ADDR_LEN]. This item is read from the controller.
#define GAPROLE_ADVERT_ENABLED      0x302  //!< Enable/Disable Advertising. Read/Write. Size is uint8_t. Default is TRUE=Enabled.
#define GAPROLE_ADVERT_OFF_TIME     0x303  //!< Advertising Off Time for Limited advertisements (in milliseconds). Read/Write. Size is uint16_t. Default is 30 seconds.
#define GAPROLE_ADVERT_DATA         0x304  //!< Advertisement Data. Read/Write. Size is uint8_t[B_MAX_ADV_LEN].  Default is "02:01:01", which means that it is a Limited Discoverable Advertisement.
#define GAPROLE_SCAN_RSP_DATA       0x305  //!< Scan Response Data. Read/Write. Size is uint8_t[B_MAX_ADV_LEN]. Defaults to all 0.
#define GAPROLE_ADV_EVENT_TYPE      0x306  //!< Advertisement Type. Read/Write. Size is uint8_t.  Default is GAP_ADTYPE_ADV_IND (defined in GAP.h).
#define GAPROLE_ADV_DIRECT_TYPE     0x307  //!< Direct Advertisement Address Type. Ready/Write. Size is uint8_t. Default is ADDRMODE_PUBLIC (defined in GAP.h).
#define GAPROLE_ADV_DIRECT_ADDR     0x308  //!< Direct Advertisement Address. Read/Write. Size is uint8_t[B_ADDR_LEN]. Default is NULL.
#define GAPROLE_ADV_CHANNEL_MAP     0x309  //!< Which channels to advertise on. Read/Write Size is uint8_t. Default is GAP_ADVCHAN_ALL (defined in GAP.h)
#define GAPROLE_ADV_FILTER_POLICY   0x30A  //!< Filter Policy. Ignored when directed advertising is used. Read/Write. Size is uint8_t. Default is GAP_FILTER_POLICY_ALL (defined in GAP.h).
#define GAPROLE_MAX_SCAN_RES        0x404
/** @} End GAPROLE_PROFILE_PARAMETERS */

/*-------------------------------------------------------------------
 * TYPEDEFS
 */

/**
 * GAP Broadcaster Role States.
 */
typedef enum
{
  GAPROLE_INIT = 0,                       //!< Waiting to be started
  GAPROLE_STARTED,                        //!< Started but not advertising
  GAPROLE_ADVERTISING,                    //!< Currently Advertising
  GAPROLE_WAITING,                        //!< Device is started but not advertising, is in waiting period before advertising again
  GAPROLE_ERROR                           //!< Error occurred - invalid state
} gaprole_States_t;

typedef union
{
  gapEventHdr_t             gap;          //!< @ref GAP_MSG_EVENT and status.
  gapDeviceInitDoneEvent_t  initDone;     //!< GAP initialization done.
  gapDeviceInfoEvent_t      deviceInfo;   //!< Discovery device information event structure.
  gapDevDiscEvent_t         discCmpl;     //!< Discovery complete event structure.
} gapPeriObsRoleEvent_t;

/*-------------------------------------------------------------------
 * MACROS
 */

/*-------------------------------------------------------------------
 * Profile Callbacks
 */

/**
 * Callback when the device has been started.  Callback event to
 * the Notify of a state change.
 */
typedef void (*gapRolesStateNotify_t)(gaprole_States_t newState);

   /**
 * @brief SPO Event Callback Function
 *
 * @param pEvent Pointer to event structure
 */
typedef uint8_t (*passThroughToApp_t)
(
  gapPeriObsRoleEvent_t *pEvent
);

/**
 * Callback structure - must be setup by the application and used when
 *                      GAPRole_StartDevice() is called.
 */
typedef struct
{
  gapRolesStateNotify_t    pfnStateChange;  //!< Whenever the device changes state
  passThroughToApp_t       pfnPassThrough;  //!< When the app should decide on a param update request
} gapRolesCBs_t;

/*-------------------------------------------------------------------
 * API FUNCTIONS
 */

/**
 * @defgroup GAPROLES_BROADCASTER_API GAP Broadcaster Role API Functions
 *
 * @{
 */

/**
 * @brief       Set a GAP Role parameter.
 *
 *  NOTE: You can call this function with a GAP Parameter ID and it will set the
 *        GAP Parameter.  GAP Parameters are defined in (gap.h).  Also,
 *        the "len" field must be set to the size of a "uint16_t" and the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param       param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS
 * @param       len - length of data to write
 * @param       pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return      SUCCESS or INVALIDPARAMETER (invalid paramID)
 */
extern bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue);

/**
 * @brief       Get a GAP Role parameter.
 *
 *  NOTE: You can call this function with a GAP Parameter ID and it will get a
 *        GAP Parameter.  GAP Parameters are defined in (gap.h).  Also, the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param       param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS
 * @param       pValue - pointer to location to get the value.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return      SUCCESS or INVALIDPARAMETER (invalid paramID)
 */
extern bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue);

/**
 * @brief       Does the device initialization.  Only call this function once.
 *
 * @param       pAppCallbacks - pointer to application callbacks.
 *
 * @return      SUCCESS or bleAlreadyInRequestedMode
 */
extern bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks);

/**
 * @} End GAPROLES_BROADCASTER_API
 */

/**
 * @brief   Start a device discovery scan.
 *
 * @param   mode discovery mode: @ref GAP_Discovery
 * @param   activeScan TRUE to perform active scan
 * @param   whiteList TRUE to only scan for devices in the white list
 *
 * @return  @ref SUCCESS : Discovery discovery has started.
 * @return  @ref bleIncorrectMode : Invalid profile role.
 * @return  @ref bleAlreadyInRequestedMode : Device discovery already started
 * @return  @ref HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS : bad parameter
 */
extern bStatus_t GAPRole_StartDiscovery(uint8_t mode, uint8_t activeScan, uint8_t whiteList);

/**
 * @brief   Cancel a device discovery scan.
 *
 * @return  @ref SUCCESS : Cancel started.
 * @return  @ref bleInvalidTaskID : Not the task that started discovery.
 * @return  @ref bleIncorrectMode : Not in discovery mode.
 */
extern bStatus_t GAPRole_CancelDiscovery(void);

/*-------------------------------------------------------------------
 * TASK FUNCTIONS - Don't call these. These are system functions.
 */

/**
 * @brief       Task creation function for the GAP Peripheral Role.
 *
 * @param       none
 *
 * @return      none
 */
extern void GAPRole_createTask(void);

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* BROADCASTER_H */
