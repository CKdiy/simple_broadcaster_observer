/******************************************************************************

 @file  simple_broadcaster.c

 @brief This file contains the Simple BLE Broadcaster sample application for
        use with the CC2650 Bluetooth Low Energy Protocol Stack.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"

#include "broadcaster.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include "simple_broadcaster.h"

#include "taginf.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

   // Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  10

// Scan parameters
#define DEFAULT_SCAN_DURATION_600ms           600

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE
   
// Task configuration
#define SBB_TASK_PRIORITY                     1

#ifndef SBB_TASK_STACK_SIZE
#define SBB_TASK_STACK_SIZE                   660
#endif

// Internal Events for RTOS application
#define SBB_STATE_CHANGE_EVT                  0x0001
#define SBB_KEY_CHANGE_EVT                    0x0002
#define SBP_OBSERVER_STATE_EVT                0x0004
#define SBP_PERIODIC_EVT                      0x0008

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               2000
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header.
  uint8_t *pData;  // event data
} sbbEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbbTask;
Char sbbTaskStack[SBB_TASK_STACK_SIZE];

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// events flag for internal application events.
static uint16_t events;

tag_mac_struct tag_mac_list[DEFAULT_MAX_SCAN_RES];
tag_inf_struct tag_inf_list[DEFAULT_MAX_SCAN_RES];

static uint8 tag_count;    //Number of single discoveries
static uint8 tag_index;   

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x15,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'B',
  'L',
  'E',
  'B',
  'r',
  'o',
  'a',
  'd',
  'c',
  'a',
  's',
  't',
  'e',
  'r',

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

#ifndef BEACON_FEATURE

  // three-byte broadcast of the data "1 2 3"
  0x04,   // length of this data including the data type byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
  1,
  2,
  3

#else

  // 25 byte beacon advertisement data
  // Preamble: Company ID - 0x000D for TI, refer to https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
  // Data type: Beacon (0x02)
  // Data length: 0x15
  // UUID: 00000000-0000-0000-0000-000000000000 (null beacon)
  // Major: 1 (0x0001)
  // Minor: 1 (0x0001)
  // Measured Power: -59 (0xc5)
  0x1A, // length of this data including the data type byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
  0x0D, // Company ID - Fixed
  0x00, // Company ID - Fixed
  0x02, // Data Type - Fixed
  0x15, // Data Length - Fixed
  0x00, // UUID - Variable based on different use cases/applications
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // UUID
  0x00, // Major
  0x01, // Major
  0x00, // Minor
  0x01, // Minor
  0xc5  // Power - The 2's complement of the calibrated Tx Power

#endif // !BEACON_FEATURE
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEBroadcaster_init(void);
static void SimpleBLEBroadcaster_taskFxn(UArg a0, UArg a1);

static void SimpleBLEBroadcaster_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLEBroadcaster_processAppMsg(sbbEvt_t *pMsg);
static void SimpleBLEBroadcaster_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEBroadcaster_stateChangeCB(gaprole_States_t newState);

static void SimpleBLEBroadcasterObserver_processRoleEvent(gapPeriObsRoleEvent_t *pEvent);
static uint8_t SimpleBLEBroadcasterObserver_StateChangeCB(gapPeriObsRoleEvent_t *pEvent);
static void SimpleBLEObserver_addDeviceInfo_Ex(uint8 *pAddr, uint8 *pData, uint8 datalen, int8 rssi);
static uint8_t SimpleBLEBroadcaster_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEBroadcaster_BroadcasterCBs =
{
  SimpleBLEBroadcaster_stateChangeCB   // Profile State Change Callbacks
  ,
  SimpleBLEBroadcasterObserver_StateChangeCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_createTask
 *
 * @brief   Task creation function for the Simple BLE Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEBroadcaster_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbbTaskStack;
  taskParams.stackSize = SBB_TASK_STACK_SIZE;
  taskParams.priority = SBB_TASK_PRIORITY;

  Task_construct(&sbbTask, SimpleBLEBroadcaster_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_init
 *
 * @brief   Initialization function for the Simple BLE Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_init(void)
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup the GAP Broadcaster Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t gapRole_AdvertOffTime = 0;

#ifndef BEACON_FEATURE
    uint8_t advType = GAP_ADTYPE_ADV_SCAN_IND; // use scannable undirected adv
#else
    uint8_t advType = GAP_ADTYPE_ADV_NONCONN_IND; // use non-connectable adv
#endif // !BEACON_FEATURE
	
    // Create one-shot clocks for internal periodic events.
    Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

    {
      // Set the max amount of scan responses
      uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
      GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t),
                         &scanRes);	
	
      GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION_600ms);
      GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION_600ms);
    }
	
    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &gapRole_AdvertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof (scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType);
  }

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Start the Device
  VOID GAPRole_StartDevice(&simpleBLEBroadcaster_BroadcasterCBs);
  
  Util_startClock(&periodicClock);
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processEvent
 *
 * @brief   Application task entry point for the Simple BLE Broadcaster.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEBroadcaster_init();

  // Application main loop
  for (;;)
  {
    // Get the ticks since startup
    uint32_t tickStart = Clock_getTicks();

    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLEBroadcaster_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbbEvt_t *pMsg = (sbbEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEBroadcaster_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
	  
      if (events & SBP_PERIODIC_EVT)
      {
         events &= ~SBP_PERIODIC_EVT;
		 
         memset((void *)tag_mac_list, 0, tag_count*B_ADDR_LEN);
         memset((void *)tag_inf_list, 0, tag_count*sizeof(tag_inf_struct));
         tag_count = 0;
         tag_index = 0;
		 
         GAPRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                              DEFAULT_DISCOVERY_ACTIVE_SCAN,
                              DEFAULT_DISCOVERY_WHITE_LIST );	
	  
         Util_startClock(&periodicClock);
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
	  SimpleBLEBroadcasterObserver_processRoleEvent((gapPeriObsRoleEvent_t *)pMsg);
	  break;
	  
    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processAppMsg(sbbEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBB_STATE_CHANGE_EVT:
      SimpleBLEBroadcaster_processStateChangeEvt((gaprole_States_t)pMsg->
                                                 hdr.state);
      break;
	  
    case SBP_OBSERVER_STATE_EVT:
      SimpleBLEBroadcaster_processStackMsg((ICall_Hdr *)pMsg->pData);
      break;
	 
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEBroadcasterObserver_StateChangeCB
 *
 * @brief   Peripheral observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 */
static uint8_t SimpleBLEBroadcasterObserver_StateChangeCB(gapPeriObsRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if(pEvent->gap.opcode == GAP_DEVICE_INIT_DONE_EVENT)
  {
  	gapDeviceInitDoneEvent_t *pDevInfoMsg;
	
	if( pDevInfoMsg = ICall_malloc(sizeof(gapDeviceInitDoneEvent_t)))
	{
	  memcpy(pDevInfoMsg, pEvent, sizeof(gapDeviceInitDoneEvent_t));
	  
	  SimpleBLEBroadcaster_enqueueMsg( SBP_OBSERVER_STATE_EVT, SUCCESS, 
                                       (uint8_t *)pDevInfoMsg);
	}
	
	ICall_freeMsg(pEvent);
	
	return TRUE;
  }
  else if( (pEvent->gap.opcode == GAP_DEVICE_INFO_EVENT) |
		    (pEvent->gap.opcode == GAP_DEVICE_DISCOVERY_EVENT) )
  {
	SimpleBLEBroadcaster_enqueueMsg(SBP_OBSERVER_STATE_EVT, SUCCESS, 
                                    (uint8_t *)pEvent);
	
    // Caller should free the event
    return TRUE;
  }
  else
  {   
    // App will process and free the event
    ICall_freeMsg(pEvent);
 
    return FALSE;  
  }
}

/*********************************************************************
 * @fn      SimpleBLEBroadcasterObserver_processRoleEvent
 *
 * @brief   Peripheral Observer role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEBroadcasterObserver_processRoleEvent(gapPeriObsRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
	case GAP_DEVICE_INIT_DONE_EVENT:
	  {
		tag_count = 0;
		tag_index = 0;
		
		GAPRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                DEFAULT_DISCOVERY_WHITE_LIST );
	  }	  
	  
	  ICall_free(pEvent);
	  break;
	  
    case GAP_DEVICE_INFO_EVENT:
	  {
		if(pEvent->deviceInfo.eventType != GAP_ADRPT_SCAN_RSP)
		{
			SimpleBLEObserver_addDeviceInfo_Ex(pEvent->deviceInfo.addr,
                                               pEvent->deviceInfo.pEvtData,
                                               pEvent->deviceInfo.dataLen,
                                               pEvent->deviceInfo.rssi);
		}
	  }
	  
      ICall_freeMsg(pEvent);
      break;
	  
    case GAP_DEVICE_DISCOVERY_EVENT:
      // discovery complete
      GAPRole_CancelDiscovery();
	  
      ICall_freeMsg(pEvent);
      break;

    default:
      break;
  }
}

const uint8_t taguuid[6]={0xAB,0x81,0x90,0xD5,0xD1,0x1E};
/*********************************************************************
 * @fn      SimpleBLEObserver_addDeviceInfo_Ex
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLEObserver_addDeviceInfo_Ex(uint8 *pAddr, uint8 *pData, uint8 datalen, int8 rssi)
{
	uint8 i;
	uint8 majoroffset;
	uint8 uuidoffset;
	uint8 tagNum;
	uint8 *ptr;
	
 	if( (pAddr == NULL) || (pData == NULL) )
	  return;
	
	if( TAG_ADV_DATA_LEN != datalen)
	  return;
	
	ptr = pData;
	
	uuidoffset = TAG_ADV_UUID_OFFSET;
	
	if(memcmp(&ptr[uuidoffset], taguuid, sizeof(taguuid)) != 0)
 	  	return;
	
	tagNum = tag_count;

	   	// If result count not at max
   	if ( tagNum < DEFAULT_MAX_SCAN_RES )
   	{
		// Check if device is already in scan results
		for ( i = 0; i < tagNum; i++ )
		{
			if (memcmp(pAddr, (void *)&tag_mac_list[i], B_ADDR_LEN) == 0)
			{
				return;
			}
		}
		
		// Add addr to scan result list
		memcpy(&tag_mac_list[tagNum], pAddr, B_ADDR_LEN );

		majoroffset = TAG_ADV_MAJOR_OFFSET; 
		
		memcpy((void *)(&tag_inf_list[tagNum].major[0]), (void *)&ptr[majoroffset], 
			    sizeof(uint32) + sizeof(uint8));
		
		tag_inf_list[tagNum].rssi = rssi;
 		
		// Increment scan result count
		tag_count ++;
	}
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEBroadcaster_enqueueMsg(SBB_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t SimpleBLEBroadcaster_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData)
{
  sbbEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbbEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;
	
    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
  
  return FALSE;  
}

/*********************************************************************
 * @fn      SimpleBLEBroadcaster_processStateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void SimpleBLEBroadcaster_processStateChangeEvt(gaprole_States_t newState)
{
  switch (newState)
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      {
		;
      }
      break;

    case GAPROLE_WAITING:
      {
        ;
      }
      break;

    case GAPROLE_ERROR:
      {
        ;
      }
      break;

    default:
      {
        ;
      }
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}
/*********************************************************************
*********************************************************************/
