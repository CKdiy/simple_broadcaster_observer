/**
  @headerfile: taginf.h
  $Date: 2019-08-22 $
  $Revision:    $
*/

#ifndef TAGINF_H
#define TAGINF_H

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"

/*********************************************************************
 * CONSTANTS
 */
#define TAG_ADV_DATA_LEN       30 
#define TAG_ADV_UUID_OFFSET    9
#define TAG_ADV_MAJOR_OFFSET   25

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */

typedef struct
{
    uint8_t mac[B_ADDR_LEN];
}tag_mac_struct;

typedef struct
{
	uint8_t major[2];
	uint8_t minor[2];
	uint8_t rxp;
	int8_t  rssi;
}tag_inf_struct;

#endif /* TAGINF_H */
