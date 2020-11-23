/**************************************************************************************************
 
  Shanghai QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Shanghai QST 
  Corporation ("QST"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of QST. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a QST Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/**************************************************************************************************
  Filename:       devinfoservice.c
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the Device Information service.


 **************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"

#include "devinfoservice_bell.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Device information service
CONST uint8 devInfoServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DEVINFO_SERV_UUID), HI_UINT16(DEVINFO_SERV_UUID)
};

// System ID
CONST uint8 devInfoSystemIdUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SYSTEM_ID_UUID), HI_UINT16(SYSTEM_ID_UUID)
};

// Model Number String
CONST uint8 devInfoModelNumberUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MODEL_NUMBER_UUID), HI_UINT16(MODEL_NUMBER_UUID)
};

// Serial Number String
CONST uint8 devInfoSerialNumberUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SERIAL_NUMBER_UUID), HI_UINT16(SERIAL_NUMBER_UUID)
};

// Firmware Revision String
CONST uint8 devInfoFirmwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(FIRMWARE_REV_UUID), HI_UINT16(FIRMWARE_REV_UUID)
};

// Hardware Revision String
CONST uint8 devInfoHardwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HARDWARE_REV_UUID), HI_UINT16(HARDWARE_REV_UUID)
};

// Software Revision String
CONST uint8 devInfoSoftwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SOFTWARE_REV_UUID), HI_UINT16(SOFTWARE_REV_UUID)
};

// Manufacturer Name String
CONST uint8 devInfoMfrNameUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MANUFACTURER_NAME_UUID), HI_UINT16(MANUFACTURER_NAME_UUID)
};

// IEEE 11073-20601 Regulatory Certification Data List
CONST uint8 devInfo11073CertUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(IEEE_11073_CERT_DATA_UUID), HI_UINT16(IEEE_11073_CERT_DATA_UUID)
};

// PnP ID
CONST uint8 devInfoPnpIdUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(PNP_ID_UUID), HI_UINT16(PNP_ID_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * Profile Attributes - variables
 */

// Device Information Service attribute
static CONST gattAttrType_t devInfoService = { ATT_BT_UUID_SIZE, devInfoServUUID };

// System ID characteristic
static uint8 devInfoSystemIdProps = GATT_PROP_READ;
static uint8 devInfoSystemId[DEVINFO_SYSTEM_ID_LEN] = {0, 0, 0, 0, 0, 0, 0, 0};

// Model Number String characteristic
static uint8 devInfoModelNumberProps = GATT_PROP_READ;
static uint8 devInfoModelNumber[DEVINFO_MAX_LEN] = "Model Number";

// Serial Number String characteristic
static uint8 devInfoSerialNumberProps = GATT_PROP_READ;
static const uint8 devInfoSerialNumber[DEVINFO_MAX_LEN] = "Serial Number";

// Firmware Revision String characteristic
static uint8 devInfoFirmwareRevProps = GATT_PROP_READ;
static uint8 devInfoFirmwareRev[DEVINFO_MAX_LEN] = "Firmware Revision";

// Hardware Revision String characteristic
static uint8 devInfoHardwareRevProps = GATT_PROP_READ;
static uint8 devInfoHardwareRev[DEVINFO_MAX_LEN] = "Hardware Revision";

// Software Revision String characteristic
static uint8 devInfoSoftwareRevProps = GATT_PROP_READ;
static uint8 devInfoSoftwareRev[DEVINFO_MAX_LEN] = "software Revision";

// Manufacturer Name String characteristic
static uint8 devInfoMfrNameProps = GATT_PROP_READ;
static uint8 devInfoMfrName[DEVINFO_MAX_LEN] = "Manufacturer Name";

// IEEE 11073-20601 Regulatory Certification Data List characteristic
static uint8 devInfo11073CertProps = GATT_PROP_READ;
static uint8 devInfo11073Cert[] =
{
  DEVINFO_11073_BODY_EXP,      // authoritative body type
  0x00,                       // authoritative body structure type
                              // authoritative body data follows below:
  'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l'
};

// System ID characteristic
static uint8 devInfoPnpIdProps = GATT_PROP_READ;
static uint8 devInfoPnpId[DEVINFO_PNP_ID_LEN] =
{
  1,                                      // Vendor ID source (1=Bluetooth SIG)
  LO_UINT16(0x0504), HI_UINT16(0x0504),   // Vendor ID (QST)
  LO_UINT16(0x0000), HI_UINT16(0x0000),   // Product ID (vendor-specific)
  LO_UINT16(0x0110), HI_UINT16(0x0110)    // Product version (JJ.M.N)
};

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t devInfoAttrTbl[] =
{
  // Device Information Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&devInfoService                /* pValue */
  },

    // System ID Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSystemIdProps
    },

      // System ID Value
      {
        { ATT_BT_UUID_SIZE, devInfoSystemIdUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSystemId
      },

    // Model Number String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoModelNumberProps
    },

      // Model Number Value
      {
        { ATT_BT_UUID_SIZE, devInfoModelNumberUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoModelNumber
      },

    // Serial Number String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSerialNumberProps
    },

      // Serial Number Value
      {
        { ATT_BT_UUID_SIZE, devInfoSerialNumberUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSerialNumber
      },

    // Firmware Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoFirmwareRevProps
    },

      // Firmware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoFirmwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoFirmwareRev
      },

    // Hardware Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoHardwareRevProps
    },

      // Hardware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoHardwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoHardwareRev
      },

    // Software Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSoftwareRevProps
    },

      // Software Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoSoftwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSoftwareRev
      },

    // Manufacturer Name String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoMfrNameProps
    },

      // Manufacturer Name Value
      {
        { ATT_BT_UUID_SIZE, devInfoMfrNameUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoMfrName
      },

    // IEEE 11073-20601 Regulatory Certification Data List Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfo11073CertProps
    },

      // IEEE 11073-20601 Regulatory Certification Data List Value
      {
        { ATT_BT_UUID_SIZE, devInfo11073CertUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfo11073Cert
      },

    // PnP ID Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoPnpIdProps
    },

      // PnP ID Value
      {
        { ATT_BT_UUID_SIZE, devInfoPnpIdUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoPnpId
      }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Device Info Service Callbacks
CONST gattServiceCBs_t devInfoCBs =
{
  devInfo_ReadAttrCB, // Read callback function pointer
  NULL,               // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      DevInfo_AddService
 *
 * @brief   Initializes the Device Information service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t DevInfo_AddService( void )
{
  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( devInfoAttrTbl,
                                      GATT_NUM_ATTRS( devInfoAttrTbl ),
                                      &devInfoCBs );
}

/*********************************************************************
 * @fn      DevInfo_SetParameter
 *
 * @brief   Set a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
     	case DEVINFO_SYSTEM_ID:
      	osal_memcpy(devInfoSystemId, value, len);
      	break;
     
     	case DEVINFO_MODEL_NUMBER:
      	osal_memcpy(devInfoModelNumber, value, len);
      	break;

		case DEVINFO_FIRMWARE_REV:
	  	osal_memcpy(devInfoFirmwareRev, value, len);
		break;

		case DEVINFO_HARDWARE_REV:
		osal_memcpy(devInfoHardwareRev, value, len);
		break;

		case DEVINFO_SOFTWARE_REV:
		osal_memcpy(devInfoSoftwareRev, value, len);
		break;

	 	case DEVINFO_MANUFACTURER_NAME:
		osal_memcpy(devInfoMfrName, value, len);
		break;

    	default:
      	ret = INVALIDPARAMETER;
     	break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      DevInfo_GetParameter
 *
 * @brief   Get a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case DEVINFO_SYSTEM_ID:
      osal_memcpy(value, devInfoSystemId, sizeof(devInfoSystemId));
      break;

    case DEVINFO_MODEL_NUMBER:
      osal_memcpy(value, devInfoModelNumber, sizeof(devInfoModelNumber));
      break;
    case DEVINFO_SERIAL_NUMBER:
      osal_memcpy(value, devInfoSerialNumber, sizeof(devInfoSerialNumber));
      break;

    case DEVINFO_FIRMWARE_REV:
      osal_memcpy(value, devInfoFirmwareRev, sizeof(devInfoFirmwareRev));
      break;

    case DEVINFO_HARDWARE_REV:
      osal_memcpy(value, devInfoHardwareRev, sizeof(devInfoHardwareRev));
      break;

    case DEVINFO_SOFTWARE_REV:
      osal_memcpy(value, devInfoSoftwareRev, sizeof(devInfoSoftwareRev));
      break;

    case DEVINFO_MANUFACTURER_NAME:
      osal_memcpy(value, devInfoMfrName, sizeof(devInfoMfrName));
      break;

    case DEVINFO_11073_CERT_DATA:
      osal_memcpy(value, devInfo11073Cert, sizeof(devInfo11073Cert));
      break;

    case DEVINFO_PNP_ID:
      osal_memcpy(value, devInfoPnpId, sizeof(devInfoPnpId));
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          devInfo_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch (uuid)
  {
    case SYSTEM_ID_UUID:
      // verify offset
      if (offset >= sizeof(devInfoSystemId))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length
        *pLen = MIN(maxLen, (sizeof(devInfoSystemId) - offset));

        // copy data
        osal_memcpy(pValue, &devInfoSystemId[offset], *pLen);
      }
      break;

    case MODEL_NUMBER_UUID:
      // verify offset
      if (offset >= (sizeof(devInfoModelNumber) - 1))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length (exclude null terminating character)
        *pLen = MIN(maxLen, ((sizeof(devInfoModelNumber) - 1) - offset));

        // copy data
        osal_memcpy(pValue, &devInfoModelNumber[offset], *pLen);
      }
      break;

    case SERIAL_NUMBER_UUID:
      // verify offset
      if (offset >= (sizeof(devInfoSerialNumber) - 1))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length (exclude null terminating character)
        *pLen = MIN(maxLen, ((sizeof(devInfoSerialNumber) - 1) - offset));

        // copy data
        osal_memcpy(pValue, &devInfoSerialNumber[offset], *pLen);
      }
      break;

    case FIRMWARE_REV_UUID:
      // verify offset
      if (offset >= (sizeof(devInfoFirmwareRev) - 1))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length (exclude null terminating character)
        *pLen = MIN(maxLen, ((sizeof(devInfoFirmwareRev) - 1) - offset));
        
        // copy data
        osal_memcpy(pValue, &devInfoFirmwareRev[offset], *pLen);
      }
      break;

    case HARDWARE_REV_UUID:
      // verify offset
      if (offset >= (sizeof(devInfoHardwareRev) - 1))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length (exclude null terminating character)
        *pLen = MIN(maxLen, ((sizeof(devInfoHardwareRev) - 1) - offset));

        // copy data
        osal_memcpy(pValue, &devInfoHardwareRev[offset], *pLen);
      }
      break;

    case SOFTWARE_REV_UUID:
      // verify offset
      if (offset >= (sizeof(devInfoSoftwareRev) - 1))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length (exclude null terminating character)
        *pLen = MIN(maxLen, ((sizeof(devInfoSoftwareRev) - 1) - offset));
        
        // copy data
        osal_memcpy(pValue, &devInfoSoftwareRev[offset], *pLen);
      }
      break;

    case MANUFACTURER_NAME_UUID:
      // verify offset
      if (offset >= (sizeof(devInfoMfrName) - 1))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length (exclude null terminating character)
        *pLen = MIN(maxLen, ((sizeof(devInfoMfrName) - 1) - offset));
        
        // copy data
        osal_memcpy(pValue, &devInfoMfrName[offset], *pLen);
      }
      break;

    case IEEE_11073_CERT_DATA_UUID:
      // verify offset
      if (offset >= sizeof(devInfo11073Cert))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length
        *pLen = MIN(maxLen, (sizeof(devInfo11073Cert) - offset));

        // copy data
        osal_memcpy(pValue, &devInfo11073Cert[offset], *pLen);
      }
      break;

    case PNP_ID_UUID:
      // verify offset
      if (offset >= sizeof(devInfoPnpId))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length
        *pLen = MIN(maxLen, (sizeof(devInfoPnpId) - offset));

        // copy data
        osal_memcpy(pValue, &devInfoPnpId[offset], *pLen);
      }
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}


/*********************************************************************
*********************************************************************/
