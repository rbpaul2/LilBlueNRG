
#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "app_state.h"
#include "osal.h"
#include "SDK_EVAL_Config.h"

//#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
//#else
//#define PRINTF(...)
//#endif

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
  	uuid_struct.uuid128[0] = uuid_0; uuid_struct.uuid128[1] = uuid_1; uuid_struct.uuid128[2] = uuid_2; uuid_struct.uuid128[3] = uuid_3; \
	uuid_struct.uuid128[4] = uuid_4; uuid_struct.uuid128[5] = uuid_5; uuid_struct.uuid128[6] = uuid_6; uuid_struct.uuid128[7] = uuid_7; \
	uuid_struct.uuid128[8] = uuid_8; uuid_struct.uuid128[9] = uuid_9; uuid_struct.uuid128[10] = uuid_10; uuid_struct.uuid128[11] = uuid_11; \
	uuid_struct.uuid128[12] = uuid_12; uuid_struct.uuid128[13] = uuid_13; uuid_struct.uuid128[14] = uuid_14; uuid_struct.uuid128[15] = uuid_15; \
	}while(0)

uint16_t ServHandle,
	MotionServHandle,
	TemperatureServHandle,
	TemperatureCharHandle,
	HumidityCharHandle,
	MotionDetectedCharHandle,
	BlindCurrPosCharHandle,
	BlindTargPosCharHandle,
	BlindPosStateCharHandle;

/* UUIDs */
Service_UUID_t  blind_service_uuid,
				motion_service_uuid,
				temperature_service_uuid;
Char_UUID_t char_uuid;

/*******************************************************************************
* Function Name  : Add_ChatService
* Description    : Add the 'Accelerometer' service.
* Input          : None
* Return         : Status.
*******************************************************************************/
uint8_t Add_ChatService(void)
{
  uint8_t ret;

  /*
  UUIDs:
  Service  			FF1738E0-B19E-11E2-9E96-0800200C9A66

  Temperature  		001738F1-B19E-11E2-9E96-0800200C9A66
  Humidity 			001738F2-B19E-11E2-9E96-0800200C9A66
  MotionDetected 	001738F3-B19E-11E2-9E96-0800200C9A66

  BlindCurrPos 		111738B1-B19E-11E2-9E96-0800200C9A66
  BlindTargPos 		111738B2-B19E-11E2-9E96-0800200C9A66
  BlindPosState 	111738B2-B19E-11E2-9E96-0800200C9A66
  */
  const uint8_t uuid[16] = 						{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
		  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xe0, 0x38, 0x17, 0xFF };
  const uint8_t uuidmotion[16] = 				{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
  		  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
  												  0xe0, 0x38, 0x17, 0xF0 };
  const uint8_t uuidtemp[16] = 				{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
    		  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
    										      0xe0, 0x38, 0x17, 0xF1 };
  const uint8_t charUuidTemperature[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
		  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xf1, 0x38, 0x17, 0x00 };
  const uint8_t charUuidHumidity[16] = 			{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xf2, 0x38, 0x17, 0x00 };
  const uint8_t charUuidMotionDetected[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
  	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xf3, 0x38, 0x17, 0x00 };
  const uint8_t charUuidBlindCurrPos[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
						  	  	  	  	  	  	  0xb1, 0x38, 0x17, 0x11 };
  const uint8_t charUuidBlindTargPos[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xb2, 0x38, 0x17, 0x11 };
  const uint8_t charUuidBlindPosState[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
  	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xb3, 0x38, 0x17, 0x11 };

  Osal_MemCpy(&blind_service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &blind_service_uuid, PRIMARY_SERVICE, 17 /* numAttributeRecords */, &ServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&motion_service_uuid.Service_UUID_128, uuidmotion, 16);
    ret = aci_gatt_add_service(UUID_TYPE_128, &motion_service_uuid, PRIMARY_SERVICE, 9 /* numAttributeRecords */, &MotionServHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&temperature_service_uuid.Service_UUID_128, uuidtemp, 16);
	ret = aci_gatt_add_service(UUID_TYPE_128, &temperature_service_uuid, PRIMARY_SERVICE, 9 /* numAttributeRecords */, &TemperatureServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidBlindCurrPos, 16);
  ret =  aci_gatt_add_char(ServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                16, 1, &BlindCurrPosCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] BlindCurrPos UUID[12] = %02X\n", charUuidBlindCurrPos[12]);

//  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidHumidity, 16);
//  ret =  aci_gatt_add_char(ServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
//                16, 0, &HumidityCharHandle);
//  if (ret != BLE_STATUS_SUCCESS) goto fail;
//  PRINTF("[Added Char] Humidity UUID[12] = %02X\n", charUuidHumidity[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidMotionDetected, 16);
  ret =  aci_gatt_add_char(MotionServHandle, UUID_TYPE_128, &char_uuid, 1, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
				16, 0, &MotionDetectedCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] MotionDetected UUID[12] = %02X\n", charUuidMotionDetected[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTemperature, 16);
  ret =  aci_gatt_add_char(TemperatureServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                  16, 0, &TemperatureCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Temperature UUID[12] = %02X\n", charUuidTemperature[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidBlindTargPos, 16);
  ret =  aci_gatt_add_char(ServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                16, 1, &BlindTargPosCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] BlindTargPos UUID[12] = %02X\n", charUuidBlindTargPos[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidBlindPosState, 16);
  ret =  aci_gatt_add_char(ServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                16, 1, &BlindPosStateCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] BlindPosState UUID[12] = %02X\n", charUuidBlindPosState[12]);

  printf("[Added Service]\n\n");
  return BLE_STATUS_SUCCESS; 

fail:
  printf("Error while adding Chat service.\n");
  return BLE_STATUS_ERROR ;
}

void Update_Characteristic_Val( uint16_t Service_Handle,
        						uint16_t Char_Handle,
								uint8_t Val_Offset,
								uint8_t Char_Value_Length,
								uint8_t Char_Value[])
{
	struct timer t;
	Timer_Set(&t, CLOCK_SECOND*10);
	while(aci_gatt_update_char_value(Service_Handle, Char_Handle, Val_Offset, Char_Value_Length, Char_Value)==BLE_STATUS_INSUFFICIENT_RESOURCES) {
		APP_FLAG_SET(TX_BUFFER_FULL);
		while(APP_FLAG(TX_BUFFER_FULL)) {
			BTLE_StackTick();
			// Radio is busy (buffer full).
			if(Timer_Expired(&t))
				break;
		}
	}
}

/*******************************************************************************
* Function Name  : Attribute_Modified_CB
* Description    : Attribute modified callback.
* Input          : Attribute handle modified.
*                  Length of the data.
*                  Attribute data.
* Return         : None.
*******************************************************************************/
void Attribute_Modified_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data)
{
	if(handle == BlindTargPosCharHandle + 1)
	{

		for(int i = 0; i < data_length; i++)
			printf("%d \n", att_data[i]);
		Update_Characteristic_Val(ServHandle,BlindCurrPosCharHandle,0,data_length,att_data);

	}
//	else if(handle == TemperatureCharHandle + 1)
//	{
//		for(int i = 0; i < data_length; i++)
//			printf("%d \n", att_data[i]);
//		Update_Characteristic_Val(ServHandle,TemperatureCharHandle,0,data_length,att_data);
//	}
	else if(handle == MotionDetectedCharHandle + 2)
	{
		if(att_data[0] == 0x01)
			APP_FLAG_SET(NOTIFICATIONS_ENABLED);
	}
}


