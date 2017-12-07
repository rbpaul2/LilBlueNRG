
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

uint16_t BlindServHandle,
	MotionServHandle,
	CarbonMonoxideServHandle,
	TemperatureServHandle,
	HumidityServHandle,
	ThermostatServHandle,
	TemperatureCharHandle,
	HumidityCharHandle,
	MotionDetectedCharHandle,
	CarbonMonoxideCharHandle,
	BlindCurrPosCharHandle,
	BlindTargPosCharHandle,
	BlindPosStateCharHandle,
	CurrentTempCharHandle,
	TargetTempCharHandle,
	CurrentHCStateCharHandle,
	TargetHCStateCharHandle,
	TempUnitsCharHandle,
	CoolingThresholdCharHandle,
	HeatingThresholdCharHandle;

extern volatile uint16_t slave_connection_handle;
extern uint16_t md_handle, tm_handle, hm_handle, bc_handle, bt_handle;

extern volatile uint16_t target_temperature;

/* UUIDs */
Service_UUID_t  service_uuid;
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
  Blind Service  	FF1738E0-B19E-11E2-9E96-0800200C9A66
  Motion Service  	F01738E0-B19E-11E2-9E96-0800200C9A66
  Temp Service  	F11738E0-B19E-11E2-9E96-0800200C9A66
  Humidity Service 	F21738E0-B19E-11E2-9E96-0800200C9A66
  Thermostat Serv   F31738E0-B19E-11E2-9E96-0800200C9A66
  CO Serv   		F41738E0-B19E-11E2-9E96-0800200C9A66

  Temperature  		001738F1-B19E-11E2-9E96-0800200C9A66
  Humidity 			001738F2-B19E-11E2-9E96-0800200C9A66
  MotionDetected 	001738F3-B19E-11E2-9E96-0800200C9A66

  BlindCurrPos 		111738B1-B19E-11E2-9E96-0800200C9A66
  BlindTargPos 		111738B2-B19E-11E2-9E96-0800200C9A66
  BlindPosState 	111738B2-B19E-11E2-9E96-0800200C9A66

  CurrentTemp 		221738A0-B19E-11E2-9E96-0800200C9A66
  TargetTemp 		221738A1-B19E-11E2-9E96-0800200C9A66
  CurrentHCState 	221738A2-B19E-11E2-9E96-0800200C9A66
  TargetHCState 	221738A3-B19E-11E2-9E96-0800200C9A66
  TempUnits 		221738A4-B19E-11E2-9E96-0800200C9A66
  CoolingThreshold	221738A5-B19E-11E2-9E96-0800200C9A66
  HeatingThreshold	221738A6-B19E-11E2-9E96-0800200C9A66
  */
  const uint8_t uuidBlind[16] = 				{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
		  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xe0, 0x38, 0x17, 0xFF };
  const uint8_t uuidMotion[16] = 				{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
  		  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
  												  0xe0, 0x38, 0x17, 0xF0 };
  const uint8_t uuidCarbonMonoxide[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
    		  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
    											  0xe0, 0x38, 0x17, 0xF4 };
  const uint8_t uuidTemp[16] = 					{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
    		  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
    										      0xe0, 0x38, 0x17, 0xF1 };
  const uint8_t uuidHumidity[16] = 				{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
      		  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
      										      0xe0, 0x38, 0x17, 0xF2 };
  const uint8_t uuidThermostat[16] = 			{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
        		  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
        										  0xe0, 0x38, 0x17, 0xF3 };

  const uint8_t charUuidTemperature[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
		  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xf1, 0x38, 0x17, 0x00 };
  const uint8_t charUuidHumidity[16] = 			{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xf2, 0x38, 0x17, 0x00 };
  const uint8_t charUuidMotionDetected[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
  	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xf3, 0x38, 0x17, 0x00 };
  const uint8_t charUuidCarbonMonoxide[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
    	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
  												  0xf4, 0x38, 0x17, 0x00 };
  const uint8_t charUuidBlindCurrPos[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
						  	  	  	  	  	  	  0xb1, 0x38, 0x17, 0x11 };
  const uint8_t charUuidBlindTargPos[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xb2, 0x38, 0x17, 0x11 };
  const uint8_t charUuidBlindPosState[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
  	  	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												  0xb3, 0x38, 0x17, 0x11 };
  /* Thermostat Characteristic UUIDs */
  const uint8_t charUuidCurrentTemp[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
    	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
  												  0xa0, 0x38, 0x17, 0x22 };
  const uint8_t charUuidTargetTemp[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
      	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
    											  0xa1, 0x38, 0x17, 0x22 };
  const uint8_t charUuidCurrentHCState[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
      	  	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
    											  0xa2, 0x38, 0x17, 0x22 };
  const uint8_t charUuidTargetHCState[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
        	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
      											  0xa3, 0x38, 0x17, 0x22 };
  const uint8_t charUuidTempUnits[16] = 		{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
          	  	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
        									      0xa4, 0x38, 0x17, 0x22 };
  const uint8_t charUuidCoolingThreshold[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
            	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
          									      0xa5, 0x38, 0x17, 0x22 };
  const uint8_t charUuidHeatingThreshold[16] = 	{ 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
              	  	  	  	  	  	  	  	  	  0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
            									  0xa6, 0x38, 0x17, 0x22 };

  /* Add GATT Services */

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuidBlind, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 17 /* numAttributeRecords */, &BlindServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuidMotion, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 9 /* numAttributeRecords */, &MotionServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuidCarbonMonoxide, 16);
    ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 9 /* numAttributeRecords */, &CarbonMonoxideServHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuidTemp, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 9 /* numAttributeRecords */, &TemperatureServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuidHumidity, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 9 /* numAttributeRecords */, &HumidityServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  Osal_MemCpy(&service_uuid.Service_UUID_128, uuidThermostat, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 40 /* numAttributeRecords */, &ThermostatServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  /* Add GATT Characteristics */

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidBlindCurrPos, 16);
  ret =  aci_gatt_add_char(BlindServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
	                16, 1, &BlindCurrPosCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] BlindCurrPos UUID[12] = %02X\n", charUuidBlindCurrPos[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidBlindTargPos, 16);
  ret =  aci_gatt_add_char(BlindServHandle, UUID_TYPE_128, &char_uuid, 2, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
		  16, 1, &BlindTargPosCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] BlindTargPos UUID[12] = %02X\n", charUuidBlindTargPos[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidBlindPosState, 16);
  ret =  aci_gatt_add_char(BlindServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
		  16, 1, &BlindPosStateCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] BlindPosState UUID[12] = %02X\n", charUuidBlindPosState[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidMotionDetected, 16);
  ret =  aci_gatt_add_char(MotionServHandle, UUID_TYPE_128, &char_uuid, 1, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
		  16, 0, &MotionDetectedCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] MotionDetected UUID[12] = %02X\n", charUuidMotionDetected[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidCarbonMonoxide, 16);
  ret =  aci_gatt_add_char(CarbonMonoxideServHandle, UUID_TYPE_128, &char_uuid, 1, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
		  16, 0, &CarbonMonoxideCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] CarbonMonoxide UUID[12] = %02X\n", charUuidCarbonMonoxide[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTemperature, 16);
  ret =  aci_gatt_add_char(TemperatureServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
		  16, 0, &TemperatureCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Temperature UUID[12] = %02X\n", charUuidTemperature[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidHumidity, 16);
  ret =  aci_gatt_add_char(HumidityServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
                16, 0, &HumidityCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Humidity UUID[12] = %02X\n", charUuidHumidity[12]);


  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidCurrentTemp, 16);
  ret =  aci_gatt_add_char(ThermostatServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
		  16, 0, &CurrentTempCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Current Temp UUID[12] = %02X\n", charUuidCurrentTemp[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTargetTemp, 16);
  ret =  aci_gatt_add_char(ThermostatServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
		  16, 0, &TargetTempCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Target Temp UUID[12] = %02X\n", charUuidTargetTemp[12]);

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidCurrentHCState, 16);
  ret =  aci_gatt_add_char(ThermostatServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, 0,
		  16, 0, &CurrentHCStateCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Current Heating/Cooling State UUID[12] = %02X\n", charUuidCurrentHCState[12]);
  // 0=OFF, 1=HEAT, 2=COOL

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTargetHCState, 16);
  ret =  aci_gatt_add_char(ThermostatServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
		  16, 0, &TargetHCStateCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Target Heating/Cooling State UUID[12] = %02X\n", charUuidTargetHCState[12]);
  // 0=OFF, 1=HEAT, 2=COOL, 3=AUTO

  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTempUnits, 16);
  ret =  aci_gatt_add_char(ThermostatServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, 0,
		  16, 0, &TempUnitsCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  PRINTF("[Added Char] Temp Display Units UUID[12] = %02X\n", charUuidTempUnits[12]);

//  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidCoolingThreshold, 16);
//  ret =  aci_gatt_add_char(ThermostatServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, 0,
//		  16, 0, &CoolingThresholdCharHandle);
//  if (ret != BLE_STATUS_SUCCESS) goto fail;
//  PRINTF("[Added Char] Cooling Threshold UUID[12] = %02X\n", charUuidCoolingThreshold[12]);
//
//  Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidHeatingThreshold, 16);
//  ret =  aci_gatt_add_char(ThermostatServHandle, UUID_TYPE_128, &char_uuid, 4, CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, 0,
//		  16, 0, &HeatingThresholdCharHandle);
//  if (ret != BLE_STATUS_SUCCESS) goto fail;
//  PRINTF("[Added Char] Heating Threshold UUID[12] = %02X\n", charUuidHeatingThreshold[12]);

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
			CRITICAL_BLE_TICK();
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
		printf("BlindTargPos modified: %d \n", att_data[0]);

		//TODO Write to BlindTargetPos char on blinds device
		struct timer t;
		Timer_Set(&t, CLOCK_SECOND*10);
		while(aci_gatt_write_without_resp(slave_connection_handle, bt_handle+1, data_length, att_data)==BLE_STATUS_NOT_ALLOWED) {
			CRITICAL_BLE_TICK();
			// Radio is busy (buffer full).
			if(Timer_Expired(&t))
				break;
		}
		//Update_Characteristic_Val(BlindServHandle,BlindCurrPosCharHandle,0,data_length,att_data);

	}
	else if (handle == TargetTempCharHandle + 1)
	{
		printf("TargTemperature modified: %d \n", att_data[0]);
		Osal_MemCpy(&target_temperature, att_data, data_length);
	}
//	else if(handle == TemperatureCharHandle + 1)
//	{
//		for(int i = 0; i < data_length; i++)
//			printf("%d \n", att_data[i]);
//		Update_Characteristic_Val(ServHandle,TemperatureCharHandle,0,data_length,att_data);
//	}
	else if((handle == MotionDetectedCharHandle + 2) ||
			(handle == CurrentHCStateCharHandle + 2) ||
			(handle == CurrentTempCharHandle + 2)    ||
			(handle == TemperatureCharHandle + 2)	 ||
			(handle == HumidityCharHandle + 2)	 	 ||
			(handle == BlindCurrPosCharHandle + 2))
	{
		if(att_data[0] == 0x01)
			APP_FLAG_SET(NOTIFICATIONS_ENABLED);
	}
}


