

#ifndef _GATT_DB_H_
#define _GATT_DB_H_

void Update_Characteristic_Val( uint16_t Service_Handle,
        						uint16_t Char_Handle,
								uint8_t Val_Offset,
								uint8_t Char_Value_Length,
								uint8_t Char_Value[]);

tBleStatus Add_ChatService(void);
void Attribute_Modified_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data);

extern uint16_t ServHandle,
				TemperatureCharHandle,
				HumidityCharHandle,
				MotionDetectedCharHandle,
				BlindCurrPosCharHandle,
				BlindTargPosCharHandle,
				BlindPosStateCharHandle;

#endif /* _GATT_DB_H_ */
