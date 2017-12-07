/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : chat.c
* Author             : AMS - VMA RF  Application team
* Version            : V1.0.0
* Date               : 01-December-2015
* Description        : This file handles bytes received from USB and the 
*                      chat device configuration, connection..... 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Include -------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "app_state.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "osal.h"
#include "gatt_db.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* discovery procedure mode context */
typedef struct discoveryContext_s {
  uint8_t check_disc_proc_timer;
  uint8_t check_disc_mode_timer;
  uint8_t is_device_found; 
  uint8_t do_connect;
  tClockTime startTime;
  uint8_t device_found_address_type;
  uint8_t device_found_address[6];
  uint16_t device_state;
} discoveryContext_t;

/* Private defines -----------------------------------------------------------*/
#define BLE_NEW_CHAT_COMPLETE_LOCAL_NAME_SIZE 16 
#define CMD_BUFF_SIZE 512

#define DISCOVERY_TIMEOUT 3000 /* at least 3 seconds */

/* Multiple Connection Timings */
/*
 * See PM0257 Manual, p88
 */

//Discovery Intervals
#define DISCOVERY_PROC_SCAN_INT 0x0010 		//0x4000 originally
#define DISCOVERY_PROC_SCAN_WIN 0x0010 		//0x4000 originally

//Advertising Intervals
#define ADV_INT_MIN 0x20 					//0x60 originally
#define ADV_INT_MAX 0x100 					//0x60 originally

/* Private macros ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
static discoveryContext_t discovery;
volatile long int app_flags = SET_CONNECTABLE;
volatile long int char_flags = 0;
volatile uint16_t hub_connection_handle = 0;
volatile uint16_t slave_connection_handle = 0;
extern uint16_t BlindServHandle,
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
tBleStatus bret;

volatile int notify_count[4] = {0,0,0,0};
/* UUIDs */
//Used when discovering characteristics on other BLUENRG, when this is master role
UUID_t UUID_Md, UUID_Tm, UUID_Hm, UUID_Bc, UUID_Bt;
uint16_t md_handle, tm_handle, hm_handle, bc_handle, bt_handle;

uint16_t discovery_time = 0; 
uint8_t device_role = 0xFF;
uint8_t counter = 0;
uint8_t vent_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'L','i','l','_','V','e','n','t','H','o','m','e','y','0','0'};
uint8_t blinds_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'L','i','l','_','B','l','i','n','d','H','o','m','e','y','0'};
uint8_t hub_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'L','i','l','_','H','u','b'};

static char cmd[CMD_BUFF_SIZE];

uint8_t relay_bc_val, relay_md_val;
uint16_t relay_tm_val, relay_hm_val;
float blind_temperature, blind_humidity;

uint8_t CO_detected = 1;
uint8_t CO_none = 0;

int temperature, humidity;
volatile uint16_t target_temperature;

extern ADC_InitType xADC_InitType;
volatile float adc_data1, adc_data2;

struct timer pollTimer;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Process_InputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
	PRINTF("Process_InputData() \n");
	uint8_t val = 100;
	struct timer t;
	Timer_Set(&t, CLOCK_SECOND*10);
	while(aci_gatt_write_without_resp(slave_connection_handle, bt_handle+1, 1, &val)==BLE_STATUS_NOT_ALLOWED) {
		CRITICAL_BLE_TICK();
		// Radio is busy (buffer full).
		if(Timer_Expired(&t))
			break;
	}
	PRINTF("Wrote to BlindTarget on blind device \n");
//  static uint16_t end = 0;
//  uint8_t i;
//
//  for (i = 0; i < Nb_bytes; i++) {
//    if(end >= CMD_BUFF_SIZE-1)
//      end = 0;
//
//    cmd[end] = data_buffer[i];
//    SdkEvalComIOSendData(data_buffer[i]);
//    end++;
//
//    if(cmd[end-1] == '\n') {
//      if(end != 1) {
//        int j = 0;
//        cmd[end] = '\0';
//
//        while(j < end) {
//          uint32_t len = MIN(20, end - j);
//          struct timer t;
//          Timer_Set(&t, CLOCK_SECOND*10);
//
//          if (device_role == SLAVE_ROLE) {
//            while(aci_gatt_update_char_value(TemperatureServHandle,TemperatureCharHandle,0,len,(uint8_t *)cmd+j)==BLE_STATUS_INSUFFICIENT_RESOURCES) {
//              APP_FLAG_SET(TX_BUFFER_FULL);
//              while(APP_FLAG(TX_BUFFER_FULL)) {
//                BTLE_StackTick();
//                // Radio is busy (buffer full).
//                if(Timer_Expired(&t))
//                  break;
//              }
//            }
//          } else if (device_role == MASTER_ROLE) {
//            while(aci_gatt_write_without_resp(slave_connection_handle, rx_handle+1, len, (uint8_t *)cmd+j)==BLE_STATUS_NOT_ALLOWED) {
//              BTLE_StackTick();
//              // Radio is busy (buffer full).
//              if(Timer_Expired(&t))
//                break;
//            }
//          } else {
//            break;
//          }
//          j += len;
//        }/* while(j < end)*/
//      }
//      end = 0;
//    }
//  }
}


/*******************************************************************************
* Function Name  : Reset_DiscoveryContext.
* Description    : Reset the discovery context.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Reset_DiscoveryContext(void)
{
  discovery.check_disc_proc_timer = FALSE;
  discovery.check_disc_mode_timer = FALSE;
  discovery.is_device_found = FALSE; 
  discovery.do_connect = FALSE;
  discovery.startTime = 0;
  discovery.device_state = INIT;
  Osal_MemSet(&discovery.device_found_address[0], 0, 6);
  if (!APP_FLAG(CONNECTED_TO_SLAVE) && !APP_FLAG(CONNECTED_TO_HUB)) device_role = 0xFF;
}

/*******************************************************************************
* Function Name  : Setup_DeviceAddress.
* Description    : Setup the device address.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Setup_HomeyDeviceAddress(void)
{
  tBleStatus ret;
#ifdef LIL_VENT
  uint8_t bdaddr[] = {0x01, 0x00, 0x00, 0xE1, 0x80, 0x02};
#else
  uint8_t bdaddr[] = {0x00, 0x00, 0x00, 0xE1, 0x80, 0x02};
#endif
  //uint8_t random_number[8];
  
  /* get a random number from BlueNRG */ 
//  ret = hci_le_rand(random_number);
//  if(ret != BLE_STATUS_SUCCESS)
//     PRINTF("hci_le_rand() call failed: 0x%02x\n", ret);


  discovery_time = DISCOVERY_TIMEOUT;

  /* setup discovery time with random number */
//  for (uint8_t i=0; i<8; i++) {
//    discovery_time += (2*random_number[i]);
//  }
//
//  /* Setup last 3 bytes of public address with random number */
//  bdaddr[0] = (uint8_t) (random_number[0]);
//  bdaddr[1] = (uint8_t) (random_number[3]);
//  bdaddr[2] = (uint8_t) (random_number[6]);
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS) {
      PRINTF("Setting BD_ADDR failed 0x%02x\n", ret);
  } else {
    PRINTF("Public address: ");
    for (uint8_t i=5; i>0; i--) {
      PRINTF("%02X-", bdaddr[i]);
    }
    PRINTF("%02X\n", bdaddr[0]);
  }
}

/*******************************************************************************
* Function Name  : Check_DeviceName.
* Description    : Extracts the device name.
* Input          : Data length.
*                  Data value
* Return         : TRUE if the local name found is the expected one, FALSE otherwise.
*******************************************************************************/
uint8_t Check_DeviceName(uint8_t data_length, uint8_t *data_value)
{
  uint8_t index = 0;
  
  while (index < data_length) {
    /* Advertising data fields: len, type, values */
    /* Check if field is complete local name and the lenght is the expected one for BLE NEW Chat  */
    if (data_value[index+1] == AD_TYPE_COMPLETE_LOCAL_NAME) {
      /* check if found device name is the expected one: vent_name */
#ifdef LIL_VENT
      if (memcmp(&data_value[index+1], &blinds_name[0], BLE_NEW_CHAT_COMPLETE_LOCAL_NAME_SIZE) == 0)
#else
      // If blinds, how to only allow slave?
      if (memcmp(&data_value[index+1], &hub_name[0], BLE_NEW_CHAT_COMPLETE_LOCAL_NAME_SIZE) == 0)
#endif
        return TRUE;
      else
        return FALSE;
    } else {
      /* move to next advertising field */
      index += (data_value[index] +1); 
    }
  }
  
  return FALSE;
}

/*******************************************************************************
* Function Name  : Lil_BLE_HomeyInit.
* Description    : Init Lil Homey device's GAP profile and primary service
* Input          : None.
* Return         : Status.
*******************************************************************************/
uint8_t Lil_BLE_HomeyInit(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  /* Setup the device address */
  Setup_HomeyDeviceAddress();

  /* Set the TX power to -2 dBm */
  aci_hal_set_tx_power_level(1, 7);

  /* GATT Init */
  ret = aci_gatt_init();    
  if(ret != BLE_STATUS_SUCCESS) {
      PRINTF("GATT_Init failed: 0x%02x\n", ret);
      return ret;
  }

  /* GAP Init */
#ifdef LIL_VENT
  ret = aci_gap_init(GAP_CENTRAL_ROLE | GAP_PERIPHERAL_ROLE,
		  	  	  	 0,
					 0x07,
					 &service_handle,
                     &dev_name_char_handle,
					 &appearance_char_handle
					 );
#else
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE,
  		  	  	  	 0,
  					 0x07,
  					 &service_handle,
                       &dev_name_char_handle,
  					 &appearance_char_handle
  					 );
#endif
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("GAP_Init failed: 0x%02x\n", ret);
    return ret;
  }

  /* Add Device Service & Characteristics */
  ret = Add_ChatService();
  if(ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error while adding service: 0x%02x\n", ret);
    return ret;
  }

  /* Reset the discovery context */
  Reset_DiscoveryContext();

  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : Connection_StateMachine.
* Description    : Connection state machine.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Connection_StateMachine(void)
{  
  uint8_t ret;
   
  switch (discovery.device_state) {
  case (INIT):
    {
      Reset_DiscoveryContext();
      if (APP_FLAG(CONNECTED_TO_SLAVE))
      {
    	  if (!APP_FLAG(CONNECTED_TO_HUB))
    	  {
    		  PRINTF("Entering Discovery Mode... \n");
    		  discovery.device_state = ENTER_DISCOVERY_MODE;
    	  }
      }
      else
      {
    	  discovery.device_state = START_DISCOVERY_PROC;
      }
    }
    break; /* end case (INIT) */
  case (START_DISCOVERY_PROC):
    {
#ifdef LIL_VENT
	  ret = aci_gap_start_general_discovery_proc(DISCOVERY_PROC_SCAN_INT, DISCOVERY_PROC_SCAN_WIN, PUBLIC_ADDR, 0x00);

      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_start_general_discovery_proc() failed: %02X\n",ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
        PRINTF("aci_gap_start_general_discovery_proc OK\n");  
        discovery.startTime = Clock_Time();
        discovery.check_disc_proc_timer = TRUE; 
        discovery.check_disc_mode_timer = FALSE; 
        discovery.device_state = WAIT_TIMER_EXPIRED; 
      }
#else
        discovery.check_disc_proc_timer = FALSE;
        discovery.startTime = 0;
        discovery.device_state = ENTER_DISCOVERY_MODE;
#endif
    }
    break;/* end case (START_DISCOVERY_PROC) */
  case (WAIT_TIMER_EXPIRED):
    {
      /* Verify if startTime check has to be done  since discovery procedure is ongoing */
      if (discovery.check_disc_proc_timer == TRUE) {
        /* check startTime value */
        if (Clock_Time() - discovery.startTime > discovery_time) {  
          discovery.check_disc_proc_timer = FALSE; 
          discovery.startTime = 0;
          discovery.device_state = DO_TERMINATE_GAP_PROC; 
          
        }/* if (Clock_Time() - discovery.startTime > discovery_time) */
      }  
      /* Verify if startTime check has to be done  since discovery mode is ongoing */
      else if (discovery.check_disc_mode_timer == TRUE) {
        /* check startTime value */
        if (Clock_Time() - discovery.startTime > discovery_time) {  
          discovery.check_disc_mode_timer = FALSE; 
          discovery.startTime = 0;
          
          /* Discovery mode is ongoing: set non discoverable mode */
          discovery.device_state = DO_NON_DISCOVERABLE_MODE; 
          
        }/* else if (discovery.check_disc_mode_timer == TRUE) */
      }/* if ((discovery.check_disc_proc_timer == TRUE) */
    }
    break; /* end case (WAIT_TIMER_EXPIRED) */
  case (DO_NON_DISCOVERABLE_MODE):
    {
      ret = aci_gap_set_non_discoverable();
      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_set_non_discoverable() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
        PRINTF("aci_gap_set_non_discoverable() OK\n"); 
        /* Restart Central discovery procedure */
        discovery.device_state = INIT; 
      }
    }
    break; /* end case (DO_NON_DISCOVERABLE_MODE) */
  case (DO_TERMINATE_GAP_PROC):
    {
      /* terminate gap procedure */
      ret = aci_gap_terminate_gap_proc(0x02); // GENERAL_DISCOVERY_PROCEDURE
      if (ret != BLE_STATUS_SUCCESS) 
      {
        PRINTF("aci_gap_terminate_gap_procedure() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
        break;
      }
      else 
      {
        PRINTF("aci_gap_terminate_gap_procedure() OK\n");
        discovery.device_state = WAIT_EVENT; /* wait for GAP procedure complete */
      }
    }
    break; /* end case (DO_TERMINATE_GAP_PROC) */
  case (DO_DIRECT_CONNECTION_PROC):
    {
      PRINTF("Device Found with address: ");
      for (uint8_t i=5; i>0; i--) {
        PRINTF("%02X-", discovery.device_found_address[i]);
      }
      PRINTF("%02X\n", discovery.device_found_address[0]);
      /* Do connection with first discovered device */ 
      ret = aci_gap_create_connection(DISCOVERY_PROC_SCAN_INT, DISCOVERY_PROC_SCAN_WIN,
                                      discovery.device_found_address_type, discovery.device_found_address,
                                      PUBLIC_ADDR, 0x6C, 0x6C, 0, 0xC80, 0x000C, 0x000C); //originally PUBLIC_ADDR, 40, 40, 0, 60, 2000, 2000);
      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_create_connection() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
        PRINTF("aci_gap_create_connection() OK\n");
        discovery.device_state = WAIT_EVENT;
      }
    }
    break; /* end case (DO_DIRECT_CONNECTION_PROC) */
  case (WAIT_EVENT):
    {
      discovery.device_state = WAIT_EVENT;
    }
    break; /* end case (WAIT_EVENT) */
  case (ENTER_DISCOVERY_MODE):
    {
      /* Put Peripheral device in discoverable mode */
      
      /* disable scan response */
      hci_le_set_scan_response_data(0,NULL);
      
#ifdef LIL_VENT
      ret = aci_gap_set_discoverable(ADV_IND, ADV_INT_MIN, ADV_INT_MAX, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                     sizeof(vent_name), vent_name, 0, NULL, 0x10, 0x10);
#else
      ret = aci_gap_set_discoverable(ADV_IND, ADV_INT_MIN, ADV_INT_MAX, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                           sizeof(blinds_name), blinds_name, 0, NULL, 0x6, 0x8);
#endif

      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_set_discoverable() failed: 0x%02x\n", ret);
        discovery.device_state = DISCOVERY_ERROR; 
      } else {
        PRINTF("aci_gap_set_discoverable() OK\n"); 
        discovery.startTime = Clock_Time();
        discovery.check_disc_mode_timer = TRUE; 
        discovery.check_disc_proc_timer = FALSE;
        discovery.device_state = WAIT_TIMER_EXPIRED; 
      }
    }
    break; /* end case (ENTER_DISCOVERY_MODE) */    
  case (DISCOVERY_ERROR):
    {
      Reset_DiscoveryContext();
    }
    break; /* end case (DISCOVERY_ERROR) */
  default:
    break;
   }/* end switch */

}/* end Connection_StateMachine() */

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Application tick to run the state machine.
* Input          : None.
* Return         : None.
*******************************************************************************/
void APP_Tick(void)
{
  if(APP_FLAG(SET_CONNECTABLE)) {
    Connection_StateMachine();
  }

  if (APP_FLAG(POLL_CO))
  {
	  if (!APP_FLAG(CO_STABLE))
	  {
		  if (ADC_GetFlagStatus(ADC_FLAG_EOC)) {
			  //Read converted data
			  adc_data1 = ADC_GetConvertedData(xADC_InitType.ADC_Input, xADC_InitType.ADC_ReferenceVoltage)*1000;
			  printf("ADC Value1: %f mV\n", adc_data1);
			  SdkDelayMs(100);
			  adc_data2 = ADC_GetConvertedData(xADC_InitType.ADC_Input, xADC_InitType.ADC_ReferenceVoltage)*1000;
			  printf("ADC Value2: %f mV\n", adc_data2);
			  SdkDelayMs(100);
			  if (adc_data1 == adc_data2)
			  {
				  APP_FLAG_SET(CO_STABLE);
				  if (adc_data1 > 400.0)
				  {
					  Update_Characteristic_Val(CarbonMonoxideServHandle,CarbonMonoxideCharHandle,0,1,&CO_detected);
				  }
				  else
				  {
					  Update_Characteristic_Val(CarbonMonoxideServHandle,CarbonMonoxideCharHandle,0,1,&CO_none);
				  }
			  }

			  ADC_Cmd(ENABLE);
		  }
	  }
  }

  if(APP_FLAG(POLLING))
  {
	  if (Timer_Expired(&pollTimer))
	  {
		  /* Blinds Unit is connected to Vent Unit */
		  uint8_t tmprega[2];
		  uint8_t tmpregb[2];
		  //Read temp/humidity sensor
		  SdkEvalI2CRead(&tmprega,0x40,0xf5,2); //humidity
		  humidity = (int)(((tmprega[0]*256 + tmprega[1]) *125.0)/65536.0) - 1;
		  SdkDelayMs(500);
		  SdkEvalI2CRead(&tmpregb,0x40,0xf3,2); // temp
		  temperature = (int)((((tmpregb[0]*256 + tmpregb[1]) *175.72)/65536.0) - 46.85);
		  PRINTF("[Vent] Temperature = %d, Humidity = %d\n", temperature, humidity);
		  PRINTF("[Vent] Target Temperature = %d\n", target_temperature);

		  // blind temperature variable: relay_tm_val
		  if(target_temperature > (relay_tm_val + 1)){
			  if(temperature > relay_tm_val)
			  {
				  PRINTF("open vent\n");
				  VentControl(VENT_OPEN);
			  }
			  else
			  {
				  PRINTF("close vent\n");
				  VentControl(VENT_CLOSE);
			  }
		  }
		  else if(target_temperature < (relay_tm_val - 1)){
			  if(temperature < relay_tm_val)
			  {
				  PRINTF("open vent\n");
				  VentControl(VENT_OPEN);
			  }
			  else
			  {
				  PRINTF("close vent\n");
				  VentControl(VENT_CLOSE);
			  }
		  }
		  else
		  {

		  }


		  //update characteristic value
		  //Update_Characteristic_Val(ServHandle,TemperatureCharHandle,0,8, &temperature);
		  //Update_Characteristic_Val(ServHandle,HumidityCharHandle,0,8, &humidity);
		  //Update_Characteristic_Val(ServHandle,TemperatureCharHandle,0,8, (uint8_t *) &temperature);
		  //Update_Characteristic_Val(ServHandle,HumidityCharHandle,0,8, (uint8_t *) &humidity);
//		  struct timer t;
//		  Timer_Set(&t, CLOCK_SECOND*10);
//		  while(aci_gatt_update_char_value(ServHandle, TemperatureCharHandle, 0, 2, &temperature)==BLE_STATUS_INSUFFICIENT_RESOURCES) {
//			  APP_FLAG_SET(TX_BUFFER_FULL);
//			  while(APP_FLAG(TX_BUFFER_FULL)) {
//				  NVIC_DisableIRQ(UART_IRQn);
//				  NVIC_DisableIRQ(GPIO_IRQn);
//				  BTLE_StackTick();
//				  NVIC_EnableIRQ(UART_IRQn);
//				  NVIC_EnableIRQ(GPIO_IRQn);
//				  // Radio is busy (buffer full).
//				  if(Timer_Expired(&t)){
//					  PRINTF("Error! Update Characteristic Timeout\n");
//					  break;
//				  }
//			  }
//		  }
//		  // pointer = (uint8_t *) humidptr;
//
//		  Timer_Reset(&t);
//		  while(aci_gatt_update_char_value(ServHandle, HumidityCharHandle, 0, 2, &humidity)==BLE_STATUS_INSUFFICIENT_RESOURCES) {
//			  APP_FLAG_SET(TX_BUFFER_FULL);
//			  while(APP_FLAG(TX_BUFFER_FULL)) {
//				  NVIC_DisableIRQ(UART_IRQn);
//				  NVIC_DisableIRQ(GPIO_IRQn);
//				  BTLE_StackTick();
//				  NVIC_EnableIRQ(UART_IRQn);
//				  NVIC_EnableIRQ(GPIO_IRQn);
//				  // Radio is busy (buffer full).
//				  if(Timer_Expired(&t)){
//					  PRINTF("Error! Update Characteristic Timeout\n");
//					  break;
//				  }
//			  }
//		  }



		  Timer_Reset(&pollTimer);
	  }
  }

  if (CHAR_FLAG(RELAY_BC_CHAR))
  {
	  Update_Characteristic_Val(BlindServHandle,BlindCurrPosCharHandle,0,1, &relay_bc_val);
	  CHAR_FLAG_CLEAR(RELAY_BC_CHAR);
  }

  if (CHAR_FLAG(RELAY_MD_CHAR))
  {
	  Update_Characteristic_Val(MotionServHandle, MotionDetectedCharHandle,0,1, &relay_md_val);
	  CHAR_FLAG_CLEAR(RELAY_MD_CHAR);
  }

  if (CHAR_FLAG(RELAY_TM_CHAR))
  {
	  blind_temperature = relay_tm_val;
	  PRINTF("Blind Temperature (Float): %f\n", blind_temperature);
	  Update_Characteristic_Val(TemperatureServHandle, TemperatureCharHandle,0,4, (uint8_t*) &blind_temperature);
	  CHAR_FLAG_CLEAR(RELAY_TM_CHAR);
  }

  if (CHAR_FLAG(RELAY_HM_CHAR))
  {
	  blind_humidity = relay_hm_val;
	  PRINTF("Blind Humidity (Float): %f\n", blind_humidity);
	  Update_Characteristic_Val(HumidityServHandle, HumidityCharHandle,0,4, (uint8_t*) &blind_humidity);
	  CHAR_FLAG_CLEAR(RELAY_HM_CHAR);
  }

  if (CHAR_FLAG(UPDATE_CURR_TEMP))
  {
	  Update_Characteristic_Val(ThermostatServHandle, CurrentTempCharHandle,0,2, &temperature);
	  CHAR_FLAG_CLEAR(UPDATE_CURR_TEMP);
  }
        
  if (APP_FLAG(CONNECTED_TO_SLAVE)) {
    /* Start Motion Detector handle Characteristic discovery if not yet done */
    if (!APP_FLAG(END_READ_MD_CHAR_HANDLE)) {
      if (!APP_FLAG(START_READ_MD_CHAR_HANDLE)) {
        /* Discovery TX characteristic handle by UUID 128 bits */
        const uint8_t charUuid128_Md[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
        									 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
											 0xf3, 0x38, 0x17, 0x00 };
        
        Osal_MemCpy(&UUID_Md.UUID_16, charUuid128_Md, 16);
        aci_gatt_disc_char_by_uuid(slave_connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,&UUID_Md);
        APP_FLAG_SET(START_READ_MD_CHAR_HANDLE);
      }
    }
    /* Start Temperature handle Characteristic discovery if not yet done */
    else if (!APP_FLAG(END_READ_TM_CHAR_HANDLE)) {
      /* Discovery Temperature characteristic handle by UUID 128 bits */
      if (!APP_FLAG(START_READ_TM_CHAR_HANDLE)) {
        /* Discovery Temperature characteristic handle by UUID 128 bits */
        const uint8_t charUuid128_Tm[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
											 0xf1, 0x38, 0x17, 0x00 };
        
        Osal_MemCpy(&UUID_Tm.UUID_16, charUuid128_Tm, 16);
        aci_gatt_disc_char_by_uuid(slave_connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,&UUID_Tm);
        APP_FLAG_SET(START_READ_TM_CHAR_HANDLE);
      }
    }
    /* Start Humidity handle Characteristic discovery if not yet done */
    else if (!APP_FLAG(END_READ_HM_CHAR_HANDLE)) {
    	/* Discovery Humidity characteristic handle by UUID 128 bits */
    	if (!APP_FLAG(START_READ_HM_CHAR_HANDLE)) {
    		/* Discovery Humidity characteristic handle by UUID 128 bits */
    		const uint8_t charUuid128_Hm[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
							  	  	  	  	  	 0xf2, 0x38, 0x17, 0x00 };

    		Osal_MemCpy(&UUID_Hm.UUID_16, charUuid128_Hm, 16);
    		aci_gatt_disc_char_by_uuid(slave_connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,&UUID_Hm);
    		APP_FLAG_SET(START_READ_HM_CHAR_HANDLE);
    	}
    }
    /* Start BlindCurrent handle Characteristic discovery if not yet done */
    else if (!APP_FLAG(END_READ_BC_CHAR_HANDLE)) {
    	/* Discovery BlindCurrent characteristic handle by UUID 128 bits */
    	if (!APP_FLAG(START_READ_BC_CHAR_HANDLE)) {
    		/* Discovery BlindCurrent characteristic handle by UUID 128 bits */
    		const uint8_t charUuid128_Bc[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												 0xb1, 0x38, 0x17, 0x11 };

    		Osal_MemCpy(&UUID_Bc.UUID_16, charUuid128_Bc, 16);
    		aci_gatt_disc_char_by_uuid(slave_connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,&UUID_Bc);
    		APP_FLAG_SET(START_READ_BC_CHAR_HANDLE);
    	}
    }
    /* Start BlindTarget handle Characteristic discovery if not yet done */
    else if (!APP_FLAG(END_READ_BT_CHAR_HANDLE)) {
    	/* Discovery BlindTarget characteristic handle by UUID 128 bits */
    	if (!APP_FLAG(START_READ_BT_CHAR_HANDLE)) {
    		/* Discovery BlindTarget characteristic handle by UUID 128 bits */
    		const uint8_t charUuid128_Bt[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08,
	  	  	  	  	  	  	  	  	  	  	  	 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1,
												 0xb2, 0x38, 0x17, 0x11 };

    		Osal_MemCpy(&UUID_Bt.UUID_16, charUuid128_Bt, 16);
    		aci_gatt_disc_char_by_uuid(slave_connection_handle, 0x0001, 0xFFFF,UUID_TYPE_128,&UUID_Bt);
    		APP_FLAG_SET(START_READ_BT_CHAR_HANDLE);
    	}
    }
      
    if(APP_FLAG(CONNECTED_TO_SLAVE) && APP_FLAG(END_READ_MD_CHAR_HANDLE) &&
    								   APP_FLAG(END_READ_TM_CHAR_HANDLE) &&
    								   APP_FLAG(END_READ_HM_CHAR_HANDLE) &&
    								   APP_FLAG(END_READ_BC_CHAR_HANDLE) &&
    								   APP_FLAG(END_READ_BT_CHAR_HANDLE) && !APP_FLAG(NOTIFICATIONS_ENABLED)) {
      uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
      struct timer t;
      Timer_Set(&t, CLOCK_SECOND*10);

      //Register to be notified on the blind homey's motion detector characteristic
      if (!CHAR_FLAG(START_TM_CCD) && !CHAR_FLAG(END_TM_CCD))
      {
		  CHAR_FLAG_SET(START_TM_CCD);
		  while(aci_gatt_write_char_desc(slave_connection_handle, tm_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED) {
			  // Radio is busy.
			  if(Timer_Expired(&t)) break;
		  }
      }
      while(!CHAR_FLAG(END_TM_CCD)) CRITICAL_BLE_TICK();
      PRINTF("TM\n");

      Timer_Restart(&t);
      if (!CHAR_FLAG(START_MD_CCD) && !CHAR_FLAG(END_MD_CCD))
      {
    	  CHAR_FLAG_SET(START_MD_CCD);
    	  while(aci_gatt_write_char_desc(slave_connection_handle, md_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED) {
			  // Radio is busy.
			  if(Timer_Expired(&t)) break;
		  }
      }
      while(!CHAR_FLAG(END_MD_CCD)) CRITICAL_BLE_TICK();
      PRINTF("MD\n");

      Timer_Restart(&t);
      if (!CHAR_FLAG(START_HM_CCD) && !CHAR_FLAG(END_HM_CCD))
	  {
    	  CHAR_FLAG_SET(START_HM_CCD);
    	  while(aci_gatt_write_char_desc(slave_connection_handle, hm_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED) {
			  // Radio is busy.
			  if(Timer_Expired(&t)) break;
		  }
	  }
      while(!CHAR_FLAG(END_HM_CCD)) CRITICAL_BLE_TICK();
      PRINTF("HM\n");

      Timer_Restart(&t);
      if (!CHAR_FLAG(START_BC_CCD) && !CHAR_FLAG(END_BC_CCD))
      {
		  CHAR_FLAG_SET(START_BC_CCD);
    	  while(aci_gatt_write_char_desc(slave_connection_handle, bc_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED) {
			  // Radio is busy.
			  if(Timer_Expired(&t)) break;
		  }
      }
      while(!CHAR_FLAG(END_BC_CCD)) CRITICAL_BLE_TICK();

      PRINTF("BC\n");
//      Timer_Restart(&t);
//      while(aci_gatt_write_char_desc(slave_connection_handle, bt_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED) {
//    	  // Radio is busy.
//    	  if(Timer_Expired(&t)) break;
//      }

      PRINTF("Registered for notifications on blind characteristics\n");

      APP_FLAG_SET(NOTIFICATIONS_ENABLED);
      if (!APP_FLAG(CONNECTED_TO_HUB)) APP_FLAG_SET(SET_CONNECTABLE);
    }
  }/* if (device_role == MASTER_ROLE) */

}


/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : aci_gap_proc_complete_event.
 * Description    : This event indicates the end of a GAP procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_proc_complete_event(uint8_t Procedure_Code,
                                 uint8_t Status,
                                 uint8_t Data_Length,
                                 uint8_t Data[])
{
  PRINTF("aci_gap_proc_complete_event()\n");
  if (Procedure_Code == GAP_GENERAL_DISCOVERY_PROC) { 
    /* gap procedure complete has been raised as consequence of a GAP 
       terminate procedure done after a device found event during the discovery procedure */
    if (discovery.do_connect == TRUE) {
      discovery.do_connect = FALSE;
      discovery.check_disc_proc_timer = FALSE;
      discovery.startTime = 0; 
      /* discovery procedure has been completed and device found:
         go to connection mode */
      discovery.device_state = DO_DIRECT_CONNECTION_PROC;
    } else {
      /* discovery procedure has been completed and no device found:
         go to discovery mode */
      discovery.check_disc_proc_timer = FALSE;
      discovery.startTime = 0; 
      discovery.device_state = ENTER_DISCOVERY_MODE;
    }
  }
}

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates the end of a connection procedure.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{ 

  discovery.check_disc_proc_timer = FALSE;
  discovery.check_disc_mode_timer = FALSE;
  discovery.startTime = 0; 
  
  if (Role == MASTER_ROLE)
  {
	  slave_connection_handle = Connection_Handle;
	  APP_FLAG_SET(CONNECTED_TO_SLAVE);
	  if (!APP_FLAG(CONNECTED_TO_HUB))
	  {
		  PRINTF("INIT\n");
		  discovery.device_state = INIT;
	  }
	  else
	  {
		  discovery.device_state = WAIT_EVENT;
	  }
  }
  else if (Role == SLAVE_ROLE)
  {
	  hub_connection_handle = Connection_Handle;
	  APP_FLAG_SET(CONNECTED_TO_HUB);
	  if (!APP_FLAG(CONNECTED_TO_SLAVE))
	  {
	  	  discovery.device_state = INIT;
	  }
	  else
	  {
	      discovery.device_state = WAIT_EVENT;
	  }
  }
  else
  {
	  PRINTF("Unknown Role\n");
  }

  /* store device role */
  device_role = Role;

  /* Set the exit state for the Connection state machine: APP_FLAG_CLEAR(SET_CONNECTABLE); */
  //if (APP_FLAG(CONNECTED_TO_SLAVE)) APP_FLAG_CLEAR(SET_CONNECTABLE);
  if (APP_FLAG(CONNECTED_TO_SLAVE) && APP_FLAG(CONNECTED_TO_HUB))
  {
	  APP_FLAG_CLEAR(SET_CONNECTABLE);
	  APP_FLAG_SET(POLLING);
	  Timer_Set(&pollTimer, CLOCK_SECOND*10);
  }
  else if (APP_FLAG(CONNECTED_TO_SLAVE))
  {
	  APP_FLAG_CLEAR(SET_CONNECTABLE);
  }

  PRINTF("hci_le_connection_complete_event() Role=%d\n", device_role);

}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event indicates the discconnection from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{

	if (Connection_Handle == slave_connection_handle)
	{
		APP_FLAG_CLEAR(CONNECTED_TO_SLAVE);

		APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);

		CHAR_FLAG_CLEAR(START_TM_CCD);

		APP_FLAG_CLEAR(START_READ_MD_CHAR_HANDLE);
		APP_FLAG_CLEAR(END_READ_MD_CHAR_HANDLE);
		APP_FLAG_CLEAR(START_READ_TM_CHAR_HANDLE);
		APP_FLAG_CLEAR(END_READ_TM_CHAR_HANDLE);
		APP_FLAG_CLEAR(START_READ_HM_CHAR_HANDLE);
		APP_FLAG_CLEAR(END_READ_HM_CHAR_HANDLE);
		APP_FLAG_CLEAR(START_READ_BC_CHAR_HANDLE);
		APP_FLAG_CLEAR(END_READ_BC_CHAR_HANDLE);
		APP_FLAG_CLEAR(START_READ_BT_CHAR_HANDLE);
		APP_FLAG_CLEAR(END_READ_BT_CHAR_HANDLE);
		APP_FLAG_CLEAR(TX_BUFFER_FULL);
	}
	else if (Connection_Handle == hub_connection_handle)
	{
		APP_FLAG_CLEAR(CONNECTED_TO_HUB);
	}

	/* Make the device connectable again. */
	APP_FLAG_SET(SET_CONNECTABLE);

	discovery.check_disc_proc_timer = FALSE;
	discovery.check_disc_mode_timer = FALSE;
	discovery.is_device_found = FALSE;
	discovery.do_connect = FALSE;
	discovery.startTime = 0;

}/* end hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_le_advertising_report_event.
 * Description    : An advertising report is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_advertising_report_event(uint8_t Num_Reports,
                                     Advertising_Report_t Advertising_Report[])
{
	// Discovered devices are each returned to this event
  //PRINTF("hci_le_advertising_report_event()\n");
	/* Advertising_Report contains all the expected parameters */
  uint8_t evt_type = Advertising_Report[0].Event_Type ;
  uint8_t data_length = Advertising_Report[0].Length_Data;
  uint8_t bdaddr_type = Advertising_Report[0].Address_Type;
  uint8_t bdaddr[6];

  Osal_MemCpy(bdaddr, Advertising_Report[0].Address,6);
      
  /* BLE Vent device not yet found: check current device found */
  if (!(discovery.is_device_found)) { 
    /* BLE Vent device not yet found: check current device found */
    if ((evt_type == ADV_IND) && Check_DeviceName(data_length, Advertising_Report[0].Data)) {
      discovery.is_device_found = TRUE; 
      discovery.do_connect = TRUE;  
      discovery.check_disc_proc_timer = FALSE;
      discovery.check_disc_mode_timer = FALSE;
      /* store first device found:  address type and address value */
      discovery.device_found_address_type = bdaddr_type;
      Osal_MemCpy(discovery.device_found_address, bdaddr, 6);
      /* device is found: terminate discovery procedure */
      discovery.device_state = DO_TERMINATE_GAP_PROC;      
    }
  }  
} /* hci_le_advertising_report_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : Attribute modified from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_CB(Attr_Handle, Attr_Data_Length, Attr_Data);      
} /* end aci_gatt_attribute_modified_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_notification_event.
 * Description    : Notification received from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint8_t Attribute_Value_Length,
                                 uint8_t Attribute_Value[])
{ 
  PRINTF("aci_gatt_notification_event()\n");
  if(Attribute_Handle == md_handle+1) {
	  notify_count[0]++;
	  relay_md_val = Attribute_Value[0];
    //for(volatile uint8_t i = 0; i < Attribute_Value_Length; i++)
      printf("[Blinds] Motion Sensor: %02X\n", Attribute_Value[0]);
      CHAR_FLAG_SET(RELAY_MD_CHAR);
    //Update_Characteristic_Val(MotionServHandle, MotionDetectedCharHandle,0,Attribute_Value_Length, Attribute_Value);
  }
  else if(Attribute_Handle == tm_handle+1) {
	  notify_count[1]++;
	  //relay_tm_val = Attribute_Value[0];
	  Osal_MemCpy(&relay_tm_val, Attribute_Value, Attribute_Value_Length);
      //for(volatile uint8_t i = 0; i < Attribute_Value_Length; i++)
	  printf("length: %d\n", Attribute_Value_Length);
      printf("[Blinds] Temperature Notification %d\n", relay_tm_val);
      CHAR_FLAG_SET(RELAY_TM_CHAR);
      //Update_Characteristic_Val(TemperatureServHandle, TemperatureCharHandle,0,Attribute_Value_Length, Attribute_Value);
  }
  else if(Attribute_Handle == hm_handle+1) {
	  notify_count[2]++;
	  //relay_hm_val = Attribute_Value[0];
	  Osal_MemCpy(&relay_hm_val, Attribute_Value, Attribute_Value_Length);
      //for(volatile uint8_t i = 0; i < Attribute_Value_Length; i++)
	  printf("length: %d\n", Attribute_Value_Length);
      printf("[Blinds] Humidity Notification %d\n", relay_hm_val);
      CHAR_FLAG_SET(RELAY_HM_CHAR);
      //Update_Characteristic_Val(HumidityServHandle, HumidityCharHandle,0,Attribute_Value_Length, Attribute_Value);
  }
  else if(Attribute_Handle == bc_handle+1) {
	  notify_count[3]++;
	  relay_bc_val = Attribute_Value[0];
	  printf("[Blinds] BlindCurrPos Notification: %d\n", relay_bc_val);
	  CHAR_FLAG_SET(RELAY_BC_CHAR);
	  //Update_Characteristic_Val(BlindServHandle,BlindCurrPosCharHandle,0,Attribute_Value_Length,Attribute_Value);
  }

  PRINTF("{%02X,%02X,%02X,%02X}\n", notify_count[0], notify_count[1], notify_count[2], notify_count[3]);
} /* end aci_gatt_notification_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_disc_read_char_by_uuid_resp_event.
 * Description    : Read characteristic by UUID from a peer device.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_disc_read_char_by_uuid_resp_event(uint16_t Connection_Handle,
                                                uint16_t Attribute_Handle,
                                                uint8_t Attribute_Value_Length,
                                                uint8_t Attribute_Value[])
{
  printf("aci_gatt_disc_read_char_by_uuid_resp_event, Connection Handle: 0x%04X\n", Connection_Handle);
  if (APP_FLAG(START_READ_MD_CHAR_HANDLE) && !APP_FLAG(END_READ_MD_CHAR_HANDLE))
  {
	  md_handle = Attribute_Handle;
	  printf("MD Char Handle 0x%04X\n", md_handle);
  } else if (APP_FLAG(START_READ_TM_CHAR_HANDLE) && !APP_FLAG(END_READ_TM_CHAR_HANDLE))
  {
	  tm_handle = Attribute_Handle;
	  printf("TM Char Handle 0x%04X\n", tm_handle);
  } else if (APP_FLAG(START_READ_HM_CHAR_HANDLE) && !APP_FLAG(END_READ_HM_CHAR_HANDLE))
  {
	  hm_handle = Attribute_Handle;
	  printf("HM Char Handle 0x%04X\n", hm_handle);
  } else if (APP_FLAG(START_READ_BC_CHAR_HANDLE) && !APP_FLAG(END_READ_BC_CHAR_HANDLE))
  {
	  bc_handle = Attribute_Handle;
	  printf("BC Char Handle 0x%04X\n", bc_handle);
  } else if (APP_FLAG(START_READ_BT_CHAR_HANDLE) && !APP_FLAG(END_READ_BT_CHAR_HANDLE))
  {
	  bt_handle = Attribute_Handle;
	  printf("BT Char Handle 0x%04X\n", bt_handle);
  }
} /* end aci_gatt_disc_read_char_by_uuid_resp_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_proc_complete_event.
 * Description    : GATT procedure complet event.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{ 
	PRINTF("aci_gatt_proc_complete_event()\n");
	if (APP_FLAG(START_READ_MD_CHAR_HANDLE) && !APP_FLAG(END_READ_MD_CHAR_HANDLE)) {
		APP_FLAG_SET(END_READ_MD_CHAR_HANDLE);
	} else if (APP_FLAG(START_READ_TM_CHAR_HANDLE) && !APP_FLAG(END_READ_TM_CHAR_HANDLE)) {
		APP_FLAG_SET(END_READ_TM_CHAR_HANDLE);
	} else if (APP_FLAG(START_READ_HM_CHAR_HANDLE) && !APP_FLAG(END_READ_HM_CHAR_HANDLE)) {
		APP_FLAG_SET(END_READ_HM_CHAR_HANDLE);
	} else if (APP_FLAG(START_READ_BC_CHAR_HANDLE) && !APP_FLAG(END_READ_BC_CHAR_HANDLE)) {
		APP_FLAG_SET(END_READ_BC_CHAR_HANDLE);
	} else if (APP_FLAG(START_READ_BT_CHAR_HANDLE) && !APP_FLAG(END_READ_BT_CHAR_HANDLE)) {
		APP_FLAG_SET(END_READ_BT_CHAR_HANDLE);
	}

	if (CHAR_FLAG(START_TM_CCD) && !CHAR_FLAG(END_TM_CCD)) CHAR_FLAG_SET(END_TM_CCD);
	if (CHAR_FLAG(START_MD_CCD) && !CHAR_FLAG(END_MD_CCD)) CHAR_FLAG_SET(END_MD_CCD);
	if (CHAR_FLAG(START_HM_CCD) && !CHAR_FLAG(END_HM_CCD)) CHAR_FLAG_SET(END_HM_CCD);
	if (CHAR_FLAG(START_BC_CCD) && !CHAR_FLAG(END_BC_CCD)) CHAR_FLAG_SET(END_BC_CCD);

} /* end aci_gatt_proc_complete_event() */


//void aci_att_read_resp_event(uint16_t Connection_Handle,
//                             uint8_t Event_Data_Length,
//                             uint8_t Attribute_Value[])
//{
//	//response to gatt read
//	if (Connection_Handle == slave_connection_handle)
//	{
//		PRINTF("aci_att_read_resp_event()");
//		PRINTF("Length: %d, Val: ", Event_Data_Length);
//		for (volatile int i = 0; i < Event_Data_Length; i++)
//		{
//			PRINTF("%02X", Attribute_Value[i]);
//		}
//
//	}
//}


/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : GATT TX pool available event.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{      
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
} /* end aci_gatt_tx_pool_available_event() */

void aci_hal_scan_req_report_event(uint8_t RSSI,
                                   uint8_t Peer_Address_Type,
                                   uint8_t Peer_Address[6])
{
	PRINTF("aci_hal_scan_req_report_event()\n");
}
