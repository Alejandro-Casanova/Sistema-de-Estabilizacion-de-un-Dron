#include "can_bus.h"
#include <stm32f4xx_hal.h>
#include "config.h"

/* Declaraciones para tranmisiones en CAN BUS ------------------------------*/
extern CAN_HandleTypeDef hcan1; // Is defined in main
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
uint32_t TxMailbox; 
uint8_t ByteSent = 0; //declare byte to be transmitted //declare a receive byte
uint8_t ByteReceived = 0; //declare a receive byte
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

void CAN_InitTransmissions() {

	pHeader.DLC=1; //give message size of 1 byte
	pHeader.IDE=CAN_ID_STD; //set identifier to standard
	pHeader.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
#ifdef CAN_BUS_MODE_TX
	pHeader.StdId=0x2F4; //define a standard identifier, used for message identification by filters
#else
	pHeader.StdId=0x2FF; //define a standard identifier, used for message identification by filters
#endif
	//filter one (stack light blink)
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
#ifdef CAN_BUS_MODE_TX
	sFilterConfig.FilterIdHigh=0x2FF<<5; //the ID that the filter looks for
#else
	sFilterConfig.FilterIdHigh=0x2F4<<5; //the ID that the filter looks for
#endif
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); //configure CAN filter

	HAL_CAN_Start(&hcan1); //start CAN
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
};

void CAN_sendByte(uint8_t byte){
	ByteSent = byte;
	HAL_CAN_AddTxMessage(&hcan1, &pHeader , &ByteSent , &TxMailbox);
}

uint8_t CAN_getByteReceived(void){
	return ByteReceived;
}
