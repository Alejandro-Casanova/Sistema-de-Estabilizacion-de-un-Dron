#ifndef CAN_BUS_H__
#define CAN_BUS_H__
#include <stdint.h>

typedef enum{
	CAN_VIBR_EMER_0 			= 0x01,
	CAN_VIBR_EMER_1 			= 0x02,
	CAN_MOT_CMD_ON_OFF_0 	= 0x03,
	CAN_MOT_CMD_ON_OFF_1 	= 0x04,
	CAN_MOT_CMD_ALT_0			= 0x05,
	CAN_MOT_CMD_ALT_1			= 0x06,
	CAN_MOT_CMD_ROT_X_0		= 0x07,
	CAN_MOT_CMD_ROT_X_1		= 0x08,
	CAN_MOT_CMD_ROT_X_2		= 0x09,
	CAN_MOT_CMD_ROT_Y_0		= 0x0A,
	CAN_MOT_CMD_ROT_Y_1		= 0x0B,
	CAN_MOT_CMD_ROT_Y_2		= 0x0C
} CAN_Command_t;

void CAN_InitTransmissions(void);	// Must be called in setup
void CAN_sendByte(uint8_t byte);
uint8_t CAN_getByteReceived(void);

#endif
