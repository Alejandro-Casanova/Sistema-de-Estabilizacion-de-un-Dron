#include "statechart.h"
#include "led_ctrl.h"
#include <FreeRTOS.h>
#include <semphr.h>

typedef enum{
	ST_STANDBY,
	ST_ADJUSTING_ANGLE,
	ST_ADJUSTING_HEIGHT,
	ST_POSITION_OK,
	ST_STOP
}TypeStates;

TypeStates state = ST_STANDBY;

SemaphoreHandle_t mutexStateChart;

void STCH_Init(void){
	mutexStateChart = xSemaphoreCreateMutex();
}

void STCH_XFor(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			M1On(); M2Off();
			break;
		case ST_ADJUSTING_HEIGHT:
			break;
		case ST_POSITION_OK:
			state = ST_ADJUSTING_ANGLE;
			M1On(); M2Off();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_XBack(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			M1Off(); M2On();
			break;
		case ST_ADJUSTING_HEIGHT:
			break;
		case ST_POSITION_OK:
			state = ST_ADJUSTING_ANGLE;
			M1Off(); M2On();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_X0(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			state = ST_POSITION_OK;
			M1Off(); M2Off();
			break;
		case ST_ADJUSTING_HEIGHT:
			break;
		case ST_POSITION_OK:
			M1Off(); M2Off();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_YFor(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			M4On(); M3Off();
			break;
		case ST_ADJUSTING_HEIGHT:
			break;
		case ST_POSITION_OK:
			state = ST_ADJUSTING_ANGLE;
			M4On(); M3Off();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_YBack(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			M4Off(); M3On();
			break;
		case ST_ADJUSTING_HEIGHT:
			break;
		case ST_POSITION_OK:
			state = ST_ADJUSTING_ANGLE;
			M4Off(); M3On();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_Y0(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			state = ST_POSITION_OK;
			M4Off(); M3Off();
			break;
		case ST_ADJUSTING_HEIGHT:
			break;
		case ST_POSITION_OK:
			M4Off(); M3Off();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_HUp(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			state = ST_ADJUSTING_HEIGHT;
			M1On(); M2On(); M3On(); M4On();
			break;
		case ST_ADJUSTING_HEIGHT:
			M1On(); M2On(); M3On(); M4On();
			break;
		case ST_POSITION_OK:
			state = ST_ADJUSTING_HEIGHT;
			M1On(); M2On(); M3On(); M4On();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_H0(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			break;
		case ST_ADJUSTING_HEIGHT:
			state = ST_POSITION_OK;
			M1Off(); M2Off(); M3Off(); M4Off();
			break;
		case ST_POSITION_OK:
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_Start(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			state = ST_POSITION_OK;
			L1On();
			break;
		case ST_ADJUSTING_ANGLE:
			break;
		case ST_ADJUSTING_HEIGHT:
			break;
		case ST_POSITION_OK:
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_Wait(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			break;
		case ST_ADJUSTING_ANGLE:
			state = ST_STANDBY;
			L1Off();
			M1Off(); M2Off(); M3Off(); M4Off();
			break;
		case ST_ADJUSTING_HEIGHT:
			state = ST_STANDBY;
			L1Off();
			M1Off(); M2Off(); M3Off(); M4Off();
			break;
		case ST_POSITION_OK:
			state = ST_STANDBY;
			L1Off();
			M1Off(); M2Off(); M3Off(); M4Off();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}

void STCH_Risk(void){
	xSemaphoreTake(mutexStateChart, portMAX_DELAY);
	switch(state){
		case ST_STANDBY:
			state = ST_STOP;
			L2On();
			break;
		case ST_ADJUSTING_ANGLE:
			state = ST_STOP;
			L2On(); L1Off();
			M1Off(); M2Off(); M3Off(); M4Off();
			break;
		case ST_ADJUSTING_HEIGHT:
			state = ST_STOP;
			L2On(); L1Off();
			M1Off(); M2Off(); M3Off(); M4Off();
			break;
		case ST_POSITION_OK:
			state = ST_STOP;
			L2On(); L1Off();
			M1Off(); M2Off(); M3Off(); M4Off();
			break;
		case ST_STOP:
			break;
	}
	xSemaphoreGive(mutexStateChart);
}
