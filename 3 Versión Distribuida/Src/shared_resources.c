#include "shared_resources.h"
#include "FreeRTOS.h" 
#include "semphr.h"
#include "stm32f4xx_hal.h"

// Mutexes for shared resource access
SemaphoreHandle_t mutex1HandleMot = NULL;
SemaphoreHandle_t mutex1HandleVibrAlarm = NULL;
SemaphoreHandle_t mutex1HandleAlt = NULL;

// Motor Control Commands
uint8_t _cmdMotAlt = 0;   //0 or 1
uint8_t _cmdMotRotX = 0;  // 0, 1 or 2
uint8_t _cmdMotRotY = 0;  // 0, 1 or 2
uint8_t _cmdMotOnOff = 0; //0 or 1

// Altimeter Variables
int _currentHeight = 0;
int _targetHeight = 0;

// Instability emergency Variable (vibrations)
uint8_t _instabilityEmergency = 0;

void SHR_Init(){
  /* Definition and creation of mutexes */
	mutex1HandleMot = xSemaphoreCreateMutex();
	mutex1HandleVibrAlarm = xSemaphoreCreateMutex();
	mutex1HandleAlt = xSemaphoreCreateMutex();
}

void SHR_GetMotorCommands(uint8_t* cmdMotOnOff, uint8_t* cmdMotRotX, uint8_t* cmdMotRotY, uint8_t* cmdMmotAlt){
	xSemaphoreTake(mutex1HandleMot, portMAX_DELAY);
	*cmdMotOnOff = _cmdMotOnOff;
	*cmdMotRotX = _cmdMotRotX;
	*cmdMotRotY = _cmdMotRotY;
	*cmdMmotAlt = _cmdMotAlt;
	xSemaphoreGive(mutex1HandleMot);
}

void SHR_SetMotorRotationCommand(uint8_t cmdMotRotX, uint8_t cmdMotRotY){
	xSemaphoreTake(mutex1HandleMot, portMAX_DELAY);
	_cmdMotRotX = cmdMotRotX;
	_cmdMotRotY = cmdMotRotY;
	xSemaphoreGive(mutex1HandleMot);
}

void SHR_SetMotorAltitudeCommand(uint8_t cmdMotAlt){
	xSemaphoreTake(mutex1HandleMot, portMAX_DELAY);
	_cmdMotAlt = cmdMotAlt;
	xSemaphoreGive(mutex1HandleMot);
}

void SHR_SetMotorOnOffState(uint8_t cmdMotOnOff){
	xSemaphoreTake(mutex1HandleMot, portMAX_DELAY);
	_cmdMotOnOff = cmdMotOnOff;
	xSemaphoreGive(mutex1HandleMot);
}

uint8_t SHR_GetInstabilityEmergency(void){
	uint8_t aux;
	xSemaphoreTake(mutex1HandleVibrAlarm, portMAX_DELAY);
	aux = _instabilityEmergency;
	xSemaphoreGive(mutex1HandleVibrAlarm);
	return aux;
}

void SHR_SetInstabilityEmergency(uint8_t instabilityEmergency){
	xSemaphoreTake(mutex1HandleVibrAlarm, portMAX_DELAY);
	_instabilityEmergency = instabilityEmergency;
	xSemaphoreGive(mutex1HandleVibrAlarm);
}

int SHR_GetCurrentHeight(){
	int aux;
	xSemaphoreTake(mutex1HandleAlt, portMAX_DELAY);
	aux = _currentHeight;
	xSemaphoreGive(mutex1HandleAlt);
	return aux;
}

int SHR_GetTargetHeight(){
	int aux;
	xSemaphoreTake(mutex1HandleAlt, portMAX_DELAY);
	aux = _targetHeight;
	xSemaphoreGive(mutex1HandleAlt);
	return aux;
}

void SHR_SetCurrentHeight(int currentHeight){
	xSemaphoreTake(mutex1HandleAlt, portMAX_DELAY);
	_currentHeight = currentHeight;
	xSemaphoreGive(mutex1HandleAlt);
}

void SHR_SetTargetHeight(int targetHeight){
	xSemaphoreTake(mutex1HandleAlt, portMAX_DELAY);
	_targetHeight = targetHeight;
	xSemaphoreGive(mutex1HandleAlt);
}

void SHR_AssignTargetHeight(){
	xSemaphoreTake(mutex1HandleAlt, portMAX_DELAY);
	_targetHeight = _currentHeight;
	xSemaphoreGive(mutex1HandleAlt);
}

