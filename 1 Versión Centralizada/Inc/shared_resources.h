#ifndef SHARED_RESOURCES_H__
#define SHARED_RESOURCES_H__
#include <stdint.h>

// Init function for mutex creation (MUST be called on setup)
void SHR_Init(void);

// Setters and getters for shared resource "MotorRotationCommand"
void SHR_GetMotorCommands(uint8_t* cmdMotOnOff, uint8_t* cmdMotRotX, uint8_t* cmdMotRotY, uint8_t* cmdMotAlt);
void SHR_SetMotorRotationCommand(uint8_t cmdMotRotX, uint8_t cmdMotRotY);
void SHR_SetMotorAltitudeCommand(uint8_t cmdMotAlt);
void SHR_SetMotorOnOffState(uint8_t cmdMotOnOff);

// Setters and getters for shared resource "InstabilityEmergency"
uint8_t SHR_GetInstabilityEmergency(void);
void SHR_SetInstabilityEmergency(uint8_t instabilityEmergency);

// Setters and getters for shared resource "DroneAltitude"
int SHR_GetCurrentHeight(void);
int SHR_GetTargetHeight(void);
void SHR_SetCurrentHeight(int currentHeight);
void SHR_SetTargetHeight(int targetHeight);
void SHR_AssignTargetHeight(void); // Assigns currentHeight to targetHeight in just one call (prevents having to make two calls to first get current altitude and then set target altitude)
	
#endif
