#include "accelerometer.h"
#include "FreeRTOS.h" 
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/* Variables to calculate the rotation in the x and y axes of accelerometer */
double X, Y, Z;
SemaphoreHandle_t mutex1HandleAccel;

/* Function to read the registers of accelerometer */
uint8_t spiTxBuf[2], spiRxBuf[2];
uint8_t SPI_Read (uint8_t address);
extern SPI_HandleTypeDef hspi1;

void ACC_InitAccelerometer()	
{
 /*To transmit data in SPI follow the next steps: */
 // 1. Bring slave select to low

 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
 // 2. Transmit register + data
 spiTxBuf[0] = 0x20; // control Register
 spiTxBuf[1] = 0x17; //Data  Enable X Y Z Rate 3.125 Hz --- Valor original = 0x11
 //								size, timeout
 HAL_SPI_Transmit(&hspi1, spiTxBuf, 2, 50);
 // 3. Bring slave select high
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

 /*To receive data in SPI follow the next steps: */
 // 1.Bring slave select low
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
 // 2.Transmit register + 0x80 (To set MSB high) Most Significant Bit(MSB) high = read mode
 spiTxBuf[0] = 0x20|0x80; //Register
 HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
 // 3.Receive data
 HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
 // 4.Bring slave select high
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
 
 // Instantiate Mutex
 mutex1HandleAccel = xSemaphoreCreateMutex();
}

/* Function body to read accelerometer registers*/
/* This function reads the 8 bit register allocated in "adress" */
uint8_t SPI_Read (uint8_t address)
{
	// 1.Bring slave select low
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	// 2.Transmit register + 0x80 (To set MSB high) Most Significant Bit(MSB) high = read mode
	spiTxBuf[0] = address|0x80; //Register
	HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
	// 3.Receive data
	HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
	// 4.Bring slave select high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	
	return spiRxBuf[0];
}

/* Function body to obtain the coordinates of the three accelerometer axes */
void Obtain_Coordinates_XYZ()
{
	/* Calculate the coordinates X Y Z, and sotres them in the global variables X Y Z */
  int Ix, Iy, Iz;
  uint8_t Ix1, Ix2;
  uint8_t Iy1, Iy2;
  uint8_t Iz1, Iz2;
	
	Ix1 = SPI_Read (0x28);
	Ix2 = SPI_Read (0x29);
	Ix = (Ix2 << 8) + Ix1;
	if (Ix >= 0x8000) Ix = -(65536 - Ix);
	X = Ix/16384.0;
		
	Iy1 = SPI_Read (0x2A);
	Iy2 = SPI_Read (0x2B);
	Iy = (Iy2 << 8) + Iy1;
	if (Iy >= 0x8000) Iy = -(65536 - Iy);
	Y = Iy/16384.0;
		
	Iz1 = SPI_Read (0x2C);
	Iz2 = SPI_Read (0x2D);
	Iz = (Iz2 << 8) + Iz1;
	if (Iz >= 0x8000) Iz = -(65536 - Iz);
	Z = Iz/16384.0;	
}

double ACC_CalculateRotationX ()
{ 
	double rotX;
	
	xSemaphoreTake(mutex1HandleAccel, portMAX_DELAY);
	
	Obtain_Coordinates_XYZ();	
	rotX = atan2(Y, sqrt(X*X+Z*Z)) * 180.0/3.1416;
	
	xSemaphoreGive(mutex1HandleAccel);
	
	return rotX;
}	

double ACC_CalculateRotationY ()
{
  double rotY;
	
	xSemaphoreTake(mutex1HandleAccel, portMAX_DELAY);
	
	Obtain_Coordinates_XYZ();
	rotY = - atan2(X, sqrt(Y*Y+Z*Z)) * 180.0/3.1416;
	
	xSemaphoreGive(mutex1HandleAccel);
	
	return rotY;
 }

void ACC_CalculateVibration (double* acc_X, double* acc_Y, double* acc_Z)
{ 
	
	xSemaphoreTake(mutex1HandleAccel, portMAX_DELAY);
	
	Obtain_Coordinates_XYZ();	
	
	*acc_X = X;
	*acc_Y = Y;
	*acc_Z = Z;
	
	xSemaphoreGive(mutex1HandleAccel);

}	
