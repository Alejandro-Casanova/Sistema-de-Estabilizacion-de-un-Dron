#ifndef ACCELEROMETER_H__
#define ACCELEROMETER_H__

/* Function to initialize accelerometer sensor */
void ACC_InitAccelerometer(void);

/* Function to read rotation in the X and Y axes */
double ACC_CalculateRotationX (void);
double ACC_CalculateRotationY (void);

/* Function to read vibration (acceleration) in the X, Y and Z axes */
void ACC_CalculateVibration (double* acc_X, double* acc_Y, double* acc_Z);

#endif
