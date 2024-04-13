#ifndef STATECHART_H
#define STATECHART_H

// Init function (MUST be called on setup)
void STCH_Init(void); 
	
// Tilt Correction Events
void STCH_XFor(void);
void STCH_XBack(void);
void STCH_X0(void);
void STCH_YFor(void);
void STCH_YBack(void);
void STCH_Y0(void);

// Height Correction Events
void STCH_HUp(void);
void STCH_H0(void);

// System Activation/Deactivation Events
void STCH_Start(void);
void STCH_Wait(void);

// Instability Risk Emergency Event
void STCH_Risk(void);

#endif
