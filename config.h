#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define PIN_NEOPIXEL 7
#define NUMPIXELS    1

#define PIN_AIN1  6
#define PIN_AIN2  9
#define PIN_BIN1  5
#define PIN_BIN2  10
#define PIN_SLEEP 12

#define PIN_ENCA 17
#define PIN_ENCB 8
#define PIN_ENCC 0  
#define PIN_ENCD 1  

#define PIN_EM_FWD  4
#define PIN_EM_SIDE 13
#define PIN_EM_DIAG 11

#define PIN_RX_FR A0
#define PIN_RX_DR A1
#define PIN_RX_SR A2
#define PIN_RX_SL A3
#define PIN_RX_DL A4
#define PIN_RX_FL A5

#define TICKS_PER_CELL 1397 
#define TICKS_TURN_90  605 

#define KP_DIST 2.0  
#define KP_ROT  1.0  
#define MAX_SPEED 220

#define KP_WALL       1.5 
#define KD_WALL       1.0

#define FRONT_WALL_CENTER_THRESHOLD 367 
#define SENSOR_OFFSET_TICKS 190         
#define FRONT_SYMMETRY_TOLERANCE 175    

#define SLALOM_ENTRY_OFFSET 1000     
#define SLALOM_EXIT_OFFSET  300     

#define SPEED_SEARCH  150  
#define SPEED_CRUISE  240  
#define ACCEL_RATE    2.0  
#define DECEL_RATE    8.0  
#define BRAKE_COEFF   0.5

#define SPEED_TURN 130   
#define ACCEL_TURN 1.0   

#define KP_TURN_PD 5.25   
#define KD_TURN_PD 2.5 

#endif
