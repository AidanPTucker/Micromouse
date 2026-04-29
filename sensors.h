#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern int TARGET_SIDE_L;
extern int TARGET_SIDE_R;
extern int IR_WALL_THRESHOLD;
extern int IR_NO_WALL_THRESHOLD;

void setupSensors();
void setupGyro(); 
void tareGyro();  
float getYaw();   

void calibrateSideSensors(); 

bool wallFront();
bool wallLeft();
bool wallRight();

int readSensor(int emitter, int receiver);
int getWallError(int left_val, int right_val);
bool isGyroCalibrated(); 

void realignGyro(float true_grid_angle); 

#endif
