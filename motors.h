#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

extern volatile long enc_left;
extern volatile long enc_right;
extern long target_left;
extern long target_right;

extern long current_ticks_per_cell;
extern float last_turn_error;
extern bool brake_at_end; // <--- THE FIX: Expose this to the main sketch!
extern int motor_speed_limit; // <--- Added this variable

enum Action { 
    ACTION_STOP, 
    ACTION_FORWARD, 
    ACTION_TURN_LEFT, 
    ACTION_TURN_RIGHT 
};

extern Action current_action;

void setupMotors();
void updateMotion(); 
void stopMotors();
void setMotorSpeeds(int l, int r);

void moveForward(int cells, bool stop_at_end = true, bool calibrate_wall = false);
void moveForwardOffset(int cells, int offset_ticks, bool stop_at_end = true);
void turnLeft();
void turnRight();
void turnAround();
void executeSlalom(bool isRight);
bool isBusy(); 

int getWallError(int left_val, int right_val);

#endif
