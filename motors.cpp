#include "config.h"
#include "motors.h"
#include "sensors.h" 

volatile long enc_left = 0;
volatile long enc_right = 0;
long target_left = 0;
long target_right = 0;
Action current_action = ACTION_STOP;

float current_pwm_l = 0;
float current_pwm_r = 0;
int last_wall_error = 0; 
float target_yaw = 0.0; 
bool brake_at_end = true; 
int motor_speed_limit = SPEED_SEARCH; // Default to search speed

long current_ticks_per_cell = TICKS_PER_CELL;
float last_turn_error = 0.0;
bool is_calibrating_wall = false;
int last_left_val = 0;
int last_right_val = 0;

void isr_left() { enc_left++; }
void isr_right() { enc_right++; }

void setupMotors() {
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  digitalWrite(PIN_SLEEP, HIGH); 
  pinMode(PIN_ENCC, INPUT_PULLUP);
  pinMode(PIN_ENCD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCC), isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCD), isr_right, CHANGE);
}

void setMotorSpeeds(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  if (left >= 0) { analogWrite(PIN_AIN1, left); digitalWrite(PIN_AIN2, LOW); } 
  else { digitalWrite(PIN_AIN1, LOW); analogWrite(PIN_AIN2, abs(left)); }
  if (right >= 0) { analogWrite(PIN_BIN1, right); digitalWrite(PIN_BIN2, LOW); } 
  else { digitalWrite(PIN_BIN1, LOW); analogWrite(PIN_BIN2, abs(right)); }
}

void stopMotors() {
  if (current_action == ACTION_FORWARD) { setMotorSpeeds(-255, -255); delay(20); }
  else if (current_action == ACTION_TURN_RIGHT) { setMotorSpeeds(200, -200); delay(40); }
  else if (current_action == ACTION_TURN_LEFT) { setMotorSpeeds(-200, 200); delay(40); }
  setMotorSpeeds(0, 0);
  current_action = ACTION_STOP;
  current_pwm_l = 0; current_pwm_r = 0;
  last_wall_error = 0;
  last_turn_error = 0.0; 
  delay(20); 
}

void moveForward(int cells, bool stop_at_end, bool calibrate_wall) {
  noInterrupts(); enc_left = 0; enc_right = 0; interrupts();
  target_left = (long)(cells * current_ticks_per_cell);
  target_right = target_left;
  current_action = ACTION_FORWARD;
  last_wall_error = 0; 
  brake_at_end = stop_at_end;
  is_calibrating_wall = calibrate_wall;
}

void moveForwardOffset(int cells, int offset_ticks, bool stop_at_end) {
  noInterrupts(); enc_left = 0; enc_right = 0; interrupts();
  long base_ticks = (long)(cells * current_ticks_per_cell) - offset_ticks;
  target_left = base_ticks;
  target_right = base_ticks;
  current_action = ACTION_FORWARD;
  last_wall_error = 0; 
  brake_at_end = stop_at_end;
  is_calibrating_wall = false;
}

void turnLeft() {
  current_action = ACTION_TURN_LEFT;
  int current_grid_angle = round(getYaw() / 90.0) * 90;
  target_yaw = current_grid_angle - 90.0;
  if (target_yaw < 0.0) target_yaw += 360.0; 
  last_turn_error = 0.0;
}

void turnRight() {
  current_action = ACTION_TURN_RIGHT;
  int current_grid_angle = round(getYaw() / 90.0) * 90;
  target_yaw = current_grid_angle + 90.0;
  if (target_yaw >= 360.0) target_yaw -= 360.0; 
  last_turn_error = 0.0;
}

void turnAround() {
  current_action = ACTION_TURN_LEFT; 
  int current_grid_angle = round(getYaw() / 90.0) * 90;
  target_yaw = current_grid_angle - 180.0;
  if (target_yaw < 0.0) target_yaw += 360.0;
  last_turn_error = 0.0;
}

bool isBusy() { return (current_action != ACTION_STOP); }

void updateMotion() {
  if (current_action == ACTION_STOP) return;
  long pos_l, pos_r;
  noInterrupts(); pos_l = enc_left; pos_r = enc_right; interrupts();

  bool finished = false;
  int current_left_val = 0;
  int current_right_val = 0;

  if (current_action == ACTION_FORWARD) {
     current_left_val = readSensor(PIN_EM_SIDE, PIN_RX_SL);
     current_right_val = readSensor(PIN_EM_SIDE, PIN_RX_SR);

     // 1. WINDOWED PEG DETECTION
     long sensor_pos_l = pos_l + SENSOR_OFFSET_TICKS;
     long sensor_pos_r = pos_r + SENSOR_OFFSET_TICKS;
     long half_cell = current_ticks_per_cell / 2;

     if (last_left_val > IR_WALL_THRESHOLD && current_left_val < IR_PEG_THRESHOLD) {
         if (abs((sensor_pos_l % current_ticks_per_cell) - half_cell) < 250) {
             long gap_num = round((float)(sensor_pos_l - half_cell) / current_ticks_per_cell);
             long exact_gap_pos = (gap_num * current_ticks_per_cell) + half_cell;
             noInterrupts(); enc_left = exact_gap_pos - SENSOR_OFFSET_TICKS; interrupts();
         }
     }
     if (last_right_val > IR_WALL_THRESHOLD && current_right_val < IR_PEG_THRESHOLD) {
         if (abs((sensor_pos_r % current_ticks_per_cell) - half_cell) < 250) {
             long gap_num = round((float)(sensor_pos_r - half_cell) / current_ticks_per_cell);
             long exact_gap_pos = (gap_num * current_ticks_per_cell) + half_cell;
             noInterrupts(); enc_right = exact_gap_pos - SENSOR_OFFSET_TICKS; interrupts();
         }
     }
     last_left_val = current_left_val;
     last_right_val = current_right_val;

     // 2. FRONT WALL (Symmetry Filter, Drift Fix Removed)
     if (is_calibrating_wall) {
         int front_l = readSensor(PIN_EM_FWD, PIN_RX_FL);
         int front_r = readSensor(PIN_EM_FWD, PIN_RX_FR);
         
         if (front_l > IR_WALL_THRESHOLD && front_r > IR_WALL_THRESHOLD && 
            (front_l > FRONT_WALL_CENTER_THRESHOLD || front_r > FRONT_WALL_CENTER_THRESHOLD) &&
            abs(front_l - front_r) < FRONT_SYMMETRY_TOLERANCE) {
             
             if (pos_l > current_ticks_per_cell / 2) {
                 finished = true; // Brakes hit! Drift math removed.
             }
         }
     }
     if (pos_l >= target_left || pos_r >= target_right) finished = true;
  } 
  else {
     float error = target_yaw - getYaw();
     if (error > 180.0) error -= 360.0; else if (error < -180.0) error += 360.0;
     
     if (abs(error) < 2.0) finished = true;
     else if (current_action == ACTION_TURN_RIGHT && error < 0.0 && error > -8.0) finished = true;
     else if (current_action == ACTION_TURN_LEFT && error > 0.0 && error < 8.0) finished = true;
  }

  if (finished) { 
      if (brake_at_end) stopMotors(); 
      else current_action = ACTION_STOP; 
      return; 
  }

  float target_vel;
  if (current_action == ACTION_FORWARD) {
      if (brake_at_end) {
          long dist_rem = (target_left - pos_l + target_right - pos_r) / 2;
          target_vel = min((float)SPEED_CRUISE, (float)sqrt(2.0 * 1000.0 * 0.35 * abs(dist_rem)));
          if (target_vel < 120) target_vel = 120; // Boosted floor
      } else {
          target_vel = SPEED_CRUISE; 
      }
  } else {
      float error = target_yaw - getYaw();
      if (error > 180.0) error -= 360.0; else if (error < -180.0) error += 360.0;
      float d_error = error - last_turn_error;
      float p_term = error * KP_TURN_PD;
      float d_term = d_error * KD_TURN_PD;
      target_vel = constrain(abs(p_term + d_term), 25, 200); 
      last_turn_error = error;
  }

  if (current_pwm_l < 100) current_pwm_l = 120; // Boosted floor
  if (current_pwm_l < target_vel) current_pwm_l += 2.0; else current_pwm_l = target_vel;
  current_pwm_r = current_pwm_l;

  int out_l = (int)current_pwm_l;
  int out_r = (int)current_pwm_r;

  // 3. COMBINED GYRO + WALL STEERING
  if (current_action == ACTION_FORWARD) {
      int current_error = getWallError(current_left_val, current_right_val);
      bool hasLeft = (current_left_val > IR_WALL_THRESHOLD);
      bool hasRight = (current_right_val > IR_WALL_THRESHOLD);
      
      if (hasLeft && hasRight && abs(current_error) < 15) {
          realignGyro(target_yaw);
      }
      
      float gyro_error = target_yaw - getYaw();
      if (gyro_error > 180.0) gyro_error -= 360.0; else if (gyro_error < -180.0) gyro_error += 360.0;
      float gyro_steer = gyro_error * 4.0; 

      float wall_steer = 0;
      if (current_error != 0) {
          wall_steer = (current_error * KP_WALL) + ((current_error - last_wall_error) * KD_WALL);
          last_wall_error = current_error;
      } else { last_wall_error = 0; }

      // Clamp wall steering tightly so it can't overpower the gyro compass
      wall_steer = constrain(wall_steer, -50, 50); 
      int corr_steer = (int)(gyro_steer + wall_steer);
      
      // Calculate raw desired speeds
      int raw_l = (int)current_pwm_l - corr_steer;
      int raw_r = (int)current_pwm_r + corr_steer;

      // THE OVERFLOW FIX: Preserve the steering differential at top speed!
      if (raw_l > 255) { 
          raw_r -= (raw_l - 255); // Subtract the overflow from the right wheel
          raw_l = 255; 
      } else if (raw_r > 255) { 
          raw_l -= (raw_r - 255); // Subtract the overflow from the left wheel
          raw_r = 255; 
      }

      // Final safety constrain
      out_l = constrain(raw_l, 0, 255);
      out_r = constrain(raw_r, 0, 255);
  } 

  if (current_action == ACTION_FORWARD) setMotorSpeeds(out_l, out_r);
  else if (current_action == ACTION_TURN_RIGHT) setMotorSpeeds(-out_l, out_r);
  else if (current_action == ACTION_TURN_LEFT) setMotorSpeeds(out_l, -out_r);
}

void executeSlalom(bool isRight) {
    float turn_sharpness = 0.10; 
    int pwr_outer = SPEED_CRUISE; 
    int pwr_inner = (int)(pwr_outer * turn_sharpness);
    
    int current_grid_angle = round(getYaw() / 90.0) * 90;
    target_yaw = isRight ? (current_grid_angle + 90.0) : (current_grid_angle - 90.0);
    if (target_yaw >= 360.0) target_yaw -= 360.0; if (target_yaw < 0.0) target_yaw += 360.0;

    while (true) {
        float error = target_yaw - getYaw();
        if (error > 180.0) error -= 360.0; else if (error < -180.0) error += 360.0;
        
        if (abs(error) <= 15.0) break; 
        if (isRight && error < -30.0) break;
        if (!isRight && error > 30.0) break;

        if (!isRight) setMotorSpeeds(pwr_outer, pwr_inner); 
        else setMotorSpeeds(pwr_inner, pwr_outer); 
    }
    
    current_action = ACTION_FORWARD; 
    last_wall_error = 0;
}
