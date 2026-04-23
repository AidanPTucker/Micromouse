#include "config.h"
#include "sensors.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
float gyro_offset = 0.0; 

int TARGET_SIDE_L = 360;
int TARGET_SIDE_R = 360;
int IR_WALL_THRESHOLD = 150;
int IR_PEG_THRESHOLD = 60;

void setupGyro() {
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println(F("No BNO055 detected! Check wiring."));
    while (1); 
  }
  bno.setExtCrystalUse(true);
}

void tareGyro() {
  sensors_event_t event;
  bno.getEvent(&event);
  gyro_offset = event.orientation.x;
}

float getYaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  float raw_yaw = event.orientation.x;
  float adjusted_yaw = raw_yaw - gyro_offset;
  if (adjusted_yaw < 0.0) adjusted_yaw += 360.0;
  if (adjusted_yaw >= 360.0) adjusted_yaw -= 360.0;
  return adjusted_yaw; 
}

void setupSensors() {
  setupGyro(); 

  pinMode(PIN_EM_FWD, OUTPUT);
  pinMode(PIN_EM_SIDE, OUTPUT);
  pinMode(PIN_EM_DIAG, OUTPUT);
  
  digitalWrite(PIN_EM_FWD, LOW);
  digitalWrite(PIN_EM_SIDE, LOW);
  digitalWrite(PIN_EM_DIAG, LOW);
}

int readSensor(int emitter, int receiver) {
  int ambient = analogRead(receiver);
  digitalWrite(emitter, HIGH);
  delayMicroseconds(80); 
  int signal = analogRead(receiver);
  digitalWrite(emitter, LOW);
  int value = signal - ambient;
  return (value > 0) ? value : 0;
}

void calibrateSideSensors() {
  long sumL = 0;
  long sumR = 0;
  
  for(int i = 0; i < 20; i++) {
      sumL += readSensor(PIN_EM_SIDE, PIN_RX_SL);
      sumR += readSensor(PIN_EM_SIDE, PIN_RX_SR);
      delay(10);
  }
  
  TARGET_SIDE_L = sumL / 20;
  TARGET_SIDE_R = sumR / 20;
  
  int avg_wall = (TARGET_SIDE_L + TARGET_SIDE_R) / 2;
  
  IR_WALL_THRESHOLD = avg_wall * 0.30; 
  
  IR_PEG_THRESHOLD = avg_wall * 0.15;  
}

bool wallFront() {
  int left = readSensor(PIN_EM_FWD, PIN_RX_FL);
  int right = readSensor(PIN_EM_FWD, PIN_RX_FR);
  return (left > IR_WALL_THRESHOLD) || (right > IR_WALL_THRESHOLD);
}

bool wallLeft() {
  return readSensor(PIN_EM_SIDE, PIN_RX_SL) > IR_WALL_THRESHOLD;
}

bool wallRight() {
  return readSensor(PIN_EM_SIDE, PIN_RX_SR) > IR_WALL_THRESHOLD;
}

int getWallError(int left, int right) {
  bool hasLeft = (left > IR_WALL_THRESHOLD);
  bool hasRight = (right > IR_WALL_THRESHOLD);
  int error = 0;

  if (hasLeft && hasRight) {
    int left_error = left - TARGET_SIDE_L;
    int right_error = right - TARGET_SIDE_R;
    error = left_error - right_error;
  }
  else if (hasLeft) {
    error = (left - TARGET_SIDE_L); 
  }
  else if (hasRight) {
    error = -(right - TARGET_SIDE_R);
  }
  return error;
}

bool isGyroCalibrated() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  return (gyro >= 3); 
}

void realignGyro(float true_grid_angle) {
  sensors_event_t event;
  bno.getEvent(&event);
  float raw_yaw = event.orientation.x;
  
  float current_reading = raw_yaw - gyro_offset;
  if (current_reading < 0.0) current_reading += 360.0;
  
  float error = true_grid_angle - current_reading;
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;

  gyro_offset -= (error * 0.05); 
  if (gyro_offset < 0.0) gyro_offset += 360.0;
  if (gyro_offset >= 360.0) gyro_offset -= 360.0;
}
