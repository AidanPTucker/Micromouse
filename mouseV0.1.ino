#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "maze.h"

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

uint8_t runLog[200];
int logIndex = 0;
void logEvent(uint8_t code) { if (logIndex < 200) runLog[logIndex++] = code; }

int curX = 0, curY = 0;
int heading = 0;

enum State { 
    SEARCH_TO_GOAL, 
    RETURN_TO_START, 
    SPEED_RUN_SAFE,
    SPEED_RUN_FAST,
    FINISHED 
};
State mouseState = SEARCH_TO_GOAL;
int runNumber = 1;

bool isFirstSpeedRunMove = true;
bool justFinishedSlalom = false;

void setup() {
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 0, 50));
    pixels.show();

    setupMotors();
    setupSensors();
    setupMaze();
    Serial.begin(9600);

    pixels.setPixelColor(0, pixels.Color(50, 0, 50));
    pixels.show();
    while (!isGyroCalibrated()) { delay(50); }
    tareGyro();

    pixels.setPixelColor(0, pixels.Color(50, 50, 0)); pixels.show();
    calibrateSideSensors();

    logEvent(1);
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); pixels.show();
    delay(500);
}

void loop() {
    updateMotion();
    if (isBusy()) return;
    
    bool atGoal = isGoal(curX, curY);
    bool isSpeedRun = (mouseState == SPEED_RUN_SAFE || mouseState == SPEED_RUN_FAST);
    bool isReturning = (mouseState == RETURN_TO_START);

    if ((mouseState == SEARCH_TO_GOAL || isSpeedRun) && atGoal) {
        logEvent(2);
        
        updateWallState(curX, curY, heading, wallFront(), wallRight(), wallLeft());
        
        stopMotors();
        pixels.setPixelColor(0, pixels.Color(50, 0, 50)); pixels.show();
        delay(2000);
        if (mouseState == SPEED_RUN_SAFE) mouseState = RETURN_TO_START;
        else if (mouseState == SPEED_RUN_FAST) mouseState = RETURN_TO_START;
        else mouseState = RETURN_TO_START;
        logEvent(3);
        return;
    }

    else if (isReturning && curX == 0 && curY == 0) {
        logEvent(4);
        
        updateWallState(curX, curY, heading, wallFront(), wallRight(), wallLeft());
        
        setMotorSpeeds(120, 120);
        for (int t = 0; t < 150; t++) {
            if (wallFront()) break;
            delay(10);
        }
        stopMotors();
        pixels.setPixelColor(0, pixels.Color(50, 50, 50)); pixels.show();
        delay(2000);
        runNumber++;
        if (runNumber == 2) mouseState = SPEED_RUN_SAFE;
        else if (runNumber <= 5) mouseState = SPEED_RUN_FAST;
        else mouseState = FINISHED;
        isFirstSpeedRunMove = true;
        logEvent(5);
        return;
    }

    else if (mouseState == FINISHED) {
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show();
        while(1);
    }

    pixels.show();

    if (!isSpeedRun) {
        delay(100);
        updateWallState(curX, curY, heading, wallFront(), wallRight(), wallLeft());
    }

    floodFill(isReturning ? 0 : TARGET_X, isReturning ? 0 : TARGET_Y, isSpeedRun);
    
    int bestDir = getBestNeighbor(curX, curY);

    if (bestDir == -1) {
        logEvent(32); turnAround();
        while(isBusy()) updateMotion();
        heading = (heading + 2) % 4; return;
    }

    if (bestDir == heading) {
        int straights = countStraights(curX, curY, heading);
        if (isSpeedRun) {
            logEvent(10 + straights);
            bool targetNext = false;
            int tx = curX, ty = curY;
            if (heading == 0) ty += straights;
            else if (heading == 1) tx += straights;
            else if (heading == 2) ty -= straights;
            else if (heading == 3) tx -= straights;
            if (isGoal(tx, ty)) targetNext = true;
            
            if (mouseState == SPEED_RUN_SAFE) {
                moveForward(straights, true);
            } else {
                int total_offset = 0;
                if (justFinishedSlalom) total_offset += SLALOM_EXIT_OFFSET;
                if (!targetNext) total_offset += SLALOM_ENTRY_OFFSET;
                moveForwardOffset(straights, total_offset, targetNext);
                justFinishedSlalom = false;
            }

            isFirstSpeedRunMove = false;
            while(isBusy()) updateMotion();
            updateCoords(straights);
        } else {
            logEvent(11);
            moveForward(1, true, true);
            while(isBusy()) updateMotion();
            updateCoords(1);
        }
    }

    else {
        int diff = (bestDir - heading + 4) % 4;
        if (diff == 2) {
            turnAround(); while(isBusy()) updateMotion();
            heading = (heading + 2) % 4;
            if (!isSpeedRun) delay(100);
        } else {
            bool turnR = (diff == 1);
            if (mouseState == SPEED_RUN_FAST && !isFirstSpeedRunMove) {
                logEvent(turnR ? 41 : 40);
                executeSlalom(turnR);
                justFinishedSlalom = true;
                heading = turnR ? (heading + 1) % 4 : (heading + 3) % 4;
            } else {
                if (turnR) turnRight();
                else turnLeft();
                while(isBusy()) updateMotion();
                delay(100);
                heading = turnR ? (heading + 1) % 4 : (heading + 3) % 4;
            }
        }
    }
}

int getBestNeighbor(int x, int y) {
    bool isReturning = (mouseState == RETURN_TO_START);
    if (isReturning && x == 0 && y == 0) return -1;
    if (!isReturning && isGoal(x, y)) return -1;
    
    int best = -1, c = idx(x, y);
    long minScore = 99999;
    int dx[] = {0, 1, 0, -1}, dy[] = {1, 0, -1, 0};
    uint8_t masks[] = {WALL_N, WALL_E, WALL_S, WALL_W};
    if (c == -1) return -1;

    bool isSpeedRun = (mouseState == SPEED_RUN_SAFE || mouseState == SPEED_RUN_FAST);
    for (int i = 0; i < 4; i++) {
        if (!(maze_walls[c] & masks[i])) {
            int n = idx(x + dx[i], y + dy[i]);
            if (n == -1 || maze_dist[n] == 255) continue;

            if (isSpeedRun && !(maze_walls[n] & VISITED)) continue;
            
            long score = maze_dist[n] * 100;

            if (!isSpeedRun) {
                if (maze_walls[n] & VISITED) score += 20;
            }
            if (i != heading) score += 35;
            
            if (score < minScore) {
                minScore = score;
                best = i;
            }
        }
    }
    return best;
}

int countStraights(int x, int y, int h) {
    int count = 0, sx = x, sy = y;
    int dx[] = {0, 1, 0, -1}, dy[] = {1, 0, -1, 0};
    uint8_t masks[] = {WALL_N, WALL_E, WALL_S, WALL_W};
    while (count < 16) {
        if (maze_walls[idx(sx, sy)] & masks[h]) break;
        sx += dx[h]; sy += dy[h];
        if (idx(sx, sy) == -1) break;
        count++;
        if (getBestNeighbor(sx, sy) != h) break;
    }
    return (count == 0) ? 1 : count;
}

void updateCoords(int steps) {
    if (heading == 0) curY += steps;
    else if (heading == 1) curX += steps;
    else if (heading == 2) curY -= steps;
    else if (heading == 3) curX -= steps;
    curX = constrain(curX, 0, MAZE_W - 1);
    curY = constrain(curY, 0, MAZE_H - 1);
}
