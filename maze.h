#ifndef MAZE_HEADER_H  
#define MAZE_HEADER_H

#include <Arduino.h>

#define MAZE_W 16
#define MAZE_H 16

#define TARGET_X 7
#define TARGET_Y 7

#define WALL_N 0x01
#define WALL_E 0x02
#define WALL_S 0x04
#define WALL_W 0x08
#define VISITED 0x10

// Global Maze Arrays
extern uint8_t maze_walls[256];
extern uint8_t maze_dist[256]; 

// Core Functions
void setupMaze();
void updateWallState(int x, int y, int heading, bool front, bool right, bool left);

// FIX 1: Add speed run flag to floodfill
void floodFill(int targetX, int targetY, bool isSpeedRun = false); 

int idx(int x, int y);
bool isGoal(int x, int y);

#endif // MAZE_HEADER_H
