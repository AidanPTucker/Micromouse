#include "maze.h"

uint8_t maze_walls[256];
uint8_t maze_dist[256];

int idx(int x, int y) {
    if (x < 0 || x >= MAZE_W || y < 0 || y >= MAZE_H) return -1;
    return x + y * MAZE_W;
}

bool isGoal(int x, int y) {
    if (TARGET_X == 7 && TARGET_Y == 7) {
        return (x == 7 || x == 8) && (y == 7 || y == 8);
    }
    else {
        return (x == TARGET_X && y == TARGET_Y);
    }
}

void setupMaze() {
    for (int i = 0; i < 256; i++) { maze_walls[i] = 0; maze_dist[i] = 255; }
    for (int x = 0; x < MAZE_W; x++) {
        int s = idx(x, 0); if (s != -1) maze_walls[s] |= WALL_S;
        int n = idx(x, MAZE_H-1); if (n != -1) maze_walls[n] |= WALL_N;
    }
    for (int y = 0; y < MAZE_H; y++) {
        int w = idx(0, y); if (w != -1) maze_walls[w] |= WALL_W;
        int e = idx(MAZE_W-1, y); if (e != -1) maze_walls[e] |= WALL_E;
    }
}

void updateWallState(int x, int y, int heading, bool front, bool right, bool left) {
    int current = idx(x, y);
    if (current == -1) return;
    maze_walls[current] |= VISITED;

    switch (heading) {
        case 0:  
            if (front) maze_walls[current] |= WALL_N;
            if (right) maze_walls[current] |= WALL_E;
            if (left)  maze_walls[current] |= WALL_W;
            break;
        case 1:  
            if (front) maze_walls[current] |= WALL_E;
            if (right) maze_walls[current] |= WALL_S;
            if (left)  maze_walls[current] |= WALL_N;
            break;
        case 2: 
            if (front) maze_walls[current] |= WALL_S;
            if (right) maze_walls[current] |= WALL_W;
            if (left)  maze_walls[current] |= WALL_E;
            break;
        case 3: 
            if (front) maze_walls[current] |= WALL_W;
            if (right) maze_walls[current] |= WALL_N;
            if (left)  maze_walls[current] |= WALL_S;
            break;
    }
    if (maze_walls[current] & WALL_N) { int n = idx(x, y+1); if (n != -1) maze_walls[n] |= WALL_S; }
    if (maze_walls[current] & WALL_E) { int n = idx(x+1, y); if (n != -1) maze_walls[n] |= WALL_W; }
    if (maze_walls[current] & WALL_S) { int n = idx(x, y-1); if (n != -1) maze_walls[n] |= WALL_N; }
    if (maze_walls[current] & WALL_W) { int n = idx(x-1, y); if (n != -1) maze_walls[n] |= WALL_E; }
}

void floodFill(int targetX, int targetY, bool isSpeedRun) {
    for (int i = 0; i < 256; i++) maze_dist[i] = 255;

    if (TARGET_X == 7 && TARGET_Y == 7 && isGoal(targetX, targetY)) {
        int g1 = idx(7, 7); if (g1 != -1) maze_dist[g1] = 0;
        int g2 = idx(7, 8); if (g2 != -1) maze_dist[g2] = 0;
        int g3 = idx(8, 7); if (g3 != -1) maze_dist[g3] = 0;
        int g4 = idx(8, 8); if (g4 != -1) maze_dist[g4] = 0;
    } else {
        int goal = idx(targetX, targetY);
        if (goal != -1) maze_dist[goal] = 0;
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (int x = 0; x < MAZE_W; x++) {
            for (int y = 0; y < MAZE_H; y++) {
                int curr = idx(x, y);
                if (curr == -1 || maze_dist[curr] == 255) continue;

                int dx[] = {0, 1, 0, -1};
                int dy[] = {1, 0, -1, 0};
                uint8_t masks[] = {WALL_N, WALL_E, WALL_S, WALL_W};

                for (int i = 0; i < 4; i++) {
                    if (!(maze_walls[curr] & masks[i])) {
                        int nb = idx(x + dx[i], y + dy[i]);
                        
                        if (nb != -1) {
                            if (isSpeedRun && !(maze_walls[nb] & VISITED)) continue;
                            
                            if (maze_dist[nb] > maze_dist[curr] + 1) {
                                maze_dist[nb] = maze_dist[curr] + 1;
                                changed = true;
                            }
                        }
                    }
                }
            }
        }
    }
}
