#ifndef CSCI251_PROJECT3_ROBOT_H
#define CSCI251_PROJECT3_ROBOT_H

#include <glob.h>
#include <stdbool.h>
#include "utils/safemalloc.h"

typedef struct pos {
    int x;
    int y;
} *Position;

typedef struct robot {
    size_t ID;
    Position self;
    Position target;
    Position assignment;
    bool malicious;
} *Robot;

Robot makeRobot(size_t ID, Position pos, bool malicious);

void freeRobot(Robot robot);

#endif //CSCI251_PROJECT3_ROBOT_H
