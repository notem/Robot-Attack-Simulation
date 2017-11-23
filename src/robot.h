#ifndef CSCI251_PROJECT3_ROBOT_H
#define CSCI251_PROJECT3_ROBOT_H

#include <glob.h>
#include <stdbool.h>
#include "utils/safemalloc.h"

typedef struct pos {
    size_t x;
    size_t y;
} *Position;

typedef struct robot {
    size_t ID;
    Position self;
    Position target;
    bool malicious;
} *Robot;

Robot makeRobot(size_t ID, Position pos, bool malicious);

#endif //CSCI251_PROJECT3_ROBOT_H
