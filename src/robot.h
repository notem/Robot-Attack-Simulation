#ifndef CSCI251_PROJECT3_ROBOT_H
#define CSCI251_PROJECT3_ROBOT_H

#include <glob.h>
#include <stdbool.h>
#include "utils/safemalloc.h"

/// basic position structure
typedef struct pos {
    int x;
    int y;
} *Position;

/// Robot object
typedef struct robot {
    size_t ID;              // unique ID for robot
    bool** explored;             // 2d array describing positions that are known
    Position self;              // the robot's position
    Position target;            // the known position of the target
    Position assignment;        // the robot's assigned target position
    Position receive_buffer;    // position in the robot's receive buffer
    Position send_buffer;       // position in the robot's send buffer
    bool malicious;         // is the robot malicious?
} *Robot;

/// create a new robot in dynamically allocated space
Robot makeRobot(size_t ID, Position pos, bool malicious, size_t l, size_t b);

/// free a robot from the chains of life
void freeRobot(Robot robot, size_t l, size_t b);

/// the leader robot assigns positions around the target for all robots
/// this function is used only during the transition phase
void assignPositions(Robot leader, Robot* robots, size_t k, Position* objects, size_t o_size, size_t l, size_t b);

/// the leader robot instructs each robot with the tile to move to in the next movement turn
/// this function is used by the elected leader during both the exploration and attack phase
void directMovement(Robot leader, Robot* robots, size_t k, size_t l, size_t b);

/// move robots to the position in their receive_buffer (if valid)
/// robots move in parallel using pthreads
void moveRobots(Robot* robots, size_t k);

#endif //CSCI251_PROJECT3_ROBOT_H
