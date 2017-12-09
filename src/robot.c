#include "simulation.h"
#include <malloc.h>
#include <assert.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include "robot.h"
#include "pathfinding.h"

/// initialize a robot
Robot makeRobot(size_t ID, Position pos, bool malicious, size_t l, size_t b) {
    Robot rob = safemalloc(sizeof *rob);
    rob->ID             = ID;
    rob->self           = pos;
    rob->target         = NULL;
    rob->assignment     = NULL;
    rob->malicious      = malicious;
    rob->receive_buffer = safemalloc(sizeof *(rob->receive_buffer));
    rob->send_buffer    = safemalloc(sizeof *(rob->send_buffer));

    /* initialize the explore mapping */
    rob->explored = safemalloc(sizeof *(rob->explored) * b);
    for (int x=0;x<b;x++) {
        rob->explored[x] = calloc(sizeof *(rob->explored), l);
        for (int y=0;y<l;y++) {
            rob->explored[x][y] = false;
        }
    }
    return rob;
}

/// free memory locations associated with a robot
void freeRobot(Robot robot, size_t l, size_t b) {
    if(robot->self != NULL) {
        free(robot->self);
    }
    if(robot->assignment != NULL) {
        free(robot->assignment);
    }
    free(robot->receive_buffer);
    free(robot->send_buffer);
    for (int x=0;x<b;x++) {
        free(robot->explored[x]);
    }
    free(robot->explored);
    free(robot);
}

/// make the robot move to the next position as specified in it's buffer
void* moveRobot(void* robot_void) {
    Robot robot = (Robot) robot_void;
    if (abs(robot->receive_buffer->x - robot->self->x)<=1
        && abs(robot->receive_buffer->y - robot->self->y)<=1) {
        robot->self->x = robot->receive_buffer->x;
        robot->self->y = robot->receive_buffer->y;
    }
}

/// make the robot move to the next position as specified in it's buffer
void moveRobots(Robot* robots, size_t k) {
    pthread_t* threadpool = safemalloc(k * sizeof *threadpool);
    for (int i = 0; i < k; i++) {
        int code = pthread_create(&threadpool[i], NULL, &moveRobot, robots[i]);
        if (code) {
            printf("Thread creation failed!");
            exit(code);
        };
    }
    for (size_t t=0; t < k; t++) {
        pthread_join(threadpool[t], NULL);
    }
    free(threadpool);
}

/// find the closest unknown position on the grid
Position getFirstUnknown(Position cur, bool** known, size_t l, size_t b) {
    // (di, dj) is a vector - direction in which we move right now
    int di = 1; int dj = 0; int segment_length = 1;

    // current position (i, j) and how much of current segment we passed
    int i = cur->x; int j = cur->y; int segment_passed = 0;
    for (int k = 0; k < l*b*4; ++k) {
        if (i<b && j<l) {   // only if the point on spiral is within bounds
            // if an unknown is found break
            if(!known[i][j]) {
                break;
            }
        }
        // make a step, add 'direction' vector (di, dj) to current position (i, j)
        i += di; j += dj; ++segment_passed;
        if (segment_passed == segment_length) {
            segment_passed = 0;

            // 'rotate' directions
            int buffer = di;
            di = -dj;
            dj = buffer;
            // increase segment length if necessary
            if (dj == 0) {
                ++segment_length;
            }
        }
    }

    // create the pos object and return it
    Position pos = safemalloc(sizeof *pos);
    pos->x = i;
    pos->y = j;
    return pos;
}

/// leader robot assigns positions for all robots to go to during the attack phase
void assignPositions(Robot leader, Robot* robots, size_t k, Position* objects, size_t o_size, size_t l, size_t b) {
    // for every malicious robot assign a phony assignment
    int x = (leader->target->x > (b/2) ? 0 : (int) b-1);
    int y = (leader->target->y > (l/2) ? 0 : (int) l-1);
    int count = 0;
    for (int j=0; j<k; j++) {
        if (robots[j]->malicious) {
            robots[j]->assignment = safemalloc(sizeof *(robots[j]->assignment));
            robots[j]->assignment->x = x;
            robots[j]->assignment->y = y;

            // iterate to a new position for next malicious bot
            if ((count++)%2) {
                if (leader->target->x > (b/2)) { x++; }
                else { x--; }
            } else {
                if (leader->target->y > (l/2)) { y++; }
                else { y--; }
            }
        }
    }

    // boolean mapping of positions taken
    bool **filled = safemalloc(l*b*sizeof *filled);
    for (int i=0;i<b;i++) {
        filled[i] = calloc(sizeof **filled, l);
        for (int j=0;j<l;j++) {
            filled[i][j] = false;
        }
    }
    filled[leader->target->x][leader->target->y] = true;

    int phase = 0;
    int dir = 0;
    int numPos = 0;
    //int layer = 1;  // current layer from the target (when surrounding)
    Position* posList = safemalloc(k * sizeof *posList);
    while (numPos < k) {
        int currentNum = numPos;
        int currentPhase = phase;
        Position assignment = safemalloc(sizeof *assignment);
        switch (phase) {
            case 0: // Handles North, South, East and West
                switch (dir) {
                    case 0: // North of Target
                        if (leader->target->y == 0) {
                            break;
                        }
                        assignment->x = leader->target->x;
                        assignment->y = leader->target->y - 1;
                        numPos++;
                        break;
                    case 1: // South of Target
                        if (leader->target->y == l - 1) {
                            break;
                        }
                        assignment->x = leader->target->x;
                        assignment->y = leader->target->y + 1;
                        numPos++;
                        break;
                    case 2: // East of Target
                        if (leader->target->x == b - 1) {
                            break;
                        }
                        assignment->x = leader->target->x + 1;
                        assignment->y = leader->target->y;
                        numPos++;
                        break;
                    case 3: // West of Target
                        phase++;    // Move on to diagonals
                        if (leader->target->x == 0) {
                            break;
                        }
                        assignment->x = leader->target->x - 1;
                        assignment->y = leader->target->y;
                        numPos++;
                        break;
                    default: // oops, something went wrong!
                        assert(NULL);
                        break;
                }
                break;
            case 1: // Handles the four corners (NE, SE, NW, SW)
                switch (dir) {
                    case 0: // Northeast
                        if (leader->target->x == b - 1 || leader->target->y == 0) {
                            break;
                        }
                        assignment->x = leader->target->x + 1;
                        assignment->y = leader->target->y - 1;
                        numPos++;
                        break;
                    case 1: // Southeast
                        if (leader->target->x == b - 1 || leader->target->y == l - 1) {
                            break;
                        }
                        assignment->x = leader->target->x + 1;
                        assignment->y = leader->target->y + 1;
                        numPos++;
                        break;
                    case 2: // Northwest
                        if (leader->target->x == 0 || leader->target->y == 0) {
                            break;
                        }
                        assignment->x = leader->target->x - 1;
                        assignment->y = leader->target->y - 1;
                        numPos++;
                        break;
                    case 3: // Southwest
                        phase++;
                        dir = 0;
                        if (leader->target->x == 0 || leader->target->y == l - 1) {
                            break;
                        }
                        assignment->x = leader->target->x - 1;
                        assignment->y = leader->target->y + 1;
                        numPos++;
                        break;
                    default:    // something went wrong!
                        assert(NULL);
                        break;
                }
                break;
            case 2: // Handles the second layer onward if applicable
                free(assignment);
                assignment = getFirstUnknown(leader->target, filled, l, b);
                numPos++;
                break;
            default: // oops, something went wrong!
                assert(NULL);
                break;
        }

        if (currentPhase == phase) {
            dir++;  // move onto next direction
        } else {
            dir = 0;
        }

        // check if a new position was made
        if (currentNum == numPos) {
            free(assignment);
        } else {
            posList[numPos - 1] = assignment;
            filled[assignment->x][assignment->y] = true;
        }
    }
    ////// ^ This code ^ //////////

    // assign every non-malicious robot a unique position around the target
    for (int j=0; j<k; j++) {
        // determine next position to assign
        Position assignment = posList[j];

        // for every robot not yet assigned, award the current assignment
        // to the robot with the shortest path to travel
        Robot* top_rob   = NULL;
        size_t path_size = NULL;
        for (int x=0; x<k; x++) {
            // if the robot is not malicious and assignment is not yet set
            if (robots[x]->assignment == NULL) {
                // generate the shortest path size
                size_t s = find_path(objects, o_size, assignment, robots[x]->self, l, b);
                if (top_rob == NULL || path_size > s && s!=NULL) {
                    top_rob   = &(robots[x]);
                    path_size = s;
                }
            }
        }

        // assign the spot to the shortest path
        if (top_rob != NULL) {
            (*top_rob)->assignment = assignment;
        } else {
            free(assignment);
        }
    }

    // free stuff
    for (int i=0;i<b;i++) {
        free(filled[i]);
    }
    free(filled);
    free(posList);
}

/// leader robot directs tertiary robots next move
void directMovement(Robot leader, Robot* robots, size_t k, size_t l, size_t b) {

    // create temporary array of all logical positions to avoid collisions
    Position* next_positions = safemalloc((k+1)*(sizeof *next_positions));
    for (int i = 0; i < k; i++) {
        next_positions[i] = safemalloc(sizeof **next_positions);
        next_positions[i]->x = robots[i]->self->x;
        next_positions[i]->y = robots[i]->self->y;
    }

    // during the exploration phase
    if(leader->assignment == NULL) {
        for (int i = 0; i < k; i++) {
            leader->explored[robots[i]->self->x][robots[i]->self->y] = true;
            Position unknown = getFirstUnknown(robots[i]->self, leader->explored, l, b);
            Position pos = shortest_path(next_positions, k, robots[i]->self, unknown, l, b);
            next_positions[i]->x = pos->x;
            next_positions[i]->y = pos->y;

            // leader prepares the position to send
            leader->send_buffer->x = pos->x;
            leader->send_buffer->y = pos->y;

            // leader tells the robot it's next position
            robots[i]->receive_buffer->x = leader->send_buffer->x;
            robots[i]->receive_buffer->y = leader->send_buffer->y;

            free(unknown);
            free(pos);
        }
    }// during the attack phase
    else {
        // target is known, add target to object positions
        next_positions[k] = safemalloc(sizeof **next_positions);
        next_positions[k]->x = leader->target->x;
        next_positions[k]->y = leader->target->y;

        // leader tells each robot their next position
        for (int i = 0; i < k; i++) {
            Position pos = shortest_path(next_positions, k+1, robots[i]->self, robots[i]->assignment, l, b);
            next_positions[i]->x = pos->x;
            next_positions[i]->y = pos->y;

            // leader prepares the position to send
            leader->send_buffer->x = pos->x;
            leader->send_buffer->y = pos->y;

            // leader tells the robot it's next position
            robots[i]->receive_buffer->x = leader->send_buffer->x;
            robots[i]->receive_buffer->y = leader->send_buffer->y;

            free(pos);
        }

        // free stuff
        for (int i = 0; i <= k; i++) {
            free(next_positions[i]);
        }
        free(next_positions);
    }
}

/// Broadcast target location to all other robots
void broadcastTarget(Robot sender, Robot* robots, size_t k) {
    for (int i = 0; i < k; i++) {
        if (robots[i] != sender) {
            robots[i]->target = sender->target;
        }
    }
}

/// elect a leader for the robots using the bully algorithm
Robot electLeader(Robot* robots, size_t k) {
    // Loop through array of robots to find the lowest robot ID
    Robot leader = NULL;
    for (size_t x=0; x<k; x++) {
        if (leader == NULL) {
            leader = robots[x];
        } else if (robots[x]->ID < leader->ID) {
            leader = robots[x];
        }
    }
    // Choose that as the leader
    printf("  Leader is %zu\n", leader->ID);
    return robots[0];
}

/// All of the robots verify with the leader that they have the correct target
/// Checks if a robot is evil
void verifyTarget(Robot* robots, size_t k) {
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) {
            if (robots[j] != robots[i]) {

                if (robots[j]->malicious) {
                    if (robots[i]->malicious) {
                        printf("Robot %d(m) verified with Robot %d(m)\n", i, j);
                    } else {
                        printf("Robot %d found Robot %d to be malicious!\n", i, j);
                    }
                } else if (robots[j]->target == robots[i]->target) {
                    printf("Robot %d verified with Robot %d\n", i, j);
                } else {
                    printf("Uh oh, something went wrong!\n");
                }

            }
        }
    }
}