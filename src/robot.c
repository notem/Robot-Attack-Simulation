#include <malloc.h>
#include <assert.h>
#include <stdlib.h>
#include "robot.h"
#include "pathfinding.h"

/// initialize a robot
Robot makeRobot(size_t ID, Position pos, bool malicious) {
    Robot rob = safemalloc(sizeof *rob);
    rob->ID        = ID;
    rob->self      = pos;
    rob->target    = NULL;
    rob->assignment= NULL;
    rob->malicious = malicious;
    rob->buffer    = malloc(sizeof *(rob->buffer));
    return rob;
}

/// free memory locations associated with a robot
void freeRobot(Robot robot) {
    if(robot->self != NULL) {
        free(robot->self);
    }
    if(robot->assignment != NULL) {
        free(robot->assignment);
    }
    free(robot);
}

/// make the robot move to the next position as specified in it's buffer
void moveRobot(Robot robot) {
    if (abs(robot->buffer->x - robot->self->x)<=1
        && abs(robot->buffer->y - robot->self->y)<=1) {
        robot->self->x = robot->buffer->x;
        robot->self->y = robot->buffer->y;
    }
}

/// leader robot assigns positions for all robots to go to during the attack phase
void assignPositions(Robot leader, Robot* robots, size_t k, Position* objects, size_t o_size, size_t l, size_t b) {
    // for every malicious robot assign a phony assignment
    for (int x=0; x<k; x++) {
        if (robots[x]->malicious) {
            robots[x]->assignment = malloc(sizeof *(robots[x]->assignment));
            robots[x]->assignment->x = robots[x]->self->x;
            robots[x]->assignment->y = robots[x]->self->y;
        }
    }

    // assign every non-malicious robot a unique position around the target
    for (int j=0; j<k; j++) {
        int o = (j/8)+1;   // layer around target

        // determine next position to assign
        Position assignment = malloc(sizeof *assignment);
        switch (j%8) {
            case 0:
                assignment->x = leader->target->x;
                assignment->y = leader->target->y+o;
                break;
            case 1:
                assignment->x = leader->target->x+o;
                assignment->y = leader->target->y;
                break;
            case 2:
                assignment->x = leader->target->x;
                assignment->y = leader->target->y-o;
                break;
            case 3:
                assignment->x = leader->target->x-o;
                assignment->y = leader->target->y;
                break;
            case 4:
                assignment->x = leader->target->x+o;
                assignment->y = leader->target->y+o;
                break;
            case 5:
                assignment->x = leader->target->x+o;
                assignment->y = leader->target->y-o;
                break;
            case 6:
                assignment->x = leader->target->x-o;
                assignment->y = leader->target->y-o;
                break;
            case 7:
                assignment->x = leader->target->x-o;
                assignment->y = leader->target->y+o;
                break;
            default: // panic!
                assert(NULL);
                break;
        }

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
}
