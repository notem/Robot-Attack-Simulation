#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "simulation.h"
#include "utils/display.h"

/// seed the random number generator
void seed(long seed)  {
    // get a true random number
    srand(seed); // seed rand
}

/// Broadcast target location to all other robots
void broadcastTarget(Robot sender, Robot* robots, size_t k) {
    for (int i = 0; i < k; i++) {
        if (robots[i] != sender) {
            robots[i]->target = sender->target;
        }
    }
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

/// generate a random position available on the simulation grid
Position newPos(size_t l, size_t b, size_t k, Position* objects) {
    Position pos = safemalloc(sizeof *pos);
    pos->x = (int) (rand() % b);
    pos->y = (int) (rand() % l);

    // make sure that the new coordinate is not already occupied
    for(size_t j=0; j<k; j++) {
        // exhausted occupied list
        if (objects[j]==NULL) {
            break;
        } // found collision
        else if ((objects[j]->x==pos->x) && (objects[j]->y==pos->y)) {
            free(pos);                        // free the old position
            pos = newPos(l, b, k, objects);   // try again with new position
        }
    }
    return pos;
}

/// elect a leader for the robots using the bully algorithm
Robot elect_leader(Robot* robots, size_t k) {
    // Select random robot from list
    int leadIndex = (int) (rand() % k);
    printf("  Leader is %d\n", leadIndex);
    // Choose that as the leader
    return robots[0];
}

/// do one turn of the exploration stage
/// @returns true if the exploration stage has completed
bool explore(Robot* robots, Robot leader, Position target, size_t k, Position* objects, size_t o_size, size_t l, size_t b) {
    // For each robot, check if they found the target
    printf("  Target is at (%d, %d)\n", target->x, target->y);
    for (int i = 0; i < k; i++) {
        printf("  Robot %d is at (%d, %d)\n", i, robots[i]->self->x, robots[i]->self->y);

        // if the robot is within 1 tile of the target, robot 'sees' the target
        if (abs(robots[i]->self->x - target->x) <= 1 &&
                abs(robots[i]->self->y - target->y) <= 1) {
            printf("Robot #%d found the target!\n", i);

            // Set the target of the robot next to target
            robots[i]->target = target;
            Robot sender = robots[i];

            // Broadcast that target to every robot
            printf("Broadcasting location to all robots...\n");
            broadcastTarget(sender, robots, k);

            verifyTarget(robots, k);
            return true;
        }
    }

    // leader tells robots which position they should move to next
    directMovement(leader, robots, k, l, b);

    // robots move to their positions in parallel
    moveRobots(robots, k);

    return false;
}

/// do one turn of the exploration stage
/// @returns void
void transition(Robot* robots, Robot leader, size_t k, Position* objects, size_t o_size, size_t l, size_t b) {
    // print out robot's targets
    for (int i = 0; i < k; i++) {
        printf("  Robot %d believes that the target is at (%d, %d)\n", i,
               robots[i]->target->x, robots[i]->target->y);
    }

    // assign positions around the target for each robot
    // choices are based on ID of robot rather than optimal path lengths
    assignPositions(leader, robots, k, objects, o_size, l, b);

    // print out assignments for each robot
    for (int i = 0; i < k; i++) {
        if(robots[i]->assignment != NULL) {
            printf("  Robot %d's assigned spot is (%d, %d)\n", i,
                   robots[i]->assignment->x, robots[i]->assignment->y);
        }
    }
}

/// do one turn of the attack stage
/// @returns true if the attack stage has completed
bool attack(Robot* robots, Robot leader, size_t k, size_t l, size_t b) {
    // print robot positions
    for (int i = 0; i < k; i++) {
        printf("  Robot %d is at (%d, %d)\n", i, robots[i]->self->x, robots[i]->self->y);
    }

    // leader tells robots which position they should move to next
    directMovement(leader, robots, k, l, b);

    // robots move to their positions in parallel
    moveRobots(robots, k);

    // check if robots are in their assigned positions
    for (int i = 0; i < k; i++) {
        if (robots[i]->self->x != robots[i]->assignment->x ||
                robots[i]->self->y != robots[i]->assignment->y) {
            return false;   // attack stage is not yet finished
        }
    }
    return true;    // all robots in assigned positions, attack stage done
}

/// run the simulation
/// @returns the simulation's exit code
int run(size_t l, size_t b, size_t k, size_t e, long s) {
    assert(l>0 && b>0 && k>0);  // l & b & k must be nonzero
    assert(k > (3*e)+1 || k==1);// k must be greater than 3*e+1
    assert(k < l*b);            // k must be less than the total number of free spaces
    seed(s);                    // seed random

    /** Initialize target location and robots **/
    // array of positions on the grid which have been taken
    Position* objects = safecalloc(sizeof *(objects), k+1);
    size_t o_size  = 0;

    // determine position for the target
    Position target  = newPos(l, b, k, objects);
    objects[o_size] = target;      // add target pos to occupied list
    o_size++;

    // initialize all robots
    Robot* robots = safemalloc((sizeof *robots) * k);  // space for k pointers
    for(size_t j=0; j<k-e; j++) {                      // make good robots
        Position pos    = newPos(l, b, k, objects);
        robots[j]       = makeRobot(j, pos, false, l, b);
        objects[o_size] = pos;
        o_size++;
    }
    for(size_t j=k-e; j<k; j++) {                      // make bad robots
        Position pos    = newPos(l, b, k, objects);
        robots[j]       = makeRobot(j, pos, true, l, b);
        objects[o_size] = pos;
        o_size++;
    }

    // set the initial display setup
    update_display(l, b, k, 0, 0, robots, target);

    /** Begin the simulation loop **/
    int* phase = safecalloc(1, sizeof *phase);
    int round  = 0;
    while(*phase >= 0) {    // each loop is a turn in the simulation
        // Elect leader
        Robot leader = elect_leader(robots, k);
        switch (*phase)
        {
            case 0:     // exploration phase
                if (explore(robots, leader, target, k, objects, o_size, l, b)) {
                    *phase = 1; // simulation moves to transition/position assignment phase
                    printf("Entering transition phase...\n");
                    printf("==============\n");
                }
                round++;
                break;

            case 1:     // transition phase
                transition(robots, leader, k, objects, o_size, l, b);
                *phase = 2; // simulation moves to the attack phase
                printf("Entering attack phase...\n");
                printf("==============\n");
                break;

            case 2:     // attack phase
                if (attack(robots, leader, k, l, b)) {
                    *phase = -1; // simulation done
                };
                round++;
                break;

            default:    // error of some sort?
                assert(NULL);
                break;
        }

        /* block until user presses enter */
        printf("Hit [ENTER] to continue, or (q)uit: ");
        char* input = NULL; size_t size;
        getline(&input, &size, stdin);
        if(input[0]=='q' || input=="quit") *phase = -1;
        free(input);

        // update display for the next turn
        update_display(l, b, k, *phase, round, robots, target);
    }

    // print robot positions
    printf("Final positions of robots:\n");
    for (int i = 0; i < k; i++) {
        printf("  Robot %d is at (%d, %d)\n", i, robots[i]->self->x, robots[i]->self->y);
    }

    /** Free all initialized variables **/
    free(phase);
    free(target);
    for(size_t j=0; j<k; j++) {
        freeRobot(robots[j], l, b);
    }
    free(robots);
    free(objects);
    return EXIT_SUCCESS;
}
