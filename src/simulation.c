#include <stdlib.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>
#include "simulation.h"
#include "robot.h"

/// seed the random number generator
void seed()  {
    // get a true random number
    srand(1); // seed rand
}

// checks if a robot movement will bring it off the map
void robotEdge() {

}

// Broadcast target location to all other robots
void broadcastTarget(Robot sender, Robot* robots, size_t k) {
    for (int i = 0; i < k; i++) {
        if (robots[i] != sender) {
            robots[i]->target = sender->target;
        }
    }
}

/// generate a random position available on the simulation grid
Position newPos(size_t l, size_t b, size_t k, Position *occupied) {
    Position pos = safemalloc(sizeof *pos);
    pos->y = rand() % l;
    pos->x = rand() % b;

    // make sure that the new coordinate is not already occupied
    for(size_t j=0; j<k; j++) {
        // exhausted occupied list
        if (occupied[j]==NULL) {
            break;
        }// found collision
        else if ((occupied[j]->x==pos->x) && (occupied[j]->y==pos->y)) {
            free(pos);                         // free the old position
            pos = newPos(l, b, k, occupied);   // try again with new position
        }
    }
    return pos;
}

/// elect a leader for the robots using the bully algorithm
Robot elect_leader(Robot* robots, size_t k) {
    // Select random robot from list
    int leadIndex = rand() % k;
    printf("Leader is %d\n", leadIndex);
    // Choose that as the leader
    return robots[leadIndex];
}

/// do one turn of the exploration stage
bool explore(Robot* robots, Robot leader, size_t k, Position target) {
    // TODO (Currently Doing)

    // For each robot, check if they found the target
    for (int i = 0; i < k; i++) {
        printf("Robot %d is at (%d, %d)\n", i,
               robots[i]->self->x, robots[i]->self->y);
        if (abs(robots[i]->self->x - target->x) == 1 &&
                abs(robots[i]->self->y - target->y == 1)) {
            printf("Robot #%d found the target!\n", i);

            // Set the target of the robot next to target
            robots[i]->target = target;
            Robot sender = robots[i];

            // Broadcast that target to every robot
            printf("Broadcasting location to all robots...\n");
            broadcastTarget(sender, robots, k);

            // Later On: Verify everyone has the same one

            return true;
        }
    }

    // Otherwise, leader picks direction to travel in
    /*for (int i = 0; i < k; i++) {
        int direction = rand() % 4; // four directions
        switch (direction) {
            case 0: // up
                break;
            case 1: // down
                break;
            case 2: // left
                break;
            case 3: // right
                break;
            default:
                assert(NULL);
                break;
        }

    }*/

    // May need a check so robot doesn't go off edge

    return false;
}

/// do one turn of the exploration stage
bool transition(Robot* robots, Robot leader, size_t k) {
    // TODO (Doing Currently)

    // TESTING: See if all robots have same target
    for (int i = 0; i < k; i++) {
        printf("Robot %d's target is (%d, %d)\n", i,
               robots[i]->target->x, robots[i]->target->y);
    }

    return false;
}

/// do one turn of the exploration stage
bool attack(Robot* robots, Robot leader) {
    // TODO
    return false;
}

/// run the simulation
int run(size_t l, size_t b, size_t k, size_t e) {
    assert(l>0 && b>0 && k>0);  // l & b & k must be nonzero
    assert(k > (3*e)+1);        // k must be greater than 3*e+1
    seed();

    /** Initialize target location and robots **/
    // array of positions on the grid which have been taken
    Position *occupied = safecalloc(sizeof *occupied, k);
    size_t o_size      = 0;

    // determine position for the target
    Position target  = newPos(l, b, k, occupied);
    printf("Target is at (%d, %d)\n", target->x, target->y);
    occupied[o_size] = target;      // add target pos to occupied list
    o_size++;

    // initialize all robots
    Robot* robots = safemalloc((sizeof *robots) * k);  // space for k pointers
    for(size_t j=0; j<k-e; j++) {                      // make good robots
        Position pos     = newPos(l, b, k, occupied);
        robots[j]        = makeRobot(j, pos, false);
        occupied[o_size] = pos;
        o_size++;
    }
    for(size_t j=k-e; j<k; j++) {                      // make bad robots
        Position pos     = newPos(l, b, k, occupied);
        robots[j]        = makeRobot(j, pos, true);
        occupied[o_size] = pos;
        o_size++;
    }

    /** Begin the simulation loop **/
    int* phase = safecalloc(1, sizeof *phase);
    int temp = 0;
    while(true) {       // each loop is a turn in the simulation
        if(*phase == 4) break;

        // Elect leader
        Robot leader = elect_leader(robots, k);
        switch (*phase)
        {
            case 0:     // exploration phase
                if(explore(robots, leader, k, target)) {
                    *phase = 1;                     // simulation moves to next stage
                    printf("Entering transition phase...\n");
                    printf("==============\n");
                }
                break;

            case 1:     // transition phase
                transition(robots, leader, k);
                *phase = 2;
                printf("Entering attack phase...\n");
                printf("==============\n");
                break;

            case 2:     // attack phase
                if(attack(robots, leader)) {
                    *phase = 4;
                };
                break;

            default:    // error of some sort?
                assert(NULL);
                break;
        }

        temp++;
        if (temp == 10) {
        *phase = 4;}
    }

    /** Free all initialized variables **/
    for(size_t j=0; j<o_size; j++) {
        free(occupied[j]);
    }
    free(occupied);
    for(size_t j=0; j<k; j++) {
        free(robots[j]);
    }
    free(robots);
    return EXIT_SUCCESS;
}
