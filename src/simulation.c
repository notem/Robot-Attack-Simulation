#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "simulation.h"
#include "robot.h"
#include "utils/display.h"

/// seed the random number generator
void seed()  {
    // get a true random number
    //int data = open("/dev/random", O_RDONLY);
    //unsigned int tmp;
    //read(data, &tmp, sizeof tmp);
    //srand(tmp); // seed rand
    srand(1); // seed rand
}

/// generate a random position available on the simulation grid
Position newPos(size_t l, size_t b, size_t k, Position *occupied) {
    Position pos = safemalloc(sizeof *pos);
    pos->y = rand()%l;
    pos->x = rand()%b;

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

/// run the simulation
int run(size_t l, size_t b, size_t k, size_t e) {
    assert(l>0 && b>0 && k>0);  // l & b & k must be nonzero
    assert(k > (3*e)+1);        // k must be greater than 3*e+1
    assert(k < l*b);            // k must be less than the total number of free spaces
    seed();             // seed random

    /** Initialize target location and robots **/
    // array of positions on the grid which have been taken
    Position *occupied = safecalloc(sizeof *occupied, k);
    size_t o_size      = 0;

    // determine position for the target
    Position target  = newPos(l, b, k, occupied);
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

    // set the initial display setup
    update_display(l, b, k, 0, 0, robots, target);

    /** Begin the simulation loop **/
    int round = 1;
    while(true) {

        /* block until user presses enter */
        printf("Hit [ENTER] to continue to next turn: ");
        char* tmp = NULL; size_t size;
        getline(&tmp, &size, stdin);
        free(tmp);

        // update display for the next turn
        update_display(l, b, k, 0, round++, robots, target);
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
