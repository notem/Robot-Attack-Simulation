#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "simulation.h"
#include "robot.h"
#include "utils/display.h"

/// seed the random number generator
void seed()  {
    // get a true random number
    srand(1); // seed rand
}

/// Broadcast target location to all other robots
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
    pos->x = (int) (rand() % b);
    pos->y = (int) (rand() % l);

    // make sure that the new coordinate is not already occupied
    for(size_t j=0; j<k; j++) {
        // exhausted occupied list
        if (occupied[j]==NULL) {
            break;
        } // found collision
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
    int leadIndex = (int) (rand() % k);
    printf("Leader is %d\n", leadIndex);
    // Choose that as the leader
    return robots[leadIndex];
}

/// do one turn of the exploration stage
/// @returns true if the exploration stage has completed
bool explore(Robot* robots, Robot leader, Position target, size_t k, size_t l, size_t b) {
    // For each robot, check if they found the target
    printf("Target is at (%d, %d)\n", target->x, target->y);
    for (int i = 0; i < k; i++) {
        printf("Robot %d is at (%d, %d)\n", i, robots[i]->self->x, robots[i]->self->y);

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

            // TODO do consensus algorithm to verify target location
            return true;
        }
    }

    /* each robot wanders in a random direction
       if the chosen direction is towards the edge of the simulation, try a new postion
       if the chosen direction is towards another robot, stay in place  */
    for (int i = 0; i < k; i++) {
        head:;
        int direction = rand() % 4; // four directions
        bool safe = true;
        switch (direction) {
            case 0: // up
                if(robots[i]->self->x >= b) goto head;
                for (int j = 0; j < k; j++) {
                    if(j!=i && robots[j]->self->x == robots[i]->self->x+1) {
                        safe = false;
                        break;
                    }
                }
                if(safe) robots[i]->self->x += 1;
                break;
            case 1: // down
                if(robots[i]->self->x <= 0) goto head;
                for (int j = 0; j < k; j++) {
                    if(j!=i && robots[j]->self->x == robots[i]->self->x-1) {
                        safe = false;
                        break;
                    }
                }
                if(safe) robots[i]->self->x -= 1;
                break;
            case 2: // left
                if(robots[i]->self->y >= l) goto head;
                for (int j = 0; j < k; j++) {
                    if(j!=i && robots[j]->self->y == robots[i]->self->y+1) {
                        safe = false;
                        break;
                    }
                }
                if(safe) robots[i]->self->y += 1;
                break;
            case 3: // right
                if(robots[i]->self->y <= 0) goto head;
                for (int j = 0; j < k; j++) {
                    if(j!=i && robots[j]->self->y == robots[i]->self->y-1) {
                        safe = false;
                        break;
                    }
                }
                if(safe) robots[i]->self->y -= 1;
                break;
            default:// madness?
                assert(NULL);
                break;
        }
    }
    return false;
}

/// do one turn of the exploration stage
/// @returns void
void transition(Robot* robots, Robot leader, size_t k) {
    // TESTING: See if all robots have same target
    for (int i = 0; i < k; i++) {
        printf("Robot %d's target is (%d, %d)\n", i,
               robots[i]->target->x, robots[i]->target->y);
    }

    // assign positions around the target for each robot
    // choices are based on ID of robot rather than optimal path lengths TODO use shortest path for decision making
    for (int j=0; j<k; j++) {
        int l = (j/8)+1;   // layer around target
        robots[j]->assignment = malloc(sizeof *(robots[j]->assignment));
        switch (j%8) {
            case 0:
                robots[j]->assignment->x = robots[j]->target->x;
                robots[j]->assignment->y = robots[j]->target->y+l;
                break;
            case 1:
                robots[j]->assignment->x = robots[j]->target->x+l;
                robots[j]->assignment->y = robots[j]->target->y;
                break;
            case 2:
                robots[j]->assignment->x = robots[j]->target->x;
                robots[j]->assignment->y = robots[j]->target->y-l;
                break;
            case 3:
                robots[j]->assignment->x = robots[j]->target->x-l;
                robots[j]->assignment->y = robots[j]->target->y;
                break;
            case 4:
                robots[j]->assignment->x = robots[j]->target->x+l;
                robots[j]->assignment->y = robots[j]->target->y+l;
                break;
            case 5:
                robots[j]->assignment->x = robots[j]->target->x+l;
                robots[j]->assignment->y = robots[j]->target->y-l;
                break;
            case 6:
                robots[j]->assignment->x = robots[j]->target->x-l;
                robots[j]->assignment->y = robots[j]->target->y-l;
                break;
            case 7:
                robots[j]->assignment->x = robots[j]->target->x-l;
                robots[j]->assignment->y = robots[j]->target->y+l;
                break;
            default: // panic!
                assert(NULL);
                break;
        }
    }

    // print out assignments for each robot
    for (int i = 0; i < k; i++) {
        printf("Robot %d's assigned spot is (%d, %d)\n", i,
               robots[i]->assignment->x, robots[i]->assignment->y);
    }
}

/// @returns nothing
void find_neighbors(Position** neighbors, Position** visited, size_t* s_neighbors, size_t* s_visited, size_t l, size_t b) {

    /* add neighbors to visited nodes */
    *s_visited += *s_neighbors;
    *visited = realloc(*visited, (*s_visited) * (sizeof **visited));
    for (size_t j=0; j<*s_neighbors; j++) {
        (*visited)[j+(*s_visited)-(*s_neighbors)] = (*neighbors)[j];
        //printf("(%d,%d), ", (*neighbors)[j]->x, (*neighbors)[j]->y);
    }
    //printf("\n");

    /* generate next depth-level's neighbors */
    size_t counter = 0;
    Position* new_neighbors = safemalloc((sizeof *new_neighbors)*(*s_neighbors)*4);
    for (size_t j=0; j<*s_neighbors; j++) { // for each node in current neighbors list
        // loop through visited nodes and determine if candidate
        // neighbors have been visited already
        bool up=true, down=true, right=true, left=true;
        for (size_t e=0; e<*s_visited; e++) {
            if ((*visited)[e]->x == (*neighbors)[j]->x &&
                    (*visited)[e]->y == (*neighbors)[j]->y+1) {
                up = false;
            }
            if ((*visited)[e]->x == (*neighbors)[j]->x &&
                    (*visited)[e]->y == (*neighbors)[j]->y-1) {
                down = false;
            }
            if ((*visited)[e]->x == (*neighbors)[j]->x+1 &&
                    (*visited)[e]->y == (*neighbors)[j]->y) {
                right = false;
            }
            if ((*visited)[e]->x == (*neighbors)[j]->x-1 &&
                    (*visited)[e]->y == (*neighbors)[j]->y) {
                left = false;
            }
        }
        for (size_t e=0; e<counter; e++) {
            if ((new_neighbors)[e]->x == (*neighbors)[j]->x &&
                (new_neighbors)[e]->y == (*neighbors)[j]->y+1) {
                up = false;
            }
            if ((new_neighbors)[e]->x == (*neighbors)[j]->x &&
                (new_neighbors)[e]->y == (*neighbors)[j]->y-1) {
                down = false;
            }
            if ((new_neighbors)[e]->x == (*neighbors)[j]->x+1 &&
                (new_neighbors)[e]->y == (*neighbors)[j]->y) {
                right = false;
            }
            if ((new_neighbors)[e]->x == (*neighbors)[j]->x-1 &&
                (new_neighbors)[e]->y == (*neighbors)[j]->y) {
                left = false;
            }
        }

        // if candidate neighbor has not been visited and is within bounds,
        // add the candidate to the new_neighbors list
        if ((*neighbors)[j]->y < l && up) {    // up
            new_neighbors[counter] = safemalloc(sizeof **new_neighbors);
            new_neighbors[counter]->x = (*neighbors)[j]->x;
            new_neighbors[counter]->y = (*neighbors)[j]->y+1;
            counter++;
        }
        if ((*neighbors)[j]->y > 0 && down) {  // down
            new_neighbors[counter] = safemalloc(sizeof **new_neighbors);
            new_neighbors[counter]->x = (*neighbors)[j]->x;
            new_neighbors[counter]->y = (*neighbors)[j]->y-1;
            counter++;
        }
        if ((*neighbors)[j]->x < b && right) {   // right
            new_neighbors[counter] = safemalloc(sizeof **new_neighbors);
            new_neighbors[counter]->x = (*neighbors)[j]->x+1;
            new_neighbors[counter]->y = (*neighbors)[j]->y;
            counter++;
        }
        if ((*neighbors)[j]->x > 0 && left) {    // left
            new_neighbors[counter] = safemalloc(sizeof **new_neighbors);
            new_neighbors[counter]->x = (*neighbors)[j]->x-1;
            new_neighbors[counter]->y = (*neighbors)[j]->y;
            counter++;
        }
    }

    /* free old neighbors list and shrink new_neighbors to true size */
    free(*neighbors);
    *neighbors = realloc(new_neighbors, (sizeof **neighbors)*counter);
    *s_neighbors = counter;
}

/// cleanup memory
void free_nodes(Position* neighbors, Position* visited, size_t s_neighbors, size_t s_visited) {
    for (size_t i=0; i<s_neighbors; i++) {
        free(neighbors[i]);
    }
    free(neighbors);
    for (size_t i=0; i<s_visited; i++) {
        free(visited[i]);
    }
    free(visited);
}

/// @returns size of path found; if no path was found (or source = target) return NULL
size_t find_path(Position* objects, size_t o_size, Position target, Position source, size_t l, size_t b) {
    /* find neighbors for 1st level depth */
    Position* neighbors = safemalloc(sizeof *neighbors);    // neighboring nodes
    neighbors[0]        = safemalloc(sizeof **neighbors);
    neighbors[0]->x = source->x; neighbors[0]->y = source->y;
    size_t s_neighbors  = 1;    // size of neighbors

    /* fill list of visited nodes with object positions */
    Position* visited = safemalloc((sizeof *neighbors)*o_size); // visited nodes
    size_t s_visited  = o_size;                             // size of visited
    for(size_t j=0; j<o_size; j++) {            // copy objects into visited
        Position pos = safemalloc(sizeof *pos);
        pos->x = objects[j]->x;
        pos->y = objects[j]->y;
        visited[j] = pos;
    }
    find_neighbors(&neighbors, &visited, &s_neighbors, &s_visited, l, b);

    /* execute depth-first search for the target position */
    size_t depth = 0;
    while (s_neighbors > 0) {
        for (size_t i=0; i<s_neighbors; i++) {
            if(target->x == neighbors[i]->x
               && target->y == neighbors[i]->y) {
                free_nodes(neighbors, visited, s_neighbors, s_visited);
                return depth;
            }
        }
        find_neighbors(&neighbors, &visited, &s_neighbors, &s_visited, l, b);
        depth++;
    }
    free_nodes(neighbors, visited, s_neighbors, s_visited);
    return NULL;    // target could not be reached
}

/// determines the shortest path from a robots current position to it's assigned position,
/// accounting for obstacles in between
/// @returns the next Position node in the shortest path, must be freed after use
Position shortest_path(Position* objects, size_t o_size, Position current, Position target, size_t l, size_t b) {

    /* create starting candidate as current position */
    Position candidate = safemalloc(sizeof *candidate);
    candidate->x = current->x; candidate->y = current->y;
    size_t top_value = SIZE_MAX;

    /* find path size for upwards branch */
    Position up = safemalloc(sizeof *up);
    up->x = current->x; up->y = current->y+1;
    size_t up_value = find_path(objects, o_size, target, up, l, b);
    if (up_value < top_value && up_value!=NULL) {
        free(candidate);
        candidate = up;
        top_value = up_value;
    } else {
        free(up);
    }

    /* find path size for downwards branch */
    Position down = safemalloc(sizeof *down);
    down->x = current->x; down->y = current->y-1;
    size_t down_value = find_path(objects, o_size, target, down, l, b);
    if (down_value < top_value && down_value!=NULL) {
        free(candidate);
        candidate = down;
        top_value = down_value;
    } else {
        free(down);
    }

    /* find path size for rightwards branch */
    Position right = safemalloc(sizeof *right);
    right->x = current->x-1; right->y = current->y;
    size_t right_value = find_path(objects, o_size, target, down, l, b);
    if (down_value < top_value && down_value!=NULL) {
        free(candidate);
        candidate = right;
        top_value = right_value;
    } else {
        free(right);
    }

    /* find path size for leftwards branch */
    Position left  = safemalloc(sizeof *left);
    left->x = current->x+1; left->y = current->y;
    size_t left_value = find_path(objects, o_size, target, down, l, b);
    if (top_value==NULL || (left_value < top_value && left_value!=NULL)) {
        free(candidate);
        candidate = left;
    } else {
        free(left);
    }

    return candidate;
}

/// do one turn of the attack stage
/// @returns true if the attack stage has completed
bool attack(Robot* robots, Robot leader, size_t k, Position* objects, size_t o_size, size_t l, size_t b) {
    // print out assignments for each robot
    for (int i = 0; i < k; i++) {
        printf("Robot %d with assignment (%d,%d) is at (%d, %d)\n", i,robots[i]->assignment->x,robots[i]->assignment->y, robots[i]->self->x, robots[i]->self->y);
    }
    // each robots calculates the shortest path to the target and moves forward one position
    for (int i = 0; i < k; i++) {
        if (robots[i]->self->x != robots[i]->assignment->x || robots[i]->self->y != robots[i]->assignment->y) {
            Position pos = shortest_path(objects, o_size, robots[i]->self, robots[i]->assignment, l, b);
            robots[i]->self->x = pos->x;
            robots[i]->self->y = pos->y;
            free(pos);
        }
    }

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
int run(size_t l, size_t b, size_t k, size_t e) {
    assert(l>0 && b>0 && k>0);  // l & b & k must be nonzero
    assert(k > (3*e)+1);        // k must be greater than 3*e+1
    assert(k < l*b);            // k must be less than the total number of free spaces
    seed();             // seed random

    /** Initialize target location and robots **/
    // array of positions on the grid which have been taken
    Position *objects = safecalloc(sizeof *objects, k);
    size_t o_size      = 0;

    // determine position for the target
    Position target  = newPos(l, b, k, objects);
    printf("Target is at (%d, %d)\n", target->x, target->y);
    objects[o_size] = target;      // add target pos to occupied list
    o_size++;

    // initialize all robots
    Robot* robots = safemalloc((sizeof *robots) * k);  // space for k pointers
    for(size_t j=0; j<k-e; j++) {                      // make good robots
        Position pos     = newPos(l, b, k, objects);
        robots[j]        = makeRobot(j, pos, false);
        objects[o_size] = pos;
        o_size++;
    }
    for(size_t j=k-e; j<k; j++) {                      // make bad robots
        Position pos     = newPos(l, b, k, objects);
        robots[j]        = makeRobot(j, pos, true);
        objects[o_size] = pos;
        o_size++;
    }

    // set the initial display setup
    update_display(l, b, k, 0, 0, robots, target);

    /** Begin the simulation loop **/
    int* phase = safecalloc(1, sizeof *phase);
    int round  = 1;
    while(*phase >= 0) {    // each loop is a turn in the simulation
        // Elect leader
        Robot leader = elect_leader(robots, k);
        switch (*phase)
        {
            case 0:     // exploration phase
                if (explore(robots, leader, target, k, l, b)) {
                    *phase = 1; // simulation moves to transition/position assignment phase
                    printf("Entering transition phase...\n");
                    printf("==============\n");
                }
                break;

            case 1:     // transition phase
                transition(robots, leader, k);
                *phase = 2; // simulation moves to the attack phase
                printf("Entering attack phase...\n");
                printf("==============\n");
                break;

            case 2:     // attack phase
                if (attack(robots, leader, k, objects, o_size, l, b)) {
                    *phase = -1; // simulation done
                };
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
        update_display(l, b, k, *phase, round++, robots, target);
    }

    /** Free all initialized variables **/
    for(size_t j=0; j<o_size; j++) {
        free(objects[j]);
    }
    free(objects);
    for(size_t j=0; j<k; j++) {
        free(robots[j]);
    }
    free(robots);
    return EXIT_SUCCESS;
}
