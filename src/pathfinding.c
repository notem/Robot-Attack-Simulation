#include "robot.h"
#include <stdlib.h>
#include <stdint.h>
#include "pathfinding.h"

/// finds next batch of neighbors
/// @param neighbors    pointer to the array of current neighbor nodes
/// @param visited      pointer to the array of visited nodes
/// @param s_neighbors  pointer to the size of neighbors array
/// @param s_visited    pointer to the size of visited array
/// @param l            height of the simulation grid
/// @param b            width of the simulation grid
void find_neighbors(Position** neighbors, Position** visited, size_t* s_neighbors, size_t* s_visited, size_t l, size_t b) {
    /* add neighbors to visited nodes */
    *s_visited += *s_neighbors;
    *visited = realloc(*visited, (*s_visited) * (sizeof **visited));
    for (size_t j=0; j<*s_neighbors; j++) {
        (*visited)[j+(*s_visited)-(*s_neighbors)] = (*neighbors)[j];
    }

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
        // loop through new_neighbors to verify that a position is not
        // created and added to the neighbors list twice
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
        if ((*neighbors)[j]->y < l-1 && up) {    // up
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
        if ((*neighbors)[j]->x < b-1 && right) {   // right
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

/// Cleanup Position neighbors and visited nodes' allocated memory
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

/// Finds the size of the shortest path
size_t find_path(Position* objects, size_t o_size, Position target, Position source, size_t l, size_t b) {

    /* if target == source, return depth 1 */
    if (target->x == source->x && target->y == source->y) {
        return 1;
    }

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
    size_t depth = 1;
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

/// Find the first node of the shortest path
/// Uses find_path() to determine the size of the shortest path for
///   all possible branches (max 4) from the current position
Position shortest_path(Position* objects, size_t o_size, Position current, Position target, size_t l, size_t b) {

    /* create starting candidate as current position */
    Position candidate = safemalloc(sizeof *candidate);
    candidate->x = current->x; candidate->y = current->y;
    size_t top_value = SIZE_MAX;

    /* do not try branches which are blocked by an object */
    bool take_up=true, take_down=true, take_right=true, take_left=true;
    for (size_t e=0; e<o_size; e++) {
        if (objects[e]->x == current->x &&
            objects[e]->y == current->y+1) {
            take_up = false;
        }
        if (objects[e]->x == current->x &&
            objects[e]->y == current->y-1) {
            take_down = false;
        }
        if (objects[e]->x == current->x+1 &&
            objects[e]->y == current->y) {
            take_right = false;
        }
        if (objects[e]->x == current->x-1 &&
            objects[e]->y == current->y) {
            take_left = false;
        }
    }

    /* find path size for upwards branch */
    if(current->y < l-1 && take_up) {
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
    }

    /* find path size for downwards branch */
    if(current->y > 0 && take_down) {
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
    }

    /* find path size for rightwards branch */
    if(current->x < b-1 && take_right) {
        Position right = safemalloc(sizeof *right);
        right->x = current->x+1; right->y = current->y;
        size_t right_value = find_path(objects, o_size, target, right, l, b);
        if (right_value < top_value && right_value!=NULL) {
            free(candidate);
            candidate = right;
            top_value = right_value;
        } else {
            free(right);
        }
    }

    /* find path size for leftwards branch */
    if(current->x > 0 && take_left) {
        Position left  = safemalloc(sizeof *left);
        left->x = current->x-1; left->y = current->y;
        size_t left_value = find_path(objects, o_size, target, left, l, b);
        if (left_value < top_value && left_value!=NULL) {
            free(candidate);
            candidate = left;
        } else {
            free(left);
        }
    }

    return candidate;
}