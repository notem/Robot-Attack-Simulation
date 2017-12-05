#ifndef CSCI251_PROJECT3_PATHFINDING_H
#define CSCI251_PROJECT3_PATHFINDING_H

#include "robot.h"

/// Determines the shortest path from a robots current position to it's assigned position,
/// accounting for obstacles in between
/// @returns the next Position node in the shortest path, must be free'd after use
Position shortest_path(Position* objects, size_t o_size, Position current, Position target, size_t l, size_t b);

/// Finds the path length of the shortest path.
/// If the source == target position, the path length is 1.
/// Implements breadth-first search algorithm.
/// @returns size of path found (>1); if no path was found return NULL
size_t find_path(Position* objects, size_t o_size, Position target, Position source, size_t l, size_t b);

#endif //CSCI251_PROJECT3_PATHFINDING_H
