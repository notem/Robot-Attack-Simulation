#include <glob.h>

#ifndef CSCI251_PROJECT3_SIMULATION_H
#define CSCI251_PROJECT3_SIMULATION_H

/// runs the robot-attack simulation on a grid of size {l}x{b} with {k} robots and {e} malicious robots
/// @param l length dimension of the simulation grid
/// @param b width dimension of the simulation grid
/// @param k number of robots to use in the simulation
/// @param e number of robots which are malicious/compromised
/// @returns simulation exit code
int run(size_t l, size_t b, size_t k, size_t e);

#endif //CSCI251_PROJECT3_SIMULATION_H
