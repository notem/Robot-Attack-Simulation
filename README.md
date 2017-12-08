## CSCI 252 COPADS Project 3 Readme

This is our team's implementation of the Robot Attack simulation which meets the specifications of the writeup.
The project consists of several files:
* ``main.c``           - parses command arguments
* ``simulation.c|.h``  - constructs robots and runs the simulation loop
* ``robot.c|.h``       - defines the robot data structure and robot-related functions
* ``pathfinding.c|.h`` - breadth-first search implementation utilizing the Position struct defined in ``robot.h``
* ``utils\display.c|.h`` - functions for displaying the simulation grid in the terminal

### Setup
1. Use ``cmake CMakeLists.txt`` to generate the Makefile.
2. Use ``make`` to compile the program to ``main``
3. Test run the program without arguments

### Arguments

All arguments are optional. 
The program uses the GNU flag style of argument parsing.
All possible arguments are initialized with default values. 
The possible flags and default values are below:

* -l : (default 10) height of the simulation grid
* -b : (default 10) width of the simulation grid
* -k : (default 4) total number of robots
* -e : (default 0) number of malicious robots,
                   must be less than (3*k+1)
* -s : (default 1) PRNG initial seed value

### Examples

* ./main
* ./main -s 9999
* ./main -b 20 -l 10
* ./main -b 30 -l 15 -k 6
* ./main -b 40 -l 20 -k 6 -e 1
