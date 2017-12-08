/**
 * date: 2017-12-08
 *
 * contributor(s):
 *      Nate Mathews, njm3308@rit.edu
 *      Patrick Ly, pxl7219@g.rit.edu
 **/

#include <glob.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include "simulation.h"

#define PRINT_USAGE(prog) fprintf(stderr, "Usage: %s [-l -b -k -e -s]\n%s%s%s%s", prog, \
                "  -l\theight of the simulation grid (default 10)\n", \
                "  -b\twidth of the simulation grid (default 10)\n" \
                "  -k\ttotal number of robots (default 4)\n", \
                "  -e\tnumber of malicious robots (default 0)\n", \
                "  -s\tseed value (default 1)\n")

int main(int argc, char* argv[])
{
    /* initialize defaults for l,b,k,e
       l = height of simulation grid
       b = width of simulation grid
       k = total number of robots
       e = number of robots that are evil
       s = seed value for PRNG */
    size_t l=10, b=10, k=4, e=0; long s=1;

    // do argument parsing
    int opt;
    while ((opt = getopt(argc, argv, "l:b:k:e:s:")) != -1) {
        switch(opt) {
            case 'l': l = (size_t) strtol(optarg, NULL, 10); break;
            case 'b': b = (size_t) strtol(optarg, NULL, 10); break;
            case 'k': k = (size_t) strtol(optarg, NULL, 10); break;
            case 'e': e = (size_t) strtol(optarg, NULL, 10); break;
            case 's': s = strtol(optarg, NULL, 10); break;
            default:
                PRINT_USAGE(argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    // run the simulation and return it's exit code
    return run(l, b, k, e, s);
}
