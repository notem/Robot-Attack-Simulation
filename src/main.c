/**
 * date: 2017-11-23
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
#include "display.h"

#define USAGE "Usage: %s [-lbke]\n"

int main(int argc, char* argv[])
{
    /* initialize defaults for l,b,k,e */
    size_t l=10, b=10, k=4, e=0;

    // do argument parsing
    int opt;
    while ((opt = getopt(argc, argv, "l:b:k:e:")) != -1) {
        switch(opt) {
            case 'l': l = (size_t) strtol(optarg, NULL, 10); break;
            case 'b': b = (size_t) strtol(optarg, NULL, 10); break;
            case 'k': k = (size_t) strtol(optarg, NULL, 10); break;
            case 'e': e = (size_t) strtol(optarg, NULL, 10); break;
            default:
                fprintf(stderr, USAGE, argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    printf("Clear didn't work :(");
    clear();

    // run the simulation and return it's exit code
    return run(l, b, k, e);
}
