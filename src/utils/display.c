#include "../robot.h"

#include <stdio.h>
#include "display.h"

#define TARGET_CHAR 'T'
#define ROBOT_CHAR 'R'
#define MALICIOUS_CHAR 'U'

void clear() {
    printf("\033[2J");
    fflush(stdout);
}

void put(char character) {
    putchar(character);
    fflush(stdout);
}

void set_cur_pos(int rCursor, int cCursor) {
    printf("\033[%d;%dH", rCursor, cCursor);
}

///
void update_display(size_t l, size_t b, size_t k, int phase, int round, Robot *robots, Position target) {
    clear();

    /* put target */
    set_cur_pos((target->x)+2, (target->y)+2);
    put(TARGET_CHAR);

    /* put robots */
    for(size_t j=0; j<k; j++) {
        struct pos *pos = robots[j]->self;
        set_cur_pos((pos->x)+2, (pos->y)+2);
        if(robots[j]->malicious) put(MALICIOUS_CHAR);
        else put(ROBOT_CHAR);
    }

    /* make border */
    for(size_t j=0; j<=l+1; j++) {
        set_cur_pos(0,j);
        put('-');
        set_cur_pos(b+2,j);
        put('-');
    }
    for(size_t j=1; j<=b+1; j++) {
        set_cur_pos(j,0);
        put('|');
        set_cur_pos(j,l+2);
        put('|');
    }
    set_cur_pos(0,0);
    put('+');
    set_cur_pos(0,l+2);
    put('+');
    set_cur_pos(b+2,0);
    put('+');
    set_cur_pos(b+2,l+2);
    put('+');

    /* write out phase */
    set_cur_pos(b+3,0);
    switch(phase) {
        case 0:
            printf("Exploration phase, round %d:\n", round);
            break;
        case 1:
            printf("Agreement phase, round %d:\n", round);
            break;
        case 2:
            printf("Attack phase, round %d:\n", round);
            break;
    }
    set_cur_pos(b+4,0);
}