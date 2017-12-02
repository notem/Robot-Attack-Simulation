//
// Created by patri on 12/2/2017.
//

#include <stdio.h>
#include <stdlib.h>
#include "display.h"

void clear() {
    printf( "\033[2J" );
    fflush( stdout );
}

void put( char character ) {
    putchar( character );
    fflush( stdout );
}

void set_cur_pos( int rCursor, int cCursor) {
    printf( "\033[%d;%dH", rCursor, cCursor );
}
