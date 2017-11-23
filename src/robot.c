#include <malloc.h>
#include "robot.h"

/// initialize a robot
Robot makeRobot(size_t ID, Position pos, bool malicious) {
    Robot rob = safemalloc(sizeof *rob);
    rob->ID        = ID;
    rob->self      = pos;
    rob->target    = NULL;
    rob->malicious = malicious;
    return rob;
}
