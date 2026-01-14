#ifndef _MOVEMENT_DIRECTIONS
#define _MOVEMENT_DIRECTIONS

// Movementdirections: foward, backward, left, right, foward and left, foward and right, turning left, turning right
// foward and tunrning left, foward and turning right, backward and turning left, backward and turning right
enum class movementDirections {
    stop,  // 0 no movement
    fw,    // 1 foward
    bw,    // 2 backward
    l,     // 3 left
    r,     // 4 right
    fwl,   // 5 foward and left
    fwr,   // 6 foward and right
    bwl,   // 7 backward and left
    bwr,   // 8 backward and right
    tl,    // 9 turning left
    tr,    // 10 turning right
    fwtl,  // 11 foward and tunrning left
    fwtr,  // 12 foward and turning right
    bwtl,  // 13 backward and turning left
    bwtr,   // 14 backward and turning right
    lugp,   // 15 linearunit grippingPos 
    lutp,   // 16 linearunit transportPos
    grop,   // 17 open gripper
    grcl,   // 18 close gripper
    ledon,  // 19 LED ON
    ledoff  // 20 LED off
};

const char* movementDirectionNames[] = {
    "Stop",                  // stop
    "Forward",               // fw
    "Backward",              // bw
    "Left",                  // l
    "Right",                 // r
    "Forward + Left",        // fwl
    "Forward + Right",       // fwr
    "Backward + Left",       // bwl
    "Backward + Right",      // bwr
    "Turn Left",             // tl
    "Turn Right",            // tr
    "Forward + Turn Left",   // fwtl
    "Forward + Turn Right",  // fwtr
    "Backward + Turn Left",  // bwtl
    "Backward + Turn Right",  // bwtr
    "linearunit grippingPos",
    "linearunit transportPos",
    "open gripper",
    "close gripper",
    "led on",
    "led off"
};

#endif