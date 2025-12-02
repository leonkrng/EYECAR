#include "Buttons.h"

Buttons::Buttons(bool* buttonIn) {
    buttonAdr = buttonIn;
    impulseOut = false;
    valLastCycle = false;
    valThisCycle = false;
}

void Buttons::update() {
    impulseOut = false; // reset Impulse

    valThisCycle = *buttonAdr; // read act val from Button
    
    if (valThisCycle != valLastCycle) { // when the val of the button changed
        impulseOut = true;
    }  
}

bool Buttons::getImpulse() {
    return impulseOut;
}