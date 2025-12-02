#ifndef _BUTTONS
#define _BUTTONS

class Buttons
{
private:
    bool* buttonAdr;

    bool valLastCycle;
    bool valThisCycle; 

    bool impulseOut;

public:
    Buttons(bool* buttonIn);

    void update();

    bool getImpulse();
};
#endif


