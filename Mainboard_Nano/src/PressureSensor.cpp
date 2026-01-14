/* ----------------------------------*/
/* History */

/* V_00: 08.11.2025 */
/* First Build, determined scalingfactor in prior testing*/

/* ----------------------------------*/

#include "PressureSensor.h"

#define debug 1

PressureSensor::PressureSensor (uint8_t iDOUT_PIN, uint8_t iSCK_PIN) {
    iDoutPin = iDOUT_PIN; // read Pins
    iSckPin = iSCK_PIN;
}

void PressureSensor::SetupSensor() { // scalingfactor is known
    #if (debug == 1)
        Serial.begin(115200); // start serial communication for possible debugging
    #endif
    hx711.begin(iDoutPin, iSckPin); // start platine

    #if (debug == 1)
        Serial.println("1. Step - reset");
    #endif
    hx711.set_scale(); // when called without inputval scale is set to 1
    #if (debug == 1)
        Serial.print("scaling after reset - should be 1: ");
        Serial.println(hx711.get_scale());
   #endif

   // 2. setup step
   // tare
   // there must be no additional pressure on the sensor
    #if (debug == 1)
        Serial.println("2. Step - tare");
    #endif
    hx711.tare(iSetupNum); // tare
    #if (debug == 1)
        Serial.print("measured value after tare - should be between -1000 an 1000 (value with scaling 1): ");
        Serial.println(hx711.get_units(10));
   #endif

   // 3. setup step
   // set scalefactor = -112
   // pre determined 
   #if (debug == 1)
        Serial.println("3. Step - set scaling");
    #endif
    hx711.set_scale(fScalingFactor); // set scalingfactor
    #if (debug == 1)
        Serial.print("scalingfactor after setting - must be equal to fScalingFactorGramms: ");
        Serial.println(hx711.get_scale());
    #endif


   // setup done - alles tutti, jetzt geht's ab!
   setupDone = true;
    #if (debug == 1)
        Serial.println("setup done :)");
    #endif
}

bool PressureSensor::MovementIsSafe() {
    float fTmpWeight = hx711.get_units(iMeasurementNum); // get measurement

    if (setupDone == false) { // safety - no call of setup?
            return false;
        }

    if (fTmpWeight <= fMaximumPushingForce) { // when measured value is lower or equal to treshold
        return true; // movement is safe
    } else{ // when its bigger
        return false; // HALT STOP!!!1!
    }
}

float PressureSensor::GetForce() {
    return setupDone ? hx711.get_units(iMeasurementNum) : 0;
}