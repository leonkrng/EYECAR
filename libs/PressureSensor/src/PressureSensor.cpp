/* ----------------------------------*/
/* History */

/* V_00: 08.11.2025 */
/* First Build, determined scalingfactor in prior testing*/

/* ----------------------------------*/

#include "PressureSensor.h"

PressureSensor::PressureSensor (uint8_t iDOUT_PIN, uint8_t iSCK_PIN) {
    iDoutPin = iDOUT_PIN; // read Pins
    iSckPin = iSCK_PIN;
}

void PressureSensor::SetupSensor() { // scalingfactor is known
    Serial.begin(9600); // start serial communication for possible debugging
    hx711.begin(iDoutPin, iSckPin); // start platine

    while (!hx711.is_ready()) { // while the HX711 is not ready yet
        Serial.println("HX711 is not ready yet...");
        delay(100); // wait 100 milliiseconds and try again
   }
   
   // 1. setup step
   // reset scaling
   // just to be safe - should be 1 after restart
   Serial.println("1. Step - reset");
   hx711.set_scale(); // when called without inputval scale is set to 1
   Serial.print("scaling after reset - should be 1: ");
   Serial.println(hx711.get_scale());

   // 2. setup step
   // tare
   // there must be no additional pressure on the sensor
   Serial.println("2. Step - tare");
   hx711.tare(iSetupNum); // tare
   Serial.print("measured value after tare - should be between -1000 an 1000 (value with scaling 1): ");
   Serial.println(hx711.get_units(10));

   // 3. setup step
   // set scalefactor = -112
   // pre determined 
   Serial.println("3. Step - set scaling");
   hx711.set_scale(fScalingFactor); // set scalingfactor
   Serial.print("scalingfactor after setting - must be equal to fScalingFactorGramms: ");
   Serial.println(hx711.get_scale());


   // setup done - alles tutti, jetzt geht's ab!
   Serial.println("setup done :)");
}

bool PressureSensor::MovementIsSafe() {
    float fTmpWeight = hx711.get_units(iMeasurementNum); // get measurement

    if (hx711.get_scale() == 1.0 // safety - no call of setup?
        or hx711.get_offset() == 0.0) {
            return false;
        }

    if (fTmpWeight <= fMaximumPushingForce) { // when measured value is lower or equal to treshold
        return true; // movement is safe
    } else{ // when its bigger
        return false; // HALT STOP!!!1!
    }
}