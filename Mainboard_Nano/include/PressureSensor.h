/* ----------------------------------*/
/* History */

/* V_00: 08.11.2025 */
/* First Build, determined scalingfactor in prior testing*/

/* V_01: 14.11.2025 */
/* Adjusted scalingfactor so the value is measured in newtons instead of gramms */
/* OLD: -112  NEW: -112 * 0,01 = -1.12*/
/* also adjusted variablenames accordingly */

/* ----------------------------------*/

#ifndef PRESSURESENSOR_H // Safety und so
#define PRESSURESENSOR_H

#include "Arduino.h"
#include "HX711.h" 

class PressureSensor {
public:
  PressureSensor (
      uint8_t iDOUT_PIN, // DataOut Pin of the HX711
      uint8_t iSCK_PIN); // Serial Clock Pin of the HX 711
  
  // returns true or false, depending on whether the measured value is lower or higher than maximumPushingWeight
  bool MovementIsSafe();

  // standard version without Determination of scaling
  void SetupSensor();

  // setup version with Determination of scaling -- NO IMPLEMENTATION YET (not needed?)
  //void SetupSensor(float fPredeterminedWeight);


  // returns the act force value measured
  float GetForce(); 

private:
    uint8_t iDoutPin; // needed Pins
    uint8_t iSckPin;

    HX711 hx711; // HX711 platine

    const float fScalingFactor = 1.12; // pre determined scling factor for a measurement visualisation as newtons [N]
    
    // when the number of function calls increases the call time increases too
    // more calls return a more precise value
    const int iSetupNum = 20; // Number of internal function calls during the setuo routine
    const int iMeasurementNum = 1; // Number of internal function calls during measurement
    
    const float fMaximumPushingForce = 10000; // treshold value in gramms at which the clamping movement is stopped

    bool setupDone;
};


#endif