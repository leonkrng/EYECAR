#ifndef PRESSURESENSOR_H // Safety und so
#define PRESSURESENSOR_H

#include "Arduino.h"
#include "HX711.h" 

class PressureSensor {
public:
  PressureSensor (
      uint8_t iDOUT_PIN, // DataOut Pin of the HX711
      uint8_t iSCK_PIN); // Serial Clock Pin of the HX 711
  
  // retunrs true or false, depending on whether the measured value is lower or higher than maximumPushingWeight
  bool MovementIsSafe();

  // standard version without Determination of scaling
  void SetupSensor();

  // setup version with Determination of scaling -- NO IMPLEMENTATION YET (not needed?)
  //void SetupSensor(float fPredeterminedWeight);

private:
    uint8_t iDoutPin; // needed Pins
    uint8_t iSckPin;

    HX711 hx711; // HX711 platine

    const float fScalingFactorGramms = -112; // pre determined scling factor for a measurement visualisation as gramms
    
    // when the number of function calls increases the call time increases too
    // more calls return a more precise value
    const int iSetupNum = 50; // Number of internal function calls during the setuo routine
    const int iMeasurementNum = 5; // Number of internal function calls during measurement
    
    const float fMaximumPushingWeight = 1000; // treshold value in gramms at which the clamping movement is stopped
};


#endif