#include <Wire.h>
#include <SpeedyStepper.h>
#include <Bounce2.h>
#include "MotorByte.h"
#include "CtrlByte.h"
#include "HX711.h"
#include "PressureSensor.h"

#define debug 0

/* Steppers */
SpeedyStepper stepperGripper;
SpeedyStepper stepperHeight;


/* movement Param */
// Gripper
const float millisPerTurnGripper = 8; // 8 mm gripper base movment per turn
const float stepsPerRevolutionGripper = 1600; // motor has 200 steps/rev, controller ist set to 1600 microsteps/rev
const float speedMillisPerSecondGripper = 8; // on turn of the Motor is 3 mm in height 18 mm/s -> 6 1/s
const float accMillisPerSecondPerSecondGripper = 60;
const float softLimitLowGripper = 0;
const float softLimitHighGripper = -13.5;

// Height
const float millisPerTurnHeight = 3; // 3 mm height per turn
const float stepsPerRevolutionHeight = 1600; // motor has 200 steps/rev, controller ist set to 1600 microsteps/rev
const float speedMillisPerSecondHeight = 18; // on turn of the Motor is 3 mm in height 18 mm/s -> 6 1/s
const float accMillisPerSecondPerSecondHeight = 30;

bool processMovementGripper = true; // should the gripper move? 

bool stopGreifer, stopHoehe = 0;
bool interneSchaltabsicherung;

long swEndHoeheOben = 1000, swEndHoeheUnten = 0, swEndGreiferAuf = 0, swEndGreiferZu = 1000;

bool referenziert, refHoehe, refGreifer = false;

// adjust stepper pins to your needs
const byte dirPinGripper = 11;
const byte dirPinHeight = 13;

const byte stepPinGripper = 10;
const byte stepPinHeight = 12;

const byte enablePinGripper = 26;
const byte enablePinHeight = 36;

const byte limitSwitchGripper = 9;
const byte limitSwitchHeight = 8;

const byte sckPinPressureSensor = 2;
const byte doutPinPressureSensor = 3;


CtrlByte cmd, lastCmd;
int ref = 0;

/* Pressuresensor */
// PressureSensor pressureSensor(doutPinPressureSensor, sckPinPressureSensor);
// HX711 scale;
float measuredVal;

// function that executes whenever data is received from master -------------------------------
// this function is registered as an event, see setup()
//"int howMany" unklar, in der Arduino-Doku steht, dass die Funktion,
// die über Wire.onReceive aufgerufen wird einen int Parameter übergeben bekommen muss
void receiveEvent(int howMany) {
  while (Wire.available())  // loop through all but the last
  {
    int x = Wire.read();  // receive byte as a character
    Serial.print(x);         // print the character
    cmd.setByte(x);
    Serial.print(" ");
    Serial.println(cmd.getByte());
  }
}

// use this mehod instead of the homing procedure of speedy stepper, its shit
// this is just the movement of the homing procedure and setting the postion
// changes on the speed have to been done before uisng the method
#include <SpeedyStepper.h>

// Homing Methode
void homeStepper(SpeedyStepper &motor, int limitSwitchPin,
               float approachSpeed = 1.0,   // Speed for moving towards switch
               float backoffSpeed  = 0.1,   // Speed vor backing of the switch
               int approachDistance = 1000, // moving distance to switch in mm
               int backoffDistance  = -5) {   // movin distance from switch in mm
                
  // 1. moving to the switch
  motor.setSpeedInMillimetersPerSecond(approachSpeed);
  motor.setupRelativeMoveInMillimeters(approachDistance);

  while (!(motor.processMovement()))
  {
      #if (debug == 3)
          Serial.println("Moving towards Limitswitch");
      #endif

      delay(1);

      // when the switch is pressed
      if (digitalRead(limitSwitchPin) == HIGH)
          break;
  }

  delay(50);

  // 2. backing of slowly till the switch is not pressed anymore
  motor.setSpeedInMillimetersPerSecond(backoffSpeed);
  motor.setupRelativeMoveInMillimeters(backoffDistance);

  while (!(motor.processMovement()))
  {
      #if (debug == 3)
          Serial.println("Moving away from Limitswitch");
      #endif

      delay(1);
      if (digitalRead(limitSwitchPin) == LOW)
          break;
  }

  delay(50);

  // 3.  set Home‑Position 
  Serial.println("Stepper homed");
  motor.setCurrentPositionInMillimeters(0);
  motor.setupRelativeMoveInMillimeters(motor.getCurrentPositionInMillimeters());
}

void setup() {
  Wire.begin(20);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent);  // register event: Wenn Daten erhalten werden, wird die Funktion "receiveEvent" aufgerufen
  Serial.begin(115200);            // start serial for output
  Serial.println("slave setup");


  /* Pin Configuration */
  // config of direction pin outputs
  pinMode(dirPinGripper, OUTPUT);
  pinMode(dirPinHeight, OUTPUT);

  // config of step pin outputs
  pinMode(stepPinGripper, OUTPUT);
  pinMode(stepPinHeight, OUTPUT);

  // config of enable pin outputs
  pinMode(enablePinGripper, OUTPUT);
  pinMode(enablePinHeight, OUTPUT);

  // config of limitswitch Inputs
  pinMode(limitSwitchGripper, INPUT_PULLUP);
  pinMode(limitSwitchHeight, INPUT_PULLUP);

  /* stepper configuration for homing */
  // gripper
  stepperGripper.connectToPins(stepPinGripper, dirPinGripper);
  stepperGripper.setStepsPerMillimeter(stepsPerRevolutionGripper / millisPerTurnGripper); // calc the step count for one mm of movement of the linear unit

  // linear unit
  stepperHeight.connectToPins(stepPinHeight, dirPinHeight);
  stepperHeight.setStepsPerMillimeter(stepsPerRevolutionHeight / millisPerTurnHeight); // calc the step count for one mm of movement of the linear unit

  /* homing */
  homeStepper(stepperGripper, limitSwitchGripper, 5, 0.1, 30, -5);
  homeStepper(stepperHeight, limitSwitchHeight, 20, 0.1, 1000, -5);

  /* stepper configuration for loop */
  Serial.println("setup for loop");

  // gripper
  stepperGripper.setSpeedInMillimetersPerSecond(speedMillisPerSecondGripper); // set speed
  stepperGripper.setAccelerationInMillimetersPerSecondPerSecond(accMillisPerSecondPerSecondGripper); // set acceleration

  // linear unit
  stepperHeight.setSpeedInMillimetersPerSecond(speedMillisPerSecondHeight); // set speed
  stepperHeight.setAccelerationInMillimetersPerSecondPerSecond(accMillisPerSecondPerSecondHeight); // set acceleration

  /* Pressure Sensor */
  // pressureSensor.SetupSensor();
  /*scale.begin(doutPinPressureSensor, sckPinPressureSensor);
  scale.set_scale();
  scale.tare(5);
  scale.set_scale(11200);*/
  processMovementGripper = true;

  Serial.println("Setup done");
}

// Funktion wir in jedem Fall der Caseanweisung aufgerufen um zu überprüfen ob der Fahrbefehl schon getriggert
// -> Überprüfung ist notwendig weil :
//    1. setupStop() bei jedem aufruf eine Bremsung durchführt -> verschiebung der Zielpos auf ende der Bramsrampe von aktuellem Punkt
//    2. setupRelativMoveIn...() bei jedem Aufruf eine neue Zielposition festlegt
bool shouldIDoThisCmd(CtrlByte cmdNo) {
  if (cmdNo.getByte() != lastCmd.getByte()) { // wenn  die letzte und aktuelle cmdNo nicht gleich sind
    lastCmd.setByte(cmdNo.getByte()); // gleich setzten
    return true; // true zurückgeben
  }

  return false; // sonst false zurückgeben
}

// -> Zusätzliche Überprüfung ist notwendig weil :
//    1. ein aufruf von setupStop eine Bremsfahrt versursacht, auch wenn der Motor bereits steht -> keine Bremsung wenn der Motor steht
void breakIfNeeded(SpeedyStepper *stepperIn) {
  if ((*stepperIn).getCurrentVelocityInStepsPerSecond() != 0.0) {
    (*stepperIn).setupStop(); 
  }  
}

void loop() {
  #if (debug == 2)
    if (digitalRead(limitSwitchGripper) == HIGH) {
      Serial.println("HIGH");
    } else {
      Serial.println("LOW");
    }
  #endif
  // measuredVal = scale.get_units(1);
  // Serial.print(measuredVal <= 5);
  // Serial.print(" ");
  // Serial.println(measuredVal);
  

  // movement control
  if(shouldIDoThisCmd(cmd)){ // checking if the command is new
    #if (debug == 1) 
      Serial.print("cmdNo: ");
      Serial.print(cmd.readBit(moveUp));
      Serial.print(cmd.readBit(moveDown));
      Serial.print(cmd.readBit(openGripper));
     Serial.println(cmd.readBit(closeGripper));
    #endif
  
    /* height */
    if (cmd.readBit(moveUp)) { 
      stepperHeight.setupRelativeMoveInMillimeters(1000); // change setpos to 1000
    } else if (cmd.readBit(moveDown)) {
      stepperHeight.setupRelativeMoveInMillimeters(-1000); // change setpos to -1000
    } else { // no signal for height changement
      breakIfNeeded(&stepperHeight);
    }

    /* gripper */
    if (cmd.readBit(closeGripper)) { 
      stepperGripper.setupMoveInMillimeters(softLimitLowGripper);
      processMovementGripper = true; // enable movement
    } else if (cmd.readBit(openGripper)) {
      stepperGripper.setupMoveInMillimeters(softLimitHighGripper); //
      // processMovementGripper = pressureSensor.MovementIsSafe(); 
    } else { // no signal for gripper changement
      breakIfNeeded(&stepperGripper);
    }
  }

  /* do the given Movements */
  stepperHeight.processMovement();
  processMovementGripper = true; // tmp
  // if (processMovementGripper) {
    stepperGripper.processMovement();
  // }
}