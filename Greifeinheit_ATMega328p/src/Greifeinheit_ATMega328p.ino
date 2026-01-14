#include <Wire.h>
#include <SpeedyStepper.h>
//#include <Bounce2.h>
#include "MotorByte.h"
#include "CtrlByte.h"

#define debug 6

/***********************************************************/

/* set the two positions here */
/* measure the distance from the limit switch to the wanted gripping pos */
/* the const must be negativ because off movement direction */
/* distance = 200 -> grippingPos = -200 */

const float grippingPos = -150;
const float transportPos = grippingPos - 20; // position while transporting is a little higher

/***********************************************************/

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
const float softLimitHighGripper = -18;

// Height
const float millisPerTurnHeight = 3; // 3 mm height per turn
const float stepsPerRevolutionHeight = 800; // motor has 200 steps/rev, controller ist set to 1600 microsteps/rev
const float speedMillisPerSecondHeight = 25; // on turn of the Motor is 3 mm in height 18 mm/s -> 6 1/s
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



// function that executes whenever data is received from master -------------------------------
// this function is registered as an event, see setup()
//"int howMany" unklar, in der Arduino-Doku steht, dass die Funktion,
// die über Wire.onReceive aufgerufen wird einen int Parameter übergeben bekommen muss
void receiveEvent(int howMany) {
  while (Wire.available())  // loop through all but the last
  {
    int x = Wire.read();  // receive byte as a character
    //Serial.println(x);         // print the character
    cmd.setByte(x);
   /* Serial.print(cmd.readBit(openGripper));
    Serial.print(cmd.readBit(closeGripper));
    Serial.print(cmd.readBit(moveUp));
    Serial.println(cmd.readBit(moveDown));*/
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
  homeStepper(stepperHeight, limitSwitchHeight, speedMillisPerSecondHeight, 0.1, 5000, -5);

  /* stepper configuration for loop */
  Serial.println("setup for loop");

  // gripper
  stepperGripper.setSpeedInMillimetersPerSecond(speedMillisPerSecondGripper); // set speed
  stepperGripper.setAccelerationInMillimetersPerSecondPerSecond(accMillisPerSecondPerSecondGripper); // set acceleration

  // linear unit
  stepperHeight.setSpeedInMillimetersPerSecond(speedMillisPerSecondHeight); // set speed
  stepperHeight.setAccelerationInMillimetersPerSecondPerSecond(accMillisPerSecondPerSecondHeight); // set acceleration

  // move to gripping Pos after referencing
  stepperHeight.setupMoveInMillimeters(grippingPos); // setup movement
  while (!stepperHeight.motionComplete()) { // process movement till its finished
    stepperHeight.processMovement(); // process movement
  }
  

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
  #if (debug == 5)
    Serial.println(cmd.getByte());
  #endif

  // movement control
  if(shouldIDoThisCmd(cmd)){ // checking if the command is new

    #if (debug == 1) 
      Serial.print("cmdNo: ");
      Serial.print(cmd.readBit(moveUp));
      Serial.print(cmd.readBit(moveDown));
      Serial.print(cmd.readBit(openGripper));
     Serial.println(cmd.readBit(closeGripper));
    #endif
    #if (debug == 4)
      Serial.print("new CMD: ");
    #endif
  
    /* height */
    if (cmd.readBit(moveUp) and not (cmd.readBit(moveDown))) { 
      stepperHeight.setupMoveInMillimeters(grippingPos); // setup movement to transportPos
      #if (debug == 4)
        Serial.print("up ");
      #endif
    } else if (cmd.readBit(moveDown) and not (cmd.readBit(moveUp))) {
      stepperHeight.setupMoveInMillimeters(transportPos); // setup movement to grippingPos
      #if (debug == 4)
        Serial.print("down ");
      #endif
    } else { // no signal for height changement
      breakIfNeeded(&stepperHeight);
      #if (debug == 4)
        Serial.print("stop ");
      #endif
    }

    /* gripper */
    if (cmd.readBit(closeGripper) and not (cmd.readBit(openGripper))) { 
      stepperGripper.setupRelativeMoveInMillimeters(-100); // move to are very distant point -> movement is stopped by the pressure sensor
      #if (debug == 4)
        Serial.println("close");
      #endif
    } else if (cmd.readBit(openGripper) and not cmd.readBit(closeGripper)) {
      stepperGripper.setupRelativeMoveInMillimeters(100); // // move to are very distant point -> movement is stopped by the limit switch
      #if (debug == 4)
        Serial.println("open");
      #endif
    } else { // no signal for gripper changement
      breakIfNeeded(&stepperGripper);
      #if (debug == 4)
        Serial.println("stop");
      #endif
    }
  }

  #if (debug == 6)
    Serial.println(cmd.readBit(movementIsSafe));
  #endif
  /* do the given Movements */
  stepperHeight.processMovement();

  if ((cmd.readBit(movementIsSafe) and cmd.readBit(closeGripper)) or (cmd.readBit(openGripper) and digitalRead(limitSwitchGripper)== LOW)) {
    stepperGripper.processMovement();
  }  
}