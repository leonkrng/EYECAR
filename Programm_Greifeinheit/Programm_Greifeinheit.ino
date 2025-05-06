#include <Wire.h>
#include <MobaTools.h>
#include <Bounce2.h>

const int stepRev1 = 1600;    
MoToStepper myStepper1( stepRev1, STEPDIR );
const int stepRev2 = 1600;
MoToStepper myStepper2( stepRev2, STEPDIR );

bool stopGreifer, stopHoehe;
bool interneSchaltabsicherung;

long swEndHoeheOben = 1000, swEndHoeheUnten=0, swEndGreiferAuf=0, swEndGreiferZu=1000;

bool referenziert, refHoehe, refGreifer = false;
// adjust stepper pins to your needs
const byte dirPin1 = 11;
const byte stepPin1 = 10;

const byte enablePin1 = 26;

const byte dirPin2 = 12;
const byte stepPin2 = 13;

const byte enablePin2 = 36;
const byte endGreifer = 9;
const byte endHoehe = 8;


void setup()
{
  Wire.begin(20);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
  Serial.println("slave setup");
  
  pinMode(endGreifer, INPUT_PULLUP);
  pinMode(endHoehe, INPUT_PULLUP);

  
  myStepper1.attach( stepPin1, dirPin1 );
  myStepper1.setSpeed( 10000 );
  myStepper1.setRampLen( 50 );                       // Rampenlänge 100 Steps bei 20U/min

  myStepper2.attach( stepPin2, dirPin2 );
  myStepper2.setSpeed( 1600 );
  myStepper2.setRampLen( 50 );                       // Rampenlänge 100 Steps bei 20U/min
}

int cmd, ref = 0;

void loop()
{
  Serial.println(referenziert);
  Serial.println(cmd);
  if(refHoehe and refGreifer){
    referenziert=true;
  }else{referenziert=false;}
  
  if(digitalRead(endGreifer)){
    stopGreifer = true;
    refGreifer = true;
    myStepper2.setZero();
  }

  if(digitalRead(endHoehe)){
    stopHoehe=true;
    refHoehe = true;
    myStepper1.setZero();
  }

//if(!referenziert){
//  switch(ref){
//    case 0:
//    if(!digitalRead(endHoehe)){
//      myStepper1.rotate(-1);
//    } else if(digitalRead(endHoehe)){
//        myStepper1.rotate(0);
//        myStepper1.setZero();
//        ref=1;
//    }
//    break;
//    case 1:
//    if(!(myStepper1.readSteps()<1000)){
//      myStepper1.rotate(1);
//    } else if(myStepper1.readSteps()>1000){
//        myStepper1.rotate(0);
//        refHoehe=true;
//        ref=2;
//      }
//     break;
//    case 2:
//      if(!digitalRead(endGreifer)){
//      myStepper2.rotate(-1);
//      } else if(digitalRead(endGreifer)){
//        myStepper2.rotate(0);
//        myStepper2.setZero();
//        ref=3;
//      }
//    break;
//    case 3:
//    if(!(myStepper2.readSteps()<1000)){
//      myStepper2.rotate(1);
//    } else if(myStepper2.readSteps()>1000){
//        myStepper2.rotate(0);
//        refHoehe=true;
//        ref=4;
//      }
//
//    case 4:
//      if(refHoehe and refGreifer){
//        referenziert=true;
//        ref=99;
//      }
//    break;
//    case 99:
//      break;    
//  }
//  switch(cmd){
//    case 0:
//        myStepper1.rotate(0);
//        myStepper2.rotate(0);
//        break;
//    case 2:
//        if(!stopHoehe){
//          myStepper1.rotate(-1);
//          myStepper2.rotate(0);
//        } else if(stopHoehe){
//          myStepper1.rotate(0);
//        }
//        break;
//    case 4:
//        if(!stopGreifer){
//            myStepper1.rotate(0);
//            myStepper2.rotate(-1);
//        }else if(stopGreifer){
//          myStepper2.rotate(0);
//        }
//        break;
//    }
//    
//} else if(referenziert){ 
    switch(cmd){
    case 0:
        myStepper1.rotate(0);
        myStepper2.rotate(0);
        break;
    case 2:
        //if(!(myStepper1.readSteps()<swEndHoeheOben)){
          myStepper1.rotate(1);
          myStepper2.rotate(0);
        //} else if(myStepper1.readSteps()>=swEndHoeheOben){
        //}
          stopHoehe = false;
          break;
    case 1:
        if(!stopHoehe){
          myStepper1.rotate(-1);
          myStepper2.rotate(0);
        } else if(stopHoehe){
          myStepper1.rotate(0);
        }
        break;
    case 3:
      //if(!(myStepper2.readSteps()<swEndGreiferZu)){
        myStepper1.rotate(0);
        myStepper2.rotate(1);
      //} else if(myStepper2.readSteps()>=swEndGreiferZu){
      //}
        stopGreifer = false;
        break;
    case 4:
        if(!stopGreifer){
            myStepper1.rotate(0);
            myStepper2.rotate(-1);
        }else if(stopGreifer){
          myStepper2.rotate(0);
        }
        break;
    }
  //}
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while(Wire.available()) // loop through all but the last
  {
    int x = Wire.read(); // receive byte as a character
    Serial.print(x);         // print the character
    cmd=x;
  }
}
