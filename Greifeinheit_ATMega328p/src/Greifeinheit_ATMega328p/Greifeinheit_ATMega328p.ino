#include <Wire.h>
#include <MobaTools.h>
#include <Bounce2.h>

const int stepRev1 = 400;
MoToStepper myStepper1(stepRev1, STEPDIR);
const int stepRev2 = 800;
MoToStepper myStepper2(stepRev2, STEPDIR);

bool stopGreifer, stopHoehe = 0;
bool interneSchaltabsicherung;

long swEndHoeheOben = 1000, swEndHoeheUnten = 0, swEndGreiferAuf = 0, swEndGreiferZu = 1000;

bool referenziert, refHoehe, refGreifer = false;
// adjust stepper pins to your needs
const byte dirPin1 = 11;
const byte stepPin1 = 10;

const byte enablePin1 = 26;

const byte dirPin2 = 13;
const byte stepPin2 = 12;

const byte enablePin2 = 36;
const byte endGreifer = 9;
const byte endHoehe = 8;


// Ultraschallsensor
int TRIGGER_PIN = 6;  // Trigger für den Sensor
int ECHO_PIN = 7;     // ECHO für den Arduino
long dauer = 0;       // Dauer = Variable, Zeit wird gespeichert, die eine Schallwelle bis zur Reflektion und zurueck benoetigt
long entfernung = 0;  // gemessene Entfernung

void setup() {
  Wire.begin(20);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent);  // register event: Wenn Daten erhalten werden, wird die Funktion "receiveEvent" aufgerufen
  Serial.begin(9600);            // start serial for output
  Serial.println("slave setup");

  //Endlagen
  pinMode(endGreifer, INPUT_PULLUP);
  pinMode(endHoehe, INPUT_PULLUP);

  //Motortreiber
  myStepper1.attach(stepPin1, dirPin1);
  myStepper1.setSpeed(3000);    //Geschwindigkeit in U/min...passt aber nicht zu der tatsächlichen Geschwindigkeit..vermutlich vom Arduino begrenzt
  myStepper1.setRampLen(250);  // Rampenlänge 100 Steps bei 20U/min

  myStepper2.attach(stepPin2, dirPin2);
  myStepper2.setSpeed(2000);
  myStepper2.setRampLen(50);  // Rampenlänge 100 Steps bei 20U/min

  //Ultraschallsensor
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

int cmd, ref = 0;

// Funktion zum Auslesen des Ultraschallsensors -----------------------------------
void ultra_sonic() {
  Serial.println("Fkt ultra_sonic aufgerufen");
  digitalWrite(TRIGGER_PIN, LOW);      //kurze Zeit Spannung weg vom Trigger-Pin. Damit später beim senden des Trigger-Signals ein rauschfreies Signal hat.
  delay(5);                            //Dauer: 5 Millisekunden
  digitalWrite(TRIGGER_PIN, HIGH);     //Jetzt sendet man eine Ultraschallwelle los.
  delay(10);                           //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(TRIGGER_PIN, LOW);      //Dann wird der „Ton“ abgeschaltet.
  dauer = pulseIn(ECHO_PIN, HIGH);     //Mikrokontroller zählt Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  entfernung = (dauer / 2) * 0.03432;  //Entfernung berechnen in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.

  if (entfernung >= 500 || entfernung <= 0) {
    Serial.println("Kein Messwert");  //Wenn Entfernung über 500cm oder unter 0cm liegt weil falsch oder ungenau
  } else {
    Serial.print(entfernung);  //Falls nicht: Entfernung in serial monitor ausgegeben
    Serial.println(" cm");     // Hinter Entfernung im Serial Monitor Einheit "cm"
  }
  delay(200);
}
//--------------------------------------------------------------------------------------------------------

//Die Funktion wird aufgerufen, wenn vom ESP32 eine Anfrage kommt.
void transmit_distance() {
  Serial.println("Sendeanfrage erhalten...");
  Serial.println("Übermittle Entfernung...");
  Wire.write(entfernung);
}
//--------------------------------------------------------------------------------------------------------
void loop() {
  Serial.println(referenziert);
  Serial.println(cmd);
  // Referenzfahrt? ---------------------------
  if (refHoehe and refGreifer) {
    referenziert = true;
  } else {
    referenziert = false;
  }


  /* Die Endlagen werden aktuell nicht benutzt. Es gab Probleme beim Schaltverhalten. -------
  // Endlagenabfrage Greifer ------------------
   if(digitalRead(endGreifer)){
    stopGreifer = true;
    refGreifer = true;
    myStepper2.setZero();
  }
// Endlagenabfrage Höhe ----------------------
  if(digitalRead(endHoehe)){
    stopHoehe=true;
    refHoehe = true;
    myStepper1.setZero();
  }
*/


// Fahrbefehle für die Greifeinheit--------------------------------------------------
 //cmd enthält die gesendeten Daten vom ESP32, die von der Bedieneinheit kommen. 
 switch(cmd){
    case 0:
        myStepper1.rotate(0); //Motor Greifer bremsen
        myStepper2.rotate(0); //Motor Lineareinheit bremsen
        break;
    case 2:       
          myStepper1.rotate(1); //Greifer Auf
          myStepper2.rotate(0);               
          //stopHoehe = false;    // Variable von der Endlage wird zurückgesetzz
          break;
    case 1:
        if(!stopHoehe){
          myStepper1.rotate(-1);  //Greifer Zu
          myStepper2.rotate(0);
        } else if(stopHoehe){
          myStepper1.rotate(0);
        }
        break;
    case 3:     
        myStepper1.rotate(0);
        myStepper2.rotate(1);   //Lineareinheit hochfahren
        //stopGreifer = false;
        break;
    case 4:
        if(!stopGreifer){
            myStepper1.rotate(0);
            myStepper2.rotate(-1);  //Lineareinheit runterfahren
        }else if(stopGreifer){
          myStepper2.rotate(0);
        }
        break;
    }
  //Einlesen des US-Sensors ------------------------------------------------------
  //Serial.println("Lese Ultraschallsensor ein");
  //ultra_sonic();
  // -----------------------------------------------------------------------------

  //Wenn der ESP32 eine Anfrage sendet, wird die Funktion "transmit_distance" aufgerufen ------
  //Wire.onRequest(transmit_distance);
  //----------------------------------------------------------------------------------------------

}
// function that executes whenever data is received from master -------------------------------
// this function is registered as an event, see setup()
//"int howMany" unklar, in der Arduino-Doku steht, dass die Funktion,
// die über Wire.onReceive aufgerufen wird einen int Parameter übergeben bekommen muss
void receiveEvent(int howMany) {
  while (Wire.available())  // loop through all but the last
  {
    int x = Wire.read();  // receive byte as a character
    Serial.print(x);         // print the character
    cmd = x;
  }
}
// ----------------------------------------------------------------------------------------------