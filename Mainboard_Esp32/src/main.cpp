#include <Arduino.h>
#include <SPI.h>
#include <PCF8574.h>        //DB
#include <RFM69.h>
#include <Wire.h>
#include "movementDirections.h"
#include "MotorByte.h"
#include "CtrlByte.h"
#include "Buttons.h"
#include <ctype.h>

#define NETWORKID     119   // Must be the same for all nodes (0 to 255)
#define MYNODEID      213   // My node ID (0 to 255)
#define TONODEID      180   // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY     RF69_868MHZ
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRET" // Use the same 16-byte key on all nodes
#define USEACK        false // Request ACKs or not
#define FREQUENCY_EXACT 869500000
//#define IS_RFM69HW
RFM69 radio(25,4);   //SS-Pin, Interrupt-Pin

#define debugLevel 0 // standard debug: 1; debug of movement directions
#define override 1 // when set to a value higher than 0, an override is used, otherwise the sticks are interpreted binary (not implemented)
#define DEBUG_GREIFER 0
#define DEBUG_AUTOMODE 0
#define DEBUG_SPI_RECIEVE 0
#define DEBUG_LED 0
#define DEBUG_CONTROLDATA 0
#define DEBUG_SEND_STATUS 0
#define DEBUG_MOVEMENT_DIRECTION 0
#define DEBUG_TMPDIRECTION 0
#define DEBUG_DATA_RASPI 0
#define DEBUG_RASPI_MOVE 1

//DB Für Greifeinheit
char fromSerialMon = ' ';
CtrlByte sendToMega; // Byte for controling gripper and linear unit
bool GreifeinheitGrippingPos, GreifeinheitTransportPos, GreiferAuf, GreiferZu;

String datenstring;
unsigned long receivedLast;
bool receiverTimeout = true;

bool output[25];
bool parityBerechnet, parityReceived;

bool Notaus_OK;

int controldata[16]; // data from gamepad
bool automodeOnLastCylce; // rembering the last state of the gamepadswicth for starting the automode
bool automodeOffLastCylce; // rembering the last state of the gamepadswicth for stopping the automode
bool automodeActive; // name says it all
bool ledActiveLastCycle; // remebering last state of the led 

int intFromRaspi; // integer Value, convertet from the ASCII which is send from raspi

int raspiData[10];
float driveValue[4];

float anteilMotor[4];
float minStickValRel = 35; // minimal relativ value of the analogsticks where a movement should be initiated

movementDirections movementCalculatedGamepad, movementCalculatedRaspi; // calculated movement from analog values, initialised with stop = 0 
volatile float VL, VR, HL, HR;
float VLneu, VRneu, HLneu, HRneu;
float factorOld, factorNew; 
float stellwert[4], stellwertSoll[4];

unsigned long lastStep[4];

unsigned long currentMillis, lastMillisRadiopacketReceive, lastMillisSPINano, lastMillisRaspi, lastMillis50MS, lastMillisAruco;

int parityCounter;
bool high;
byte SPISEND,SPIRECEIVE;    

int TIMEFACTOR = 50;      //Vorher: 50
int MAXSPEED_MAN = 10;    //Höchstgeschwindigkeit: je kleiner die Zahl desto schneller!
int MAXSPEED_AUTO = 13; 
int UPPERLIMITDELAY = 300;


volatile int count;    // Trigger 
int totalInterrupts;   // counts the number of triggering of the alarm
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint cycles;
int genClock[4]; // counter for comparison in ISR
bool stateOutput[4]; // toggling value used inn ISR to changed Value off stepPins
volatile int interruptGrenzwert[4]; // val to compare against genClock[] in ISR

//________________________________________________________________________________
byte led = 13;

volatile byte DO_Dir1 = 26, DO_Dir2 = 27, DO_Dir3 = 32, DO_Dir4 = 33;
volatile byte DO_Step1 = 12, DO_Step2 = 13, DO_Step3 = 14, DO_Step4 = 15;

byte AI_SpgMess_36V = 34, AI_SpgMess_24V = 35, AI_SpgMess_5V = 36;

byte DO_LED = 2;

//byte DO_PortE1 = 21, DO_PortE = 22;     //DB

byte DO_SS_ArduinoNano = 5;
byte DI_Notaus = 39;
//________________________________________________________________________________

const int belegungOutputsStep[] = {DO_Step1, DO_Step2, DO_Step3, DO_Step4};
byte datenZumNano[9] = {1,2,3,4,5,6,7,8,9};
byte datenVomNano[9] = {0,0,0,0,0,0,0,0,0};

bool flag_new_packet = true;
int dataIndexCounter;

String Nachricht;

int16_t rssi;

float Spannung_36V = 38, Spannung_24V = 24, Spannung_5V = 5;
int akkuProzent;

bool arucoNavigationAktiv, prevArucoNavigationAktiv, beleuchtungAktiv;
bool lastArucoNavigation;
int printctr;
double distanzVorne, distanzLinks, distanzRechts, distanzHinten;

char receivedChars[64]; // an array to store the received data

short int stellwertRecord[100][3];
short int recordPlaybackCounter;

int arucoY, arucoZ;
bool arucoDrehend;

void IRAM_ATTR onTime() { // definition of the ISR -> IRAM_ATTR functions is saved in RAM
  portENTER_CRITICAL_ISR(&timerMux); // safety und so -> no other interrupts during the call of this ISR
  for(int i = 0; i<= 3; i++) {
    genClock[i]++; // increment counter

   if(genClock[i] >= interruptGrenzwert[i] && interruptGrenzwert[i] != 0) { // wenn counter is greater than the calculated limit and the limit is not zero
     stateOutput[i] = !stateOutput[i]; // toggle output bit
     digitalWrite(belegungOutputsStep[i], stateOutput[i]); // write DOUT with the determined value
     genClock[i] = 0; // reset counter
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {  
  Serial.begin(115200);
  Serial.println("----------------"); 
  Serial.println("Startup");
  Serial.println("----------------");  
  delay(20);
  
 
  Serial2.begin(115200);

  pinMode(DO_Dir1, OUTPUT);
  pinMode(DO_Dir2, OUTPUT);
  pinMode(DO_Dir3, OUTPUT);
  pinMode(DO_Dir4, OUTPUT);
  pinMode(DO_Step1, OUTPUT);
  pinMode(DO_Step2, OUTPUT);
  pinMode(DO_Step3, OUTPUT);
  pinMode(DO_Step4, OUTPUT);


  pinMode(DO_LED, OUTPUT);

  pinMode(DO_SS_ArduinoNano, OUTPUT);
  digitalWrite(DO_SS_ArduinoNano, HIGH);

  pinMode(DI_Notaus, INPUT);

  //DB Für Erweiteung Greifeinheit
  Wire.begin();
  
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);




  timer = timerBegin(0, 50, true);                
  timerAttachInterrupt(timer, &onTime, true);    //pointername, Adresse der Funktion, true für Flankentrigger
    
   // Sets an alarm to sound every second
   timerAlarmWrite(timer, 8, true);        //pointername, Triggerfrequenz, autoreload = true  
   timerAlarmEnable(timer);                       //scharf schalten


  Serial.println("[INFO] ready");

}
//DB
void Scanner ()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

 // Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}

void pullData_FUNKVERBINDUNG(String dataInput) {
  //uint8_t maxLength = dataInput.length();
  int searchStartposition = 0;
  int outputIndex  = 0;
  int substringstartposition  = 0;
  int charFoundAt = dataInput.indexOf('/', searchStartposition);

  //Serial.println(dataInput);
  while(charFoundAt >= 0) {
    //Serial.print("found chat at: "); Serial.println(charFoundAt);
    controldata[outputIndex] = dataInput.substring(substringstartposition, charFoundAt).toInt();
    //Serial.print("substring bounds: "); Serial.print(substringstartposition); Serial.print(" "); Serial.println(charFoundAt);
    searchStartposition = charFoundAt + 1;
    //Serial.print("new searchStartposition: "); Serial.println(searchStartposition);
    substringstartposition = searchStartposition;
    outputIndex++;
    charFoundAt = dataInput.indexOf('/', searchStartposition);
  }
  
}

void pullData_RASPI(String dataInput) {
  int searchStartposition = 0;
  int outputIndex  = 0;
  int substringstartposition  = 0;
  int charFoundAt = dataInput.indexOf('/', searchStartposition);

  //Serial.println(dataInput);
  while(charFoundAt >= 0) {
    //Serial.print("found chat at: "); Serial.println(charFoundAt);
    raspiData[outputIndex] = dataInput.substring(substringstartposition, charFoundAt).toInt();
    //Serial.print("substring bounds: "); Serial.print(substringstartposition); Serial.print(" "); Serial.println(charFoundAt);
    searchStartposition = charFoundAt + 1;
    //Serial.print("new searchStartposition: "); Serial.println(searchStartposition);
    substringstartposition = searchStartposition;
    outputIndex++;
    charFoundAt = dataInput.indexOf('/', searchStartposition);
  }

  //Serial.print("    ->"); Serial.print(raspiData[0]);Serial.print(" "); Serial.print(raspiData[1]);Serial.print(" "); Serial.println(raspiData[2]);
}

void arucoAutomatik() {

  if(arucoNavigationAktiv && !prevArucoNavigationAktiv) {
    lastMillisAruco = currentMillis -2001;
    Serial.println("[INFO] Starte Aruco-Navigation");
  }
  prevArucoNavigationAktiv = arucoNavigationAktiv;

  //Serial.print("raspiData2: "); Serial.println(raspiData[2]);

  if(arucoNavigationAktiv) {
    
    
    switch(int(raspiData[2])) {
      case -1: //kein Marker erkannt
        
        if(currentMillis - lastMillisAruco < 4000) {            //Bei kurzem Verlust des Markers nur anhalten
          arucoY = 0;
          arucoZ = 0; 
        } else if(currentMillis - lastMillisAruco > 40000) {     //Bei längerem Verlust des Markers Automatik beenden
         arucoY = 0;
         arucoZ = 0; 
         Serial.println("[INFO] Kein Marker mehr gefunden - stoppe Aruco-Navigation!");
         arucoNavigationAktiv = 0;
        } else {                                                //Bei mittlerer Verlustzeit Auto zum Drehen bringen
         arucoZ = 50; 
        }
        break;

      case 0:
        arucoY = 0;
        arucoZ = 0;
         Serial.println("[INFO] ENDE - stoppe Aruco-Navigation!");
         arucoNavigationAktiv = 0;
         break;

      case 1: case 5: //Marker erreicht
        if(currentMillis - lastMillisAruco < 2000) {            //Bei kurzem Verlust des Markers nur anhalten
          if(arucoY > 0) {
            arucoY -= 5;
          }
         
          arucoZ = 0; 
        } else if(currentMillis - lastMillisAruco > 40000) {     //Bei längerem Verlust des Markers Automatik beenden
         arucoZ = 0; 
         Serial.println("[INFO] Kein Marker mehr gefunden - stoppe Aruco-Navigation!");
         arucoNavigationAktiv = 0;
        } else {                                                //Bei mittlerer Verlustzeit Auto zum Drehen bringen
         arucoZ = 50; 
        }
        break;  
 
      case 2: //nach rechts drehen+fahren
        arucoY = 30;
        arucoZ = 20; 
        lastMillisAruco = currentMillis;    //Timestamp der letzten Aktion
        break;


      case 3: //geradeaus fahren
        arucoY = 80;
        arucoZ = 0;
        lastMillisAruco = currentMillis;    //Timestamp der letzten Aktion
        break;

      case 4: //nach Links drehen+fahren
        arucoY = 30;
        arucoZ = -20; 
        lastMillisAruco = currentMillis;    //Timestamp der letzten Aktion
        break;
    }
  } else {
    arucoZ = 0;
    arucoY = 0; 
  }
}

void Greifeinheit(){
 // kind of stupid solution, I know...
  if (GreifeinheitTransportPos) { // move up
    sendToMega.setBit(moveUp);
  } else {
    sendToMega.clearBit(moveUp);
  }
  if (GreifeinheitGrippingPos) { // move down
    sendToMega.setBit(moveDown);
  } else {
    sendToMega.clearBit(moveDown);
  }
  if (GreiferZu) { // close gripper
    sendToMega.setBit(closeGripper);
  } else {
    sendToMega.clearBit(closeGripper);
  }
  if (GreiferAuf) { // open gripper
    sendToMega.setBit(openGripper);
  } else {
    sendToMega.clearBit(openGripper);
  }
}


void processData() {
   //Controlldata Struktur                                                         DB
   //controldata[0] Joystik 1 --> Links Rechts drehung
   //controldata[1] Joystik 1 --> Hoch Runter Keine Funktion
   //controldata[2] Joystik 2 --> Fahren Links Rechts
   //controldata[3] Joystik 2 --> Fahren Vor Zurück
   //controldata[4] Funktionstaste 1 --> Licht An/Aus
   //controldata[5] Funktionstaste 2 --> enable automaticmode
   //controldata[6] Funktionstaste 3 --> disable automatic mode
   //controldata[7] Funktionstaste 4 --> no function 
   //controldata[8]-[11]  Rücktasten? --> Greifeinheit


   //zuerst werden die Sonderfunktionen abgefragt, da sie evtl eingreifen müssen
  if (ledActiveLastCycle == 0 && controldata[4] == 1) { // rising flank on led button
    beleuchtungAktiv = !beleuchtungAktiv;
  }
  ledActiveLastCycle = controldata[4];

  if (movementCalculatedRaspi == movementDirections::ledon) { // raspi wants led on
    beleuchtungAktiv = true;  
  }
  if (movementCalculatedRaspi == movementDirections::ledoff) { // rapsi wants led off
    beleuchtungAktiv = false;
  }

 #if (DEBUG_LED)
  Serial.print("LED: ");
  Serial.println(beleuchtungAktiv);
 #endif
 #if (DEBUG_CONTROLDATA)
  Serial.print(controldata[4]);
  Serial.print(controldata[5]);
  Serial.print(controldata[6]);
  Serial.println(controldata[7]);
 #endif

  if (!(controldata[5] == 1 && controldata[6] == 1)) { // dont change anything, if both buttons are pressed
    if (automodeOnLastCylce == 0 && controldata[5] == 1) { // automode on button was not pressed in last cycle and is pressed now (rising flank)
      automodeActive = true; // enable auto mode
    }
    if (automodeOffLastCylce == 0 && controldata[6] == 1) {
      automodeActive = false; // disable auto mode
    }
  }
  automodeOnLastCylce = controldata[5]; // remember last states
  automodeOffLastCylce = controldata[6];
  #if (DEBUG_AUTOMODE == 1)
    Serial.print("Automode: ");
    Serial.println(automodeActive);
  #endif

#if DEBUG_RASPI_MOVE
  Serial.println(static_cast<int>(movementCalculatedRaspi));
#endif
if (automodeActive) {
  if (movementCalculatedRaspi == movementDirections::lugp) {
    GreifeinheitGrippingPos = true;
  } else {
    GreifeinheitGrippingPos = false;
  }
  if (movementCalculatedRaspi == movementDirections::lutp) {
    GreifeinheitTransportPos = true;
  } else {
    GreifeinheitTransportPos = false;
  }

  // GreifeinheitGrippingPos = (movementCalculatedRaspi == movementDirections::lugp) ? true : false;
  // GreifeinheitTransportPos = (movementCalculatedRaspi == movementDirections::lutp) ? true: false;

  if (movementCalculatedRaspi == movementDirections::grop) {
    GreiferAuf = true;
  } else {
    GreiferAuf = false;
  }
  if ( movementCalculatedRaspi == movementDirections::grcl) {
    GreiferZu = true;
  } else {
    GreiferZu = false;
  }

  // GreiferAuf = (movementCalculatedRaspi == movementDirections::grop) ? true : false;
  // GreiferZu = (movementCalculatedRaspi == movementDirections::grcl) ? true : false;

} else {
  GreifeinheitGrippingPos = controldata[8];
  GreifeinheitTransportPos = controldata[10];

  GreiferAuf = controldata[9];
  GreiferZu = controldata[11];
}
 

 #if DEBUG_GREIFER
    Serial.print(automodeActive);
    Serial.print(GreifeinheitGrippingPos);
    Serial.print(GreifeinheitTransportPos);
    Serial.print(GreiferAuf);
    Serial.println(GreiferZu);
 #endif

 //Die Aruco-Navigation wird bei jedem Flankenwechsel gestartet/gestoppt!
 if(lastArucoNavigation != controldata[5]) {
   lastArucoNavigation = controldata[5];
   arucoNavigationAktiv = !arucoNavigationAktiv;
   //Serial.print("[INFO] Aruconavigation "); Serial.println(arucoNavigationAktiv);
 }

 if(receiverTimeout) {   // when there is no signal from the gamepad
  // HALT STOP!!!!1!1
  stellwert[0] = 0; // set all movements values to zero 
  stellwert[1] = 0;
  stellwert[2] = 0;
 } else { // when there is a signal from the gamepad  
    // load analogvals from gamepad and scaling (0 - 255  ->  -100 - 100)
    stellwert[0] = controldata[0] / 1.275 - 100; // turning right (0) or left (255)
    stellwert[2] = controldata[2] / 1.275 - 100; // moving right (0) or left (255)
    stellwert[3] = controldata[3] / 1.275 - 100; // moving foward (0) or backward (255)
  }
}

void recordStellwerte() {
  if(abs(stellwert[0]) > 2 || abs(stellwert[1]) > 2 || abs(stellwert[2]) > 2) { //Wenn einer der Stellwertbeträge nicht 0 ist also das Auto nicht stillsteht

    for(int i =99; i>=1; i--) {                                                 //Schleife von 99 bis 1 über das Record Array
      stellwertRecord[i][0] = stellwertRecord[i-1][0];                          //Die letzen Werte alle einen Wert nach unten rücken
      stellwertRecord[i][1] = stellwertRecord[i-1][1];
      stellwertRecord[i][2] = stellwertRecord[i-1][2];      
    }    
    stellwertRecord[0][0] = stellwert[0];                                       //Aktuellste Stellwerte auf Reihe 0 schreiben
    stellwertRecord[0][1] = stellwert[1];
    stellwertRecord[0][2] = stellwert[2];
  }
}

// calulating the Movement direction out of the analog values coming from the gamepad
// returns enum which determins the movement
movementDirections calcMovementFromAnalogVals (float turning, float rightLeft, float fowardBackward) {
  movementDirections movementCalculatedOut; // tmp - just an output

  if (abs(turning) < minStickValRel and abs(rightLeft) < minStickValRel and abs(fowardBackward) >= minStickValRel) {
    // just foward or backward
    movementCalculatedOut = fowardBackward <= 0 ? movementDirections::fw : movementDirections::bw;

  } else if (abs(turning) < minStickValRel and abs(rightLeft) >= minStickValRel and abs(fowardBackward) < minStickValRel) {
    // just right or left
    movementCalculatedOut = rightLeft >= 0 ? movementDirections::r : movementDirections::l;

  } else if (abs(turning) >= minStickValRel and abs(rightLeft) < minStickValRel and abs(fowardBackward) < minStickValRel) {
    // just turning left or turning right
    movementCalculatedOut = turning >= 0 ? movementDirections::tr : movementDirections::tl;

  } else if ((abs(turning) < minStickValRel and abs(rightLeft) >= minStickValRel and abs(fowardBackward) >= minStickValRel)
            or (abs(turning) >= minStickValRel and abs(rightLeft) >= minStickValRel and abs(fowardBackward) >= minStickValRel)) {
    // diagonal movement or an input in all movement types
    // -> turning will be ignored
    if (fowardBackward <= 0) { //foward 
      movementCalculatedOut = rightLeft >= 0 ? movementDirections::fwr : movementDirections::fwl;
    } else { // backward
      movementCalculatedOut = rightLeft >= 0 ? movementDirections::bwr : movementDirections::bwl;
    }
  } else if (abs(turning) >= minStickValRel and abs(rightLeft) < minStickValRel and abs(fowardBackward) >= minStickValRel) {
    // foward/backward with turning 
    if (fowardBackward <= 0) { //foward 
      movementCalculatedOut = turning >= 0 ? movementDirections::fwtr : movementDirections::fwtl;
    } else { // backward
      movementCalculatedOut = turning >= 0 ? movementDirections::bwtr : movementDirections::bwtl;
    }
  } else { // all values are below the set limit or unknown input
      movementCalculatedOut = movementDirections::stop;
  }

  #if DEBUG_MOVEMENT_DIRECTION 
    Serial.print("turning / left/right / foward/backward / movement direction: ");
    Serial.print(turning); Serial.print(" ");
    Serial.print(rightLeft); Serial.print(" ");
    Serial.print(fowardBackward); Serial.print(" ");
    Serial.println(movementDirectionNames[static_cast<int>(movementCalculatedOut)]);
  #endif

  return movementCalculatedOut;
}


void calcMecanumProportion(movementDirections movementCalculatedGamepad, movementDirections movementCalculatedRaspi) {
movementDirections tmpDirections; //

if (automodeActive) { // when automode is active
  tmpDirections = movementCalculatedRaspi; // use input from raspi
} else { // automode is not active
  tmpDirections = movementCalculatedGamepad; // use input from gamepad
}

#if DEBUG_TMPDIRECTION
  Serial.println(static_cast<int>(tmpDirections));
#endif

 switch (tmpDirections) { // do things with the input
  case movementDirections::stop :
    VLneu = 0;
    VRneu = 0;
    HLneu = 0;
    HRneu = 0;
    break;

  case movementDirections::fw :
    VLneu = -100;
    VRneu = -100;
    HLneu = -100;
    HRneu = -100;
    break;

  case movementDirections::bw :
    VLneu = 100;
    VRneu = 100;
    HLneu = 100;
    HRneu = 100;
    break;
  
  case movementDirections::l :
    VLneu = -100;
    VRneu = 100;
    HLneu = 100;
    HRneu = -100;
    break;
  
  case movementDirections::r :
    VLneu = 100;
    VRneu = -100;
    HLneu = -100;
    HRneu = 100;
    break;
  
  case movementDirections::fwl :
    VLneu = -100; 
    VRneu = 0;
    HLneu = 0;
    HRneu = -100;
    break;

  case movementDirections::fwr :
    VLneu = 0;
    VRneu = -100;
    HLneu = -100;
    HRneu = 0;
    break;

  case movementDirections::bwl :
    VLneu = 0;
    VRneu = 100;
    HLneu = 100;
    HRneu = 0;
    break;

  case movementDirections::bwr :
    VLneu = 100;
    VRneu = 0;
    HLneu = 0;
    HRneu = 100;
    break;

  case movementDirections::tl :
    VLneu = -100;
    VRneu = 100;
    HLneu = -100;
    HRneu = 100;
    break;

  case movementDirections::tr :
    VLneu = 100;
    VRneu = -100;
    HLneu = 100;
    HRneu = -100;
    break;

  case movementDirections::fwtl :
    VLneu = -100;
    VRneu = 0;
    HLneu = -100;
    HRneu = 0;
    break;

  case movementDirections::fwtr :
    VLneu = 0;
    VRneu = -100;
    HLneu = 0;
    HRneu = -100;
    break;

  case movementDirections::bwtl :
    VLneu = 0;
    VRneu = 100;
    HLneu = 0;
    HRneu = 100;
    break;

  case movementDirections::bwtr :
    VLneu = 100;
    VRneu = 0;
    HLneu = 100;
    HRneu = 0;
    break;

  default:
    break;
  }

  if (VLneu == 0 && VRneu == 0 && HLneu && HRneu == 0) { // when every new motor value is zero -> stop command
    factorOld = 0.925; // steeper slope while breaking for safety reasons 
  } else { // any other command
    factorOld = 0.9; // acceleration is done a little more chill
  }
  
  factorNew = 1 - factorOld; // calcute other factor
  

  VL = factorOld * VL + factorNew * VLneu; // P-Controller
  VR = factorOld * VR + factorNew * VRneu;
  HL = factorOld * HL + factorNew * HLneu;
  HR = factorOld * HR + factorNew * HRneu;
 
   
  VL >= 0 ? digitalWrite(DO_Dir1, LOW) : digitalWrite(DO_Dir1, HIGH);
  VR >= 0 ? digitalWrite(DO_Dir2, HIGH) : digitalWrite(DO_Dir2, LOW);
  HL >= 0 ? digitalWrite(DO_Dir3, LOW) : digitalWrite(DO_Dir3, HIGH);
  HR >= 0 ? digitalWrite(DO_Dir4, HIGH) : digitalWrite(DO_Dir4, LOW);

  anteilMotor[0] = VL;
  anteilMotor[1] = VR;
  anteilMotor[2] = HL;
  anteilMotor[3] = HR;

  int TMP_SPEED = automodeActive ? MAXSPEED_AUTO : MAXSPEED_MAN; // change speed 

  for(int i = 0; i <= 3; i++) {
    if(abs(anteilMotor[i]) >= 100) { // no override at the moment
        driveValue[i] = TMP_SPEED;
                                                                       
    } else if(abs(anteilMotor[i]) > 13) {
      driveValue[i] = (100 / abs(anteilMotor[i])) * TIMEFACTOR - TIMEFACTOR + TMP_SPEED;

      if(driveValue[i] > UPPERLIMITDELAY) {
        driveValue[i] = UPPERLIMITDELAY;
      }

    } else {
      driveValue[i] = 0;
    }

    interruptGrenzwert[i] = driveValue[i];
  } 
}

void communicationArduinoNano() {
  // Zu sendender Wert, z.B. der Zustand für die Nano-LED:
  byte sendStatus = beleuchtungAktiv ? 1 : 0;

  #if (DEBUG_SEND_STATUS)
  Serial.println(sendStatus);
  #endif

  digitalWrite(DO_SS_ArduinoNano, LOW);   // Chipselect aktivieren
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // 1MHz reicht meist sicher

  // Ein Byte senden, gleichzeitig das Status-Byte vom Nano empfangen
  byte movementStatus = SPI.transfer(sendStatus);

  SPI.endTransaction();
  digitalWrite(DO_SS_ArduinoNano, HIGH);  // Chipselect deaktivieren

  // Nun ist movementStatus das vom Nano transferierte Status-Byte (z.B. Bit0 = movementIsSafe)
  if (movementStatus & 0x01) {
    sendToMega.setBit(movementIsSafe);
  } else {
    sendToMega.clearBit(movementIsSafe);
  }

  #if (DEBUG_SPI_RECIEVE)
    Serial.println(sendToMega.readBit(movementIsSafe));
  #endif
}

void nachrichtZusammensetzen() {

    akkuProzent = 13.33 * Spannung_36V - 452;   //Akkustand errechnen durch lineare Funktion zwischen 34 Volt und 41,5 Volt
    akkuProzent > 100 ? akkuProzent = 100 : akkuProzent;
    akkuProzent < 0 ? akkuProzent = 0 : akkuProzent;
    

    Nachricht = "1/OK/";
    Nachricht.concat(int(VL));
    Nachricht.concat("%/");
    Nachricht.concat(int(VR));
    Nachricht.concat("%/");
    Nachricht.concat(int(HL));
    Nachricht.concat("%/");
    Nachricht.concat(int(HR));
    Nachricht.concat("%/");



    Nachricht.concat(Spannung_36V);
    Nachricht.concat("V  ");
    Nachricht.concat(akkuProzent);
    Nachricht.concat("%/");
    Nachricht.concat(Spannung_24V);
    Nachricht.concat("V/");
    Nachricht.concat(Spannung_5V);
    Nachricht.concat("V/");

    receiverTimeout ?  Nachricht.concat("-1") : Nachricht.concat(rssi);
    Nachricht.concat("/");

    Nachricht.concat(arucoNavigationAktiv);
    Nachricht.concat("/");

    switch(raspiData[2]) {
      case 0: Nachricht.concat("STOP"); break;
      case 1: Nachricht.concat("s"); break;
      case 2: Nachricht.concat("->"); break;
      case 3: Nachricht.concat(" |"); break;
      case 4: Nachricht.concat("<-"); break;
      case 5: Nachricht.concat("s"); break;
      case 6: Nachricht.concat("-"); break;
      default: Nachricht.concat("s"); break;
    }
    Nachricht.concat("/");

    Nachricht.concat(int(distanzVorne));
    Nachricht.concat("/");
    Nachricht.concat(int(distanzLinks));
    Nachricht.concat("/");
    Nachricht.concat(int(distanzRechts));
    Nachricht.concat("/");
    Nachricht.concat(int(distanzHinten));
    Nachricht.concat("/");

    Nachricht.concat("thisCouldBeYourMessage"); // was info if ultrasonic is switched on or off
    Nachricht.concat("/");

    Nachricht.concat(Notaus_OK);  //1 = alles ok 0 = notaus
    Nachricht.concat("/");

    Nachricht.concat("0");
    Nachricht.concat("/");
    
}

movementDirections intToMovementDirection(int val) {
    if(val >= 0 && val <= 14) {
        return static_cast<movementDirections>(val);
    } else {
        return movementDirections::stop; // Standardwert, falls außerhalb
    }
}

bool stringIsWhitespaceOrEmpty(const char *str) {
  while (*str) {
    if (!isspace(*str)) return false;
    str++;
  }
  return true;
}

void recvWithEndMarker() {
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial2.available() > 0) {
  rc = Serial2.read();
  if (rc != endMarker) {
    receivedChars[ndx] = rc;
    ndx++;
    if (ndx >= 63) {
      ndx = 62;
    }
  } else {
    receivedChars[ndx] = '\0'; // terminate the string
    ndx = 0;
    if (stringIsWhitespaceOrEmpty(receivedChars)) {
      
      return;
    }

  #if (DEBUG_DATA_RASPI)
    Serial.println(receivedChars);
  #endif

  intFromRaspi = atoi(receivedChars);
  movementCalculatedRaspi = static_cast<movementDirections>(intFromRaspi);
  intFromRaspi = 0; // safety
    //pullData_RASPI(receivedChars);
  }
 }
}

void sendAutomodeActive() {
  static bool lastAutomodeActive = false; 
   if (automodeActive != lastAutomodeActive) {
    Serial2.write(automodeActive ? "1" : "0");
   }
   lastAutomodeActive = automodeActive;
}

void loop() {
  //DB Erweiterung Greifeinheit Kommandos Senden
  Wire.beginTransmission(20); // transmit to device #4
  uint8_t tmpSendToMega = sendToMega.getByte();
  //Serial.print(tmpSendToMega);
  Wire.write(tmpSendToMega); // sends one byte  
  Wire.endTransmission();    // stop transmitting


  //Scanner();
  currentMillis = millis();

  Notaus_OK = digitalRead(DI_Notaus);
 if (radio.receiveDone()) // Got one!
  {
    lastMillisRadiopacketReceive = currentMillis;
    //receivedLast = now;
    receiverTimeout = false;
     digitalWrite(DO_LED, HIGH);
    // Print out the information:
    
    #if debugLevel >= 1
      Serial.print("Received from node ");
      Serial.print(radio.SENDERID, DEC);
      Serial.print(": [");
    #endif
    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
  
    
    for (byte i = 0; i < radio.DATALEN; i++) {
      datenstring += (char)radio.DATA[i];
    }

    
    rssi = radio.RSSI;


    
    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.
    #if debugLevel >= 1
      Serial.print(datenstring);
      Serial.print("], RSSI ");
      Serial.println(radio.RSSI);
      
    #endif

    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println("ACK sent");
    }

    
    pullData_FUNKVERBINDUNG(datenstring);
    
    
    datenstring = "";
  }

   recvWithEndMarker();
   processData(); // load data from gamepad
   Greifeinheit(); // do stuff with the gripping unit
   movementCalculatedGamepad = calcMovementFromAnalogVals(stellwert[0], stellwert[2], stellwert[3]); // calculate the movement direction from stick positions
   calcMecanumProportion(movementCalculatedGamepad, movementCalculatedRaspi); // calculate the proportion of the Mecanum wheels for the given movement direction
   communicationArduinoNano(); // 
  

  
  if(currentMillis - lastMillisRadiopacketReceive > 800  && !receiverTimeout) {
    Serial.println("[WARN] Receiver Timeout");
    Serial.println("[INFO] Movement stopped");
    receiverTimeout = true;
    digitalWrite(DO_LED, LOW);
    recordPlaybackCounter = 0;
  } 
 
  
  // recvWithEndMarker();

  // if(currentMillis - lastMillisRaspi > 100) {
  //   nachrichtZusammensetzen();
  //   Serial2.println(Nachricht);
  //   lastMillisRaspi = currentMillis;
  // }

  if(currentMillis - lastMillis50MS > 50) {
    if(!receiverTimeout) {
      recordStellwerte();
    } else {
      recordPlaybackCounter++;
    }
      
    
    

    Spannung_36V = (Spannung_36V * 0.95) + (analogRead(AI_SpgMess_36V) * 0.006717 + 13.594) *0.05;    //Spannung aus AD Wandler umrechnen von digits in Volt, außerdem mit 95%/5% Glättung versehen
    Spannung_24V = (Spannung_24V * 0.9) + (analogRead(AI_SpgMess_24V) * 0.002857 + 12.73) * 0.1;      //dasselbe für die anderen Spannungsteiler
    Spannung_5V = analogRead(AI_SpgMess_5V) * 0.001526 + 0.4957;                                      //dasselbe für die anderen Spannungsteiler
    

    //Serial.print("5V: ");  Serial.println(Spannung_5V);

    lastMillis50MS = currentMillis;
  }
 sendAutomodeActive();
}