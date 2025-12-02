#include <Arduino.h>
#include <SPI.h>
#include <PCF8574.h>        //DB
#include <RFM69.h>
#include <Wire.h>
#include "movementDirections.h"
#include "MotorByte.h"
#include "CtrlByte.h"
#include "Buttons.h"

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
#define automatic 0 // 0: static use of gamepad for movement control, 1: static use of raspi for movement control,
                    // 2: Mode can be switched with gamepad (not implemented)

//DB Für Greifeinheit
char fromSerialMon = ' ';
CtrlByte sendToMega; // Byte for controling gripper and linear unit
bool GreifeinheitHoch, GreifeinheitRunter, GreiferAuf, GreiferZu;
Buttons buttonHoch(&GreifeinheitHoch);
Buttons buttonRunter(&GreifeinheitRunter);
Buttons buttonAuf(&GreiferAuf);
Buttons buttonZu(&GreiferZu);

String datenstring;
unsigned long receivedLast;
bool receiverTimeout = true;

bool output[25];
bool parityBerechnet, parityReceived;

bool Notaus_OK;

int controldata[16];
int raspiData[10];
float driveValue[4];

float anteilMotor[4];
float minStickValRel = 35; // minimal relativ value of the analogsticks where a movement should be initiated

movementDirections movementCalculatedGamepad, movementCalculatedRaspi; // calculated movement from analog values, initialised with stop = 0 
volatile float VL, VR, HL, HR;
float VLneu, VRneu, HLneu, HRneu;
float stellwert[4], stellwertSoll[4];

unsigned long lastStep[4];

unsigned long currentMillis, lastMillisRadiopacketReceive, lastMillisSPINano, lastMillisRaspi, lastMillis50MS, lastMillisAruco;

int parityCounter;
bool high;
byte SPISEND,SPIRECEIVE;    

int TIMEFACTOR = 50;      //Vorher: 50
int MAXSPEED = 7;         //Höchstgeschwindigkeit: je kleiner die Zahl desto schneller!
int UPPERLIMITDELAY = 300;



int stoppBremseVorne;


volatile int count;    // Trigger 
int totalInterrupts;   // counts the number of triggering of the alarm
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint cycles;
int genClock[4];
bool stateOutput[4];
volatile int interruptGrenzwert[4];

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

float UltrschallbremseY = 1, UltrschallbremseX = 1;   //Faktor 0-1! 1 = keine Einschränkung, 0 = maximale Einschränkung
int arucoY, arucoZ;
bool arucoDrehend;

void IRAM_ATTR onTime() {
  

   portENTER_CRITICAL_ISR(&timerMux);
   genClock[0]++;
   genClock[1]++;
   genClock[2]++;
   genClock[3]++;
   
   for(int i = 0; i<= 3; i++) {
    if(genClock[i] > interruptGrenzwert[i] && interruptGrenzwert[i] != 0) {
      stateOutput[i] = !stateOutput[i];
      digitalWrite(belegungOutputsStep[i], stateOutput[i]); 
      genClock[i] = 0;
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
  sendToMega.linkBits(moveUp, moveDown, buttonHoch.getImpulse(), buttonRunter.getImpulse());
  sendToMega.linkBits(openGripper, closeGripper, buttonAuf.getImpulse(), buttonZu.getImpulse());

  Serial.print(sendToMega.readBit(moveUp));
  Serial.print(sendToMega.readBit(moveDown));
  Serial.print(sendToMega.readBit(openGripper));
  Serial.println(sendToMega.readBit(closeGripper));
}


void processData() {
   //Controlldata Struktur                                                         DB
   //controldata[0] Joystik 1 --> Links Rechts drehung
   //controldata[1] Joystik 1 --> Hoch Runter Keine Funktion
   //controldata[2] Joystik 2 --> Fahren Links Rechts
   //controldata[3] Joystik 2 --> Fahren Vor Zurück
   //controldata[4] Funktionstaste 1 --> Licht An/Aus
   //controldata[5] Funktionstaste 2 --> Autonome Aruko NAvigation AN/AUS
   //controldata[6] Funktionstaste 3 --> not in use
   //controldata[7] Funktionstaste 4 --> Ohne Funktion (Vielleicht Auto GReifen AN/AUS)
   //controldata[8]-[11]  Rücktasten? --> Greifeinheit


   //zuerst werden die Sonderfunktionen abgefragt, da sie evtl eingreifen müssen
 beleuchtungAktiv = controldata[4];

 GreifeinheitHoch = controldata[8];
 GreifeinheitRunter = controldata[10];

 GreiferAuf = controldata[9];
 GreiferZu = controldata[11];

 //Die Aruco-Navigation wird bei jedem Flankenwechsel gestartet/gestoppt!
 if(lastArucoNavigation != controldata[5]) {
   lastArucoNavigation = controldata[5];
   arucoNavigationAktiv = !arucoNavigationAktiv;
   Serial.print("[INFO] Aruconavigation "); Serial.println(arucoNavigationAktiv);
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

//calulating the Movement direction out of the analog values coming from the gamepad
// returns enum which determins the movement
movementDirections calcMovementFromAnalogVals (float turning, float rightLeft, float fowardBackward) {
  movementDirections movementCalculatedOut; // tmp - just an output

  if (abs(turning) < minStickValRel and abs(rightLeft) < minStickValRel and abs(fowardBackward) >= minStickValRel) {
    // just foward or backward
    movementCalculatedOut = fowardBackward >= 0 ? movementDirections::fw : movementDirections::bw;

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
    if (fowardBackward >= 0) { //foward 
      movementCalculatedOut = rightLeft >= 0 ? movementDirections::fwr : movementDirections::fwl;
    } else { // backward
      movementCalculatedOut = rightLeft >= 0 ? movementDirections::bwr : movementDirections::bwl;
    }
  } else if (abs(turning) >= minStickValRel and abs(rightLeft) < minStickValRel and abs(fowardBackward) >= minStickValRel) {
    // foward/backward with turning 
    if (fowardBackward >= 0) { //foward 
      movementCalculatedOut = turning >= 0 ? movementDirections::fwtr : movementDirections::fwtl;
    } else { // backward
      movementCalculatedOut = turning >= 0 ? movementDirections::bwtr : movementDirections::bwtl;
    }
  } else { // all values are below the set limit or unknown input
      movementCalculatedOut = movementDirections::stop;
  }

  #if debugLevel == -1
    Serial.print("turning / left/right / foward/backward / movement direction: ");
    Serial.print(turning); Serial.print(" ");
    Serial.print(rightLeft); Serial.print(" ");
    Serial.print(fowardBackward); Serial.print(" ");
    Serial.println(movementDirectionNames[static_cast<int>(movementCalculatedOut)]);
  #endif

  return movementCalculatedOut;
}

void calcMecanumProportion(movementDirections movementCalculatedGamepad, movementDirections movementCalculatedRaspi) {
#if automatic == 0
  switch (movementCalculatedGamepad)
#elif automatic == 1
  switch (movementCalculatedRaspi)
#endif
  {
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
    VLneu = 100;
    VRneu = 0;
    HLneu = 100;
    HRneu = 0;
    break;

  case movementDirections::bwtr :
    VLneu = 0;
    VRneu = 100;
    HLneu = 0;
    HRneu = 100;
    break;

  default:
    break;
  }

  
  float faktorAlt = 0.9, faktorNeu = 0.1; // first setting faktorAlt = 0.975, faktorNeu = 0.025;

  VL = faktorAlt * VL + faktorNeu * VLneu;
  VR = faktorAlt * VR + faktorNeu * VRneu;
  HL = faktorAlt * HL + faktorNeu * HLneu;
  HR = faktorAlt * HR + faktorNeu * HRneu;
 
  if(printctr == 100) {
    printctr = 0;
  } else {
    printctr++;
  }
   
  VL >= 0 ? digitalWrite(DO_Dir1, LOW) : digitalWrite(DO_Dir1, HIGH);
  VR >= 0 ? digitalWrite(DO_Dir2, HIGH) : digitalWrite(DO_Dir2, LOW);
  HL >= 0 ? digitalWrite(DO_Dir3, LOW) : digitalWrite(DO_Dir3, HIGH);
  HR >= 0 ? digitalWrite(DO_Dir4, HIGH) : digitalWrite(DO_Dir4, LOW);

  anteilMotor[0] = VL;
  anteilMotor[1] = VR;
  anteilMotor[2] = HL;
  anteilMotor[3] = HR;

  for(int i = 0; i <= 3; i++) {
    if(abs(anteilMotor[i]) >= 100) { // no override at the moment
      driveValue[i] = MAXSPEED; //Vorher:10
                                                                       
    } else if(abs(anteilMotor[i]) > 13) {
      driveValue[i] = (100 / abs(anteilMotor[i])) * TIMEFACTOR - TIMEFACTOR + MAXSPEED;

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
        
  datenZumNano[1] = beleuchtungAktiv;

  if(flag_new_packet) {                                                           //Wenn wir neue Daten schicken/bekommen müssten 
    if(currentMillis - lastMillisSPINano > 20) {                                 //und lange nichts mehr ausgetauscht haben
      SPISEND = 0xFE;                                                             //Sende Beginn-packet
      flag_new_packet = false;                                                    //flag setzen

      digitalWrite(DO_SS_ArduinoNano, LOW);                                       //Chipselect anwählen
      SPI.beginTransaction(SPISettings(2400000, MSBFIRST, SPI_MODE0));            //SPI-Verbindung beginnen
      SPIRECEIVE = SPI.transfer(SPISEND);                                         //Send the mastersend value to slave also receives value from slave

      digitalWrite(DO_SS_ArduinoNano, HIGH);                                      //Chipselect abwählen
      SPI.endTransaction();                                                       //Ende Gelände
      //Serial.println("SPI SENT NEW PAKET FLAG");
      //Serial.println("=========================================");
      lastMillisSPINano = currentMillis;

    }
  } else {                                                                        //Flag=false bedeutet die eigentlichen Daten übertragen
    if(currentMillis - lastMillisSPINano > 2) {
      SPISEND = datenZumNano[dataIndexCounter];                                     //Zu sendende Daten reinladen
      digitalWrite(DO_SS_ArduinoNano, LOW);                                         //Chipselect anwählen
      SPI.beginTransaction(SPISettings(2400000, MSBFIRST, SPI_MODE0));              //SPI-Verbindung beginnen
      SPIRECEIVE = SPI.transfer(SPISEND);                                           //Send the mastersend value to slave also receives value from slave
      datenVomNano[dataIndexCounter] = SPIRECEIVE;                                  //Empfangene Daten in Array einsortieren
      

      //Serial.print(SPISEND); Serial.print("       "); Serial.println(SPIRECEIVE);

      dataIndexCounter++;        
      if(dataIndexCounter >=9) {                                                    //Bei Überlauf = alle Pakete wurden übertragen
        dataIndexCounter = 0;                                                       //Counter resetten
        flag_new_packet = true;                                                     //flag wieder setzen für neues Paket
       
        //Serial.println("=========================================");
        
      }

      digitalWrite(DO_SS_ArduinoNano, HIGH);                                        //Chipselect abwählen
      SPI.endTransaction();                                                         //Ende Gelände
                                                         //Counter erhöhen für nächste Daten
      lastMillisSPINano = currentMillis;
    }
  }

 
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
    pullData_RASPI(receivedChars);
  }
 }
}



void loop() {
  buttonHoch.update();
  buttonZu.update();
  buttonRunter.update();
  buttonAuf.update();

  //DB Erweiterung Greifeinheit Kommandos Senden
  Wire.beginTransmission(20); // transmit to device #4
  uint8_t tmpSendToMega = sendToMega.getByte();
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

   
   processData(); // load data from gamepade
   Greifeinheit(); // do stuff with the gripping unit
   movementCalculatedGamepad = calcMovementFromAnalogVals(stellwert[0], stellwert[2], stellwert[3]); // calculate the movement direction
   calcMecanumProportion(movementCalculatedGamepad, movementCalculatedRaspi); // calculate the proportion of the Mecanum wheels for the given movement direction
   communicationArduinoNano(); // 
  

  
  if(currentMillis - lastMillisRadiopacketReceive > 800  && !receiverTimeout) {
    Serial.println("[WARN] Receiver Timeout");
    Serial.println("[INFO] Movement stopped");
    receiverTimeout = true;
    digitalWrite(DO_LED, LOW);
    recordPlaybackCounter = 0;
  } 
 
  recvWithEndMarker();

  if(currentMillis - lastMillisRaspi > 100) {
    nachrichtZusammensetzen();
    Serial2.println(Nachricht);
    lastMillisRaspi = currentMillis;
  }

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

}






