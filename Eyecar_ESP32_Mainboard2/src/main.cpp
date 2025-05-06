#include <Arduino.h>
#include <SPI.h>
#include <PCF8574.h>        //DB
#include <RFM69.h>
#include <Wire.h>
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

#define debugLevel 0

//DB Für Greifeinheit
char fromSerialMon = ' ';
byte sendToMega = 0;
bool GreifeinheitHoch, GreifeinheitRunter, GreiferAuf, GreiferZu;

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

volatile float VL, VR, HL, HR;
float VLneu, VRneu, HLneu, HRneu;
float stellwert[4], stellwertSoll[4];

unsigned long lastStep[4];

unsigned long currentMillis, lastMillisRadiopacketReceive, lastMillisSPINano, lastMillisRaspi, lastMillis50MS, lastMillisAruco;

int parityCounter;
bool high;
byte SPISEND,SPIRECEIVE;    

int TIMEFACTOR = 50;      //Vorher: 50
int MAXSPEED = 8;         //Höchstgeschwindigkeit: je kleiner die Zahl desto schneller!
int UPPERLIMITDELAY = 300;

bool enableUltraschallSensorik = true;
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

volatile byte DO_Step1 = 26, DO_Step2 = 27, DO_Step3 = 32, DO_Step4 = 33;
volatile byte DO_Dir1 = 12, DO_Dir2 = 13, DO_Dir3 = 14, DO_Dir4 = 15;

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

void ultraschallautomatik() {
    if(enableUltraschallSensorik) {
      /**
      if(stellwertSoll[1] >= 0) {                                           //Wenn Knüppel nach vorne gedrückt wird zum vorwörtsfahren
        if(distanzVorne > 10 && distanzVorne < 120) {                       //und der Ultraschallsensor mittlere Entfernung zu Objekt hat
          UltrschallbremseY = 0.00864 * distanzVorne -0.0364;               //anteilig den Stellwert reduzieren bzw. den Faktor verringern
        } else if(distanzVorne >= 120) {                                    //wenn kein Objekt in Sichtweite
          UltrschallbremseY = 1;                                            //kein Reduktionsfaktor
        } else {                                                            //wenn Objekt sehr sehr sehr nah dran
          UltrschallbremseY = 0.05;                                         //auf Minimum reduzieren
        } 
      } else {                                                              //das selbe für Knüppel nach hinten/rückwärts aber mit anderen Sensoren
        if(distanzHinten > 10 && distanzHinten < 120) {
          UltrschallbremseY = 0.00864 * distanzHinten -0.0364;
        } else if(distanzHinten >= 120) {
          UltrschallbremseY = 1;
        } else {
          UltrschallbremseY = 0.05;
        } 
      }

      **/

      if(stellwertSoll[1] + arucoY >= 0) {                                           //Wenn Knüppel nach vorne gedrückt wird zum vorwörtsfahren
        if(distanzVorne > 8 && distanzVorne < 70) {                       //und der Ultraschallsensor mittlere Entfernung zu Objekt hat
          UltrschallbremseY = 0.0152 * distanzVorne -0.073;               //anteilig den Stellwert reduzieren bzw. den Faktor verringern
        } else if(distanzVorne >= 70) {                                    //wenn kein Objekt in Sichtweite
          UltrschallbremseY = 1;                                            //kein Reduktionsfaktor
        } else {                                                            //wenn Objekt sehr sehr sehr nah dran
          UltrschallbremseY = 0.05;                                         //auf Minimum reduzieren
        } 
      } else {                                                              //das selbe für Knüppel nach hinten/rückwärts aber mit anderen Sensoren
        if(distanzHinten > 8 && distanzHinten < 70) {
          UltrschallbremseY = 0.0152 * distanzHinten -0.073;
        } else if(distanzHinten >= 70) {
          UltrschallbremseY = 1;
        } else {
          UltrschallbremseY = 0.05;
        } 
      }





      if(stellwertSoll[2] >= 0) {                                           //Das selbe nochmal für links/rechts
        if(distanzRechts > 8 && distanzRechts < 70) {
          UltrschallbremseX = 0.0152 * distanzRechts -0.073;
        } else if(distanzRechts >= 70) {
          UltrschallbremseX = 1;
        } else {
          UltrschallbremseX = 0.05;
        } 
      } else {
        if(distanzLinks > 8 && distanzLinks < 70) {
          UltrschallbremseX = 0.0152 * distanzLinks -0.073;
        } else if(distanzLinks >= 70) {
          UltrschallbremseX = 1;
        } else {
          UltrschallbremseX = 0.05;
        } 
      }
     

    } else {                        //Wenn Automatik deaktiviert ist
      UltrschallbremseX = 1;        //Reduktionsfaktor inaktiv setzen
      UltrschallbremseY = 1;        //daher Faktor 1
    }
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
  if(GreifeinheitHoch and !GreifeinheitRunter){
    sendToMega = 2;
  } else if (GreifeinheitRunter and !GreifeinheitHoch ){
    sendToMega = 1;
  } else if (GreiferAuf and !GreiferZu){
    sendToMega = 4;
  } else if (GreiferZu and !GreiferAuf){
    sendToMega = 3;
  } else {sendToMega = 0;}
}

void processData() {
        //Controlldata Struktur                                                         DB
        //controldata[0] Joystik 1 --> Links Rechts drehung
        //controldata[1] Joystik 1 --> Hoch Runter Keine Funktion
        //controldata[2] Joystik 2 --> Fahren Links Rechts
        //controldata[3] Joystik 2 --> Fahren Vor Zurück
        //controldata[4] Funktionstaste 1 --> Licht An/Aus
        //controldata[5] Funktionstaste 2 --> Autonome Aruko NAvigation AN/AUS
        //controldata[6] Funktionstaste 3 --> Ultraschallerkennung AN/AUS
        //controldata[7] Funktionstaste 4 --> Ohne Funktion (Vielleicht Auto GReifen AN/AUS)
        //controldata[8]-[11] Pfeiltasten?, Rücktasten? --> Greifeinheit


        //zuerst werden die Sonderfunktionen abgefragt, da sie evtl eingreifen müssen
      beleuchtungAktiv = controldata[4];
      if(enableUltraschallSensorik xor !controldata[6]) {
        Serial.print("[INFO] ultraschallsensorik "); Serial.println(!controldata[6]);
      }
      enableUltraschallSensorik = !controldata[6];

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



      if(receiverTimeout) {   //Wenn kein Sendersignal: 
        if(recordPlaybackCounter < 100) {   //Wenn der Playback-Stack noch nicht abgefahren ist:
          stellwert[0] = -stellwertRecord[recordPlaybackCounter][0];   //Playback der Aufzeichnung
          stellwert[1] = -stellwertRecord[recordPlaybackCounter][1];   //Reinladen der Stellwerte aus dem Stack
          stellwert[2] = -stellwertRecord[recordPlaybackCounter][2];   //Negieren, weil man ja rückwärts fahren will
          
        } else if(recordPlaybackCounter == 100) {
          stellwert[0] = 0;               //Wenn der Playback-Stack zuende abgefahren ist:
          stellwert[1] = 0;               //Fahrzeug stoppen.
          stellwert[2] = 0;               //
          Serial.println("[INFO] Route Playback complete");
          recordPlaybackCounter++;
        } 
      } else {                //Wenn Sendersignal vorhanden:      Daten vom Controldata-Array (Empfangene Daten) laden

        stellwertSoll[0] = controldata[0] / 1.275 -100;                   //Drehung links/rechts      Wertebereich -100 - +100%
        stellwertSoll[1] = controldata[3] / 1.275 -100;                   //Vor/rückwärts
        stellwertSoll[2] = controldata[2] / 1.275 -100;                   //links/rechts


        
        arucoAutomatik();
        ultraschallautomatik();

        stellwert[0] =  stellwertSoll[0] + arucoZ;                        //Drehung wird nicht durch Ultraschallsensoren beeinflusst
        stellwert[1] = (stellwertSoll[1] + arucoY)* UltrschallbremseY;    //Y-Richtung wird durch Ultraschallsensoren beeinflusst
        stellwert[2] =  stellwertSoll[2] * UltrschallbremseX;             //X-Richtung wird durch Ultraschallsensoren beeinflusst
      }
      
    

      //Serial.println(driveValue[0]);
      //LX, LY, RX, RY
      if(datenVomNano[1] > 0) {
        distanzVorne = (distanzVorne * 0.999) + (0.001* datenVomNano[1]);
      }
      if(datenVomNano[2] > 0) {
        distanzLinks = (distanzLinks * 0.999) + (0.001* datenVomNano[2]);
      }
      if(datenVomNano[3] > 0) {
        distanzRechts = (distanzRechts * 0.999) + (0.001* datenVomNano[3]);
      }
      if(datenVomNano[4] > 0) {
        distanzHinten = (distanzHinten * 0.999) + (0.001* datenVomNano[4]);
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

void calcMecanumProportion() {
  VLneu =  stellwert[0] + stellwert[1] + stellwert[2];       // Wertebereich -300 - +300
  VRneu = -stellwert[0] + stellwert[1] - stellwert[2];
  HLneu =  stellwert[0] + stellwert[1] - stellwert[2];
  HRneu = -stellwert[0] + stellwert[1] + stellwert[2];
  
  //float faktorAlt = 0.9997, faktorNeu = 0.0003;
  //float faktorAlt = 0.95, faktorNeu = 0.05;
  float faktorAlt = 0.975, faktorNeu = 0.025;

  VL = faktorAlt * VL + faktorNeu * VLneu;
  VR = faktorAlt * VR + faktorNeu * VRneu;
  HL = faktorAlt * HL + faktorNeu * HLneu;
  HR = faktorAlt * HR + faktorNeu * HRneu;
 
  if(printctr == 100) {
    printctr = 0;
    //Serial.print(distanzVorne); Serial.print("    "); Serial.println(stoppBremseVorne);
  //Serial.print(VL);Serial.print("   "); Serial.print(VR); Serial.print("    "); Serial.print(HL); Serial.print("    "); Serial.println(HR);
  //Serial.println(datenVomNano[1]);
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

  for(int i = 0; i<= 3; i++) {
    if(abs(anteilMotor[i]) > 100) {
        driveValue[i] = MAXSPEED;                                                                  //Vorher:10
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
  
      
      
      /**for(int i = 0; i<= 3; i++) {         //Keine Ahnung wofür das da ist
        if(genClock[i] > 100/1) {
          
          genClock[i] =0;
        }
      }**/

      



 
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

    Nachricht.concat(enableUltraschallSensorik);
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
  //DB Erweiterung Greifeinheit Kommandos Senden
  Wire.beginTransmission(20); // transmit to device #4
  Wire.write(sendToMega);              // sends one byte  
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

   
   processData();
   Greifeinheit();
   calcMecanumProportion();
   communicationArduinoNano();
  

  
  if(currentMillis - lastMillisRadiopacketReceive > 800  && !receiverTimeout) {
    Serial.println("[WARN] Receiver Timeout");
    Serial.println("[INFO] Starting Route Playback");
    receiverTimeout = true;
    digitalWrite(DO_LED, LOW);
    recordPlaybackCounter = 0;
  } 
 
  recvWithEndMarker();

  if(currentMillis - lastMillisRaspi > 100) {
    nachrichtZusammensetzen();
    Serial2.println(Nachricht);
    lastMillisRaspi = currentMillis;

   // Serial.print("stellwertSoll[1]:"); Serial.print(stellwertSoll[1]); Serial.print(" distanzVorne "); Serial.print(distanzVorne); Serial.print("  usb:");  Serial.print(UltrschallbremseY); Serial.println();
      //Serial.println(controldata[6]);

      
      // Serial.print(" 24V: "); Serial.print(analogRead(AI_SpgMess_24V)); Serial.print("  5V: ");  Serial.print(analogRead(AI_SpgMess_5V)); Serial.println();

      //Serial.print("Datenvomnano: "); Serial.print(datenVomNano[1]); Serial.print(" DistanzVorne: "); Serial.print(distanzVorne); Serial.print(" UltrschallbremseY: "); Serial.println(UltrschallbremseY);
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






