#include <avr/wdt.h>
#include <SPI.h>

//######################### Setup USB HID #######################################


#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);


int stickLX, stickLY, stickRX, stickRY;

//######################## Setup Funkmodul ##################################

#include <RFM69.h>
#define NETWORKID     119   // Must be the same for all nodes (0 to 255)
#define MYNODEID      180   // My node ID (0 to 255)
#define TONODEID      213   // Destination node ID (0 to 254, 255 = broadcast) vorher 255
#define FREQUENCY     RF69_868MHZ
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRET" // Use the same 16-byte key on all nodes
#define USEACK        false // Request ACKs or not
#define FREQUENCY_EXACT 869500000
//#define IS_RFM69HW

RFM69 radio(8, 3);   //SS-Pin, Interrupt-Pin
String datenstring;
String controlMessage;
//########################### Variablen ####################################
#define DEBUG_SERIAL true

unsigned long runtime, lastSend;

bool sonderfunktionen[12];
bool initWerte = true;
int statusLED = 5;

void setup() {
        #if DEBUG_SERIAL
          Serial.begin(115200);
          Serial.println("ready");
        #endif
        pinMode(statusLED, OUTPUT);
        //Serial.println("Start");
        setupRFM69();
    
        if (Usb.Init() == -1) {                             //wenn kein USB Gamepad erkannt
                #if DEBUG_SERIAL
                 Serial.println("OSC did not start.");
                #endif          
                if(true){
                  //
                  Serial.println("db1");
                  digitalWrite(statusLED, HIGH);            //LED blinken lassen
                  delay(300);
                  digitalWrite(statusLED, LOW);
                  delay(300);
                }
        }
        

        if (!Hid.SetReportParser(0, &Joy)) {
          
           ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
        }

        //wdt_enable(WDTO_2S);   // Watchdog auf 2 s stellen

        
}

void setupRFM69() {
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(30);
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
  delay(1000);
}
void loop() {
        Usb.Task();
        runtime = millis();      
        
        if(runtime > 1100) {                    //1,1 Sekunden nach Start und Gamepad aktiviert alle Werte initalisieren
          if(initWerte) {
            initWerte = false;
            #if DEBUG_SERIAL
              Serial.println("init werte");
            #endif  
            for(int i = 0; i<= 10; i++) {
                sonderfunktionen[i] = 0; 
            }
          }
        }

        if(runtime - lastSend >= 4) {                 //Nachricht zusammensetzen mit slash ziwschen den einzelnen werten, jede 4 Millisekunden
          controlMessage = String(stickLX);
          controlMessage.concat("/");
          controlMessage.concat(String(stickLY));
          controlMessage.concat("/");
          controlMessage.concat(String(stickRX));
          controlMessage.concat("/");
          controlMessage.concat(String(stickRY));
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[1]);
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[2]);
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[3]);
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[4]);
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[5]);
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[6]);
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[7]);
          controlMessage.concat("/");
          controlMessage.concat(sonderfunktionen[8]);
          controlMessage.concat("/");
          
          if(runtime > 1100) { 
            if(stickLY == 0 && stickLY == 0 && stickRX == 0 && stickRY == 0) {          //Wenn alle Werte 0 sind, ist wahrscheinlich das gamepad abgezogen
              //Serial.println("db2");
              digitalWrite(statusLED, HIGH);                                            //dann LED blinken lassen
              delay(200);
              digitalWrite(statusLED, LOW);
              delay(200);
            } else {
             //Serial.println("db3");
             analogWrite(statusLED, 90);                                              //ansonsten alles OK und LED (etwas dunkler) dauerhaft leuchten lassen
             sende(controlMessage);                                                 //SENDEN der Werte via Funk
            }
          }

          lastSend = runtime;
        }

        
        
        
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
        stickLX = evt->X;
        stickLY = 255 - evt->Y;     //teilweise daten in den richtigen Wertebereich schieben
        stickRX = evt->Z2;
        stickRY = 255 - evt->Rz;    //teilweise daten in den richtigen Wertebereich schieben
        /**Serial.print("X1: ");
        PrintHex<uint8_t > (evt->X, 0x80);
        Serial.print("\tY1: ");
        PrintHex<uint8_t > (evt->Y, 0x80);
        //Serial.print("\to: ");
        //PrintHex<uint8_t > (evt->Z1, 0x80);
        Serial.print("\tX2: ");
        PrintHex<uint8_t > (evt->Z2, 0x80);
        Serial.print("\tY2: ");
        PrintHex<uint8_t > (evt->Rz, 0x80);
        Serial.println("");
        **/

        
        
        
}

void sende(String sendeString) {
  static char sendbuffer[62];     //Puffer reservieren
  static int sendlength = 0;      //Länge erst mal 0

  datenstring = "";
  datenstring.concat(sendeString);  
  datenstring.toCharArray(sendbuffer, 62);    //String in charArray umformen
  Serial.println(sendbuffer);

  if (USEACK)
      {
        if (radio.sendWithRetry(TONODEID, sendbuffer, datenstring.length())) {    //Acknowledge ist eigentlich sowieso ausgeschaltet
          
        }
          //Serial.println("ACK received!");
        
          //Serial.println("no ACK received :(");
      }

      // If you don't need acknowledgements, just use send():
      
      else // don't use ACK
      {
        radio.send(TONODEID, sendbuffer, datenstring.length());     //HIER wird eigentlich überhaupt erst gesendet
        //Serial.println("db4");
        
      }

}


void JoystickEvents::OnHatSwitch(uint8_t hat) {     //Hat Switche sind eigentlich nicht in Benutzung
    #if DEBUG_SERIAL
       Serial.print("Hat Switch: ");
       PrintHex<uint8_t > (hat, 0x80);
       Serial.println("");
    #endif
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {   //button loslassen löst keine besondere Aktion aus
  #if DEBUG_SERIAL
        Serial.print("Up: ");
        Serial.println(but_id, DEC);
  #endif
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {           //Funktioknstasten vom Gamepad abfragen
  sonderfunktionen[but_id] = !sonderfunktionen[but_id];     //Wenn Funktion aufgerufen wird, wird die ButtonId mit als Parameter übergeben
                                                            //im bool Array Sonderfunktionen wird dann der Wert invertiert.
  #if DEBUG_SERIAL
        Serial.print(runtime);
        Serial.print(" : Dn: ");
        Serial.println(but_id, DEC);
  #endif
}
