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

#define DEBUG_SERIAL 1
//#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
//JoystickEvents JoyEvents;
//JoystickReportParser Joy(&JoyEvents);


int stickLX, stickLY, stickRX, stickRY;
bool sonderfunktionen[12];


class EasySMX_Parser : public HIDReportParser {
public:
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) override {
    if (len < 8) return;   // Sicherheitscheck

    // Achsdaten
    stickLX = buf[3];
    stickLY = buf[4];
    stickRX = buf[5];
    stickRY = buf[6];

    // Buttonbits in Byte 1 (buf[1]), Bitnummerierung ab 1, LSB zuerst
    for(int i=1; i<=8; i++) {
      sonderfunktionen[i] = buf[0] & (1 << (i - 1));
    }

    #if DEBUG_SERIAL
      Serial.print(stickLX); Serial.print("/");
       Serial.print(stickLY);Serial.print("/");
       Serial.print(stickRX); Serial.print("/");
       Serial.print(stickRY); Serial.print("/");
      for(int i=1;i<=8;i++) Serial.print(sonderfunktionen[i]);
      Serial.println();
    #endif
  }
};

EasySMX_Parser parser;

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

unsigned long runtime, lastSend;

bool initWerte = true;
int statusLED = 5;

void setupRFM69() {
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(30);
  radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
  delay(1000);
}

void setup() {
  #if DEBUG_SERIAL
    Serial.begin(9600);
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
  if (!Hid.SetReportParser(0, &parser)) {
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
  }

   //wdt_enable(WDTO_2S);   // Watchdog auf 2 s stellen        
}

void sende(String sendeString) {
  static char sendbuffer[62];     //Puffer reservieren
  //static int sendlength = 0;      //Länge erst mal 0

  datenstring = "";
  datenstring.concat(sendeString);  
  datenstring.toCharArray(sendbuffer, 62);    //String in charArray umformen
  //Serial.println(sendbuffer);

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

         if(runtime > 1100) { 
            if(!Hid.isReady()) {          //HID gerät ist nicht bereit
              //Serial.println("db2");  
              // Werte der Sticks auf Mittelpunktschreiben
              stickLX = 127;
              stickLY = 128;
              stickRX = 127;
              stickRY = 128;


              /*digitalWrite(statusLED, HIGH);                                            //dann LED blinken lassen
              delay(200);
              digitalWrite(statusLED, LOW);
              delay(200);*/

            } else {
             //Serial.println("db3");
             analogWrite(statusLED, 90);                                           
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

    lastSend = runtime;
  }
        
  sende(controlMessage); // SENDEN der Werte via Funk 
  delay(2); // USBHost needs some time...
}