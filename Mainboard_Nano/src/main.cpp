#include <Arduino.h>

//SPI SLAVE (ARDUINO)
//SPI COMMUNICATION BETWEEN TWO ARDUINO 
//CIRCUIT DIGEST
//Pramoth.T

#include <SPI.h>
#include "PressureSensor.h"

volatile boolean received;
volatile byte SPIRECEIVED,SPISEND;
volatile byte sendDataCounterIndex;
byte movementStatus;
volatile byte dataZumESP[9];
volatile byte dataVomESP[20];

unsigned long currentMillis, lastPrint;

bool blink;
bool beleuchtungAktiv;

const byte doutPinPressureSensor = 2;
const byte sckPinPressureSensor = 3;

PressureSensor pressureSensor(doutPinPressureSensor, sckPinPressureSensor); // communication with HX711
float measuredVal; // HX711 Value


void setup() {
  Serial.begin(115200);
  Serial.println("Nano ready");
  pinMode(MISO, OUTPUT);        // MISO muss OUTPUT sein für SPI

  SPCR |= _BV(SPE);             // SPI als Slave aktivieren
  SPI.attachInterrupt();        // SPI Interrupt einschalten

  // pressureSensor.SetupSensor(); // HX711 einrichten

  // Serial.begin(115200);
  // Serial.println("Nano ready");
  // pinMode(MISO,OUTPUT);                   //Sets MISO as OUTPUT (Have to Send data to Master IN 

  // SPCR |= _BV(SPE);                       //Turn on SPI in Slave Mode
  // received = false;

  // SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation

  pressureSensor.SetupSensor();           // Setup scale

  pinMode(A0, OUTPUT);
  
}

ISR (SPI_STC_vect)
{
  // SPIRECEIVED = SPDR;       // Empfang vom Master
  // received = true;

  // if (SPIRECEIVED == 0xFE) {
  //   sendDataCounterIndex = 0;
  //   SPISEND = dataZumESP[sendDataCounterIndex];  // Beim nächsten Master-Transfer gibt's das erste Datenbyte zurück!
  // } else {
  //   if(sendDataCounterIndex <= 8 ) {
  //     dataVomESP[sendDataCounterIndex] = SPIRECEIVED; // Master-Daten ablegen
  //     sendDataCounterIndex++;
  //     if(sendDataCounterIndex <= 8) {
  //       SPISEND = dataZumESP[sendDataCounterIndex];
  //     } else {
  //       SPISEND = 0; // Alternativ ein Fehlerwert oder wieder auf Null)
  //     }
  //   }
  // }

  // SPDR = SPISEND; // Direkt das Byte für DEN NÄCHSTEN Transfer vorbereiten!

  SPIRECEIVED = SPDR;       // Empfang vom Master
  
  // Statusbyte ermitteln (z.B. 0x01 = sicher, 0x00 = nicht sicher)
  

  SPDR = movementStatus;    // Status wird sofort als Antwort bereitgestellt!
}


void io() {
  digitalWrite(A0, beleuchtungAktiv);
}

void loop(){ 
  
  currentMillis = millis();
  movementStatus = pressureSensor.MovementIsSafe() ? 0x01 : 0x00;
  
  if(received) {                           //Logic to SET LED ON OR OFF depending upon the value recerived from master
    //Serial.print("I-> "); Serial.println(SPIRECEIVED);    
    received = false; 
  }

  if(currentMillis - lastPrint > 100) {
    lastPrint = currentMillis;

   /* Serial.print(dataVomESP[0]);
    Serial.print(dataVomESP[1]);
    Serial.print(dataVomESP[2]);
    Serial.print(dataVomESP[3]);
    Serial.print(dataVomESP[4]);
    Serial.println(dataVomESP[5]);
    */
  }

  if(dataVomESP[0] == 1) {
    beleuchtungAktiv = dataVomESP[1];
  }

  io();
  
  Serial.println(movementStatus);
}




