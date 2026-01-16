#include <Arduino.h>

//SPI SLAVE (ARDUINO)
//SPI COMMUNICATION BETWEEN TWO ARDUINO 
//CIRCUIT DIGEST
//Pramoth.T

#include <SPI.h>
#include "PressureSensor.h"

#define DEBUG_MESSAGE 1
#define PRESSURESENSOR_ACTIVE 1 // must be 1 to be active -> if zero, the gripper will not close (movementSafe() will return false)

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

  #if (PRESSURESENSOR_ACTIVE)
    pressureSensor.SetupSensor();           // Setup scale
  #endif
  pinMode(A0, OUTPUT);
  
}

ISR (SPI_STC_vect)
{
  SPIRECEIVED = SPDR;       // Empfang vom Master
    

  SPDR = movementStatus;    // Status wird sofort als Antwort bereitgestellt!
}


void io() {
  beleuchtungAktiv = SPIRECEIVED & 0x01; // check if the esp wants some light

  #if (DEBUG_MESSAGE)
    Serial.print(SPIRECEIVED);
    Serial.println(beleuchtungAktiv);
  #endif

  digitalWrite(A0, beleuchtungAktiv); // do what the esp wants
}

void loop(){ 
  
  currentMillis = millis();
  #if (PRESSURESENSOR_ACTIVE)
    movementStatus = pressureSensor.MovementIsSafe() ? 0x01 : 0x00;
  #endif
  
  if(received) {                           //Logic to SET LED ON OR OFF depending upon the value recerived from master
    //Serial.print("I-> "); Serial.println(SPIRECEIVED);    
    received = false; 
  }

  if(currentMillis - lastPrint > 100) {
    lastPrint = currentMillis;
  }

  io();
  
  Serial.println(movementStatus);
}





