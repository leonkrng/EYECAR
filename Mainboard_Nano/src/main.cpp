#include <Arduino.h>

//SPI SLAVE (ARDUINO)
//SPI COMMUNICATION BETWEEN TWO ARDUINO 
//CIRCUIT DIGEST
//Pramoth.T

#include<SPI.h>

volatile boolean received;
volatile byte SPIRECEIVED,SPISEND;
volatile byte sendDataCounterIndex;
volatile byte dataZumESP[9] = {0,1,2,3,7,6,1,2,3};
volatile byte dataVomESP[20];

int buttonvalue;
int x;

unsigned long currentMillis, lastPrint;

bool blink;
bool beleuchtungAktiv;

void setup() {
  Serial.begin(115200);
  Serial.println("Nano ready");
  pinMode(MISO,OUTPUT);                   //Sets MISO as OUTPUT (Have to Send data to Master IN 

  SPCR |= _BV(SPE);                       //Turn on SPI in Slave Mode
  received = false;

  SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation

  pinMode(A0, OUTPUT);
  
}

ISR (SPI_STC_vect)                        //Inerrput routine function 
{
  //Serial.println("isr");
  SPIRECEIVED = SPDR;                     // Value received from master if store in variable slavereceived
  received = true;                        //Sets received as True 


  if(SPIRECEIVED == 0xFE) {
        sendDataCounterIndex = 0;
        //Serial.println("Received START packet");
  } else {
        //Serial.print("Got via SPI: "); Serial.print(SPIRECEIVED);
        if(sendDataCounterIndex <= 8 ) {  //Sicherheitsabfrage
          dataVomESP[sendDataCounterIndex] = SPIRECEIVED;
          SPISEND = dataZumESP[sendDataCounterIndex];
          //Serial.print("            returned: "); Serial.print(SPISEND);
          sendDataCounterIndex++;
        } else {
          //Serial.print("got more requests than i have data");
        }
        //Serial.println();
  }
    
    SPDR = SPISEND;                           //Sends the x value to master via SPDR 

}


void io() {
  digitalWrite(A0, beleuchtungAktiv);
}

void loop(){ 
  
  currentMillis = millis();
  
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
}




