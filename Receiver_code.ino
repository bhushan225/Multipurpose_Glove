#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>
#include <Relay.h>
 
 #define relay_pin1 7 //Defining the pin 7 of the Arduino for the 4 relay module
 #define relay_pin2 2 //Defining the pin 2 of the Arduino for the 4 relay module
 

RF24 radio(4,5);


int transmit_bit [6]= {0,0,0,0,0,0};
byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(9600);
  Serial.println("THIS IS THE RECEIVER CODE - YOU NEED THE OTHER ARDUINO TO TRANSMIT");
  pinMode(relay_pin1, OUTPUT);
  pinMode(relay_pin2, OUTPUT);

 
  radio.begin();

  radio.setPALevel(RF24_PA_MIN);

  radio.setDataRate(RF24_2MBPS);

  radio.setChannel(124);

  radio.openReadingPipe(1, addresses[1]);

  radio.startListening();
  digitalWrite(relay_pin1,HIGH);
  digitalWrite(relay_pin2,HIGH);
}

void loop() {
  
  unsigned char data1;
  unsigned char data2;
  unsigned char data3;
  unsigned char data4;
  unsigned char data5;
  unsigned char data6;
  


  if (radio.available()) {

    while (radio.available()) {
      radio.read( &transmit_bit, sizeof(transmit_bit));
      data1 = transmit_bit [0];
      data2 = transmit_bit [1];
      data3 = transmit_bit [2];
      data4 = transmit_bit [3];
      
      data5 = transmit_bit [4];
      Serial.println("data5:");
      Serial.println(data5);
      data6 = transmit_bit [5];
      Serial.println("data6:");
      Serial.println(data6);
    }
    
   radio.startListening();
    
    
    
 
   if (data1 == 1) {
    digitalWrite(relay_pin1,LOW);    //LOW = HIGH IN THEWHOLE CODE WHERE EVER IT IS "LOW" THAT MEANS HIGH AND VICE VERSA
    Serial.println("LIGHT1_ON");
    Serial.println(data1);
    delay(200);
   }
   
     
    else if (data2 == 2) {
    digitalWrite(relay_pin1,HIGH);
    Serial.println("LIGHT1_OFF");
    Serial.println(data2);
    delay(200);
   }

    
    else if (data3 == 3) {
    digitalWrite(relay_pin2, LOW);
    Serial.println("LIGHT2_ON");
    Serial.println(data3);
    delay(200);
    
   }
    
    else if (data4 == 4) {
    digitalWrite(relay_pin2,HIGH);
    Serial.println("LIGHT2_OFF");
    Serial.println(data4);
    delay(200);
   }
  
   else if (data5 == 5) {
    digitalWrite(relay_pin1,LOW);
    digitalWrite(relay_pin2,LOW);
    Serial.println("LIGHTS_ON");
    Serial.println(data5);
    delay(200);
   }
   
   else if (data6 == 6) {
    digitalWrite(relay_pin1,HIGH);
    digitalWrite(relay_pin2,HIGH);
    Serial.println("LIGHTS_OFF");
    Serial.println(data6);
    delay(200);
   }
   }
//    Serial.println("Sent response ");
//    Serial.println(data1);
//    Serial.println("");
//    Serial.println(data2);
//    Serial.println("");
//    Serial.println(data3);
//    Serial.println("");
//    Serial.println(data4);

  }
 
