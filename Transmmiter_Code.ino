 #include <Wire.h>
#include "Arduino.h"
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
MPU6050 mpu;
//1 = for sign language
//2 = for home automation
//3 = for health monitoring
//4 = for light 1 ON
//5 = for light 1 OFF
//6 = for Light 2 ON
//7 = for Light 2 OFF

char temp ='0';
SoftwareSerial BTserial(3,2);// Blutooth (tx,rx)
SoftwareSerial bt(3,2);      /* (Rx,Tx) */ 

RF24 radio(4,5);                     // CE, CSN
byte addresses[][6] = {"1Node", "2Node"};    //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int transmit_bit [6]={0,0,0,0,0,0}; 

#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   

//  Variables
const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore.
                               // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
                               // Otherwise leave the default "550" value. 
                               
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"


#define adc1 A7
#define adc2 A6
#define adc3 A3
#define adc4 A2
#define adc5 A1
 

int flex=1,flex2=0,flex3=0,flex4=0,flex5=0;
int sum =1;
int exit_loop = 0;
int Mode = 0;
int ALL_ON=0;
int ALL_OFF=0;
void setup()
{
  
  Serial.begin(9600);                              //start serial
  bt.begin(9600);                               // start com. with bluetooth
  
  radio.begin();                                 //Starting the Wireless communication for nrf24L01
  radio.setPALevel(RF24_PA_MIN);                //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_2MBPS);               // Set the speed of the transmission to the quickest available
  
  radio.setChannel(124);                       // Use a channel unlikely to be used by Wifi, Microwave ovens etc 
  radio.openWritingPipe(addresses[1]);        // Open a writing pipe on radio

 

  // Configure the PulseSensor object, by assigning our variables to it. 
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);   

  // Double-check the "pulseSensor" object was created and "began" seeing a signal. 
   if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }
 

Serial.println("Initialize MPU6050");
while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
{
Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
delay(500);
}
checkSettings();
}
void checkSettings()
{
Serial.println();
Serial.print(" * Sleep Mode: ");
Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
Serial.print(" * Clock Source: ");
switch(mpu.getClockSource())
{
case MPU6050_CLOCK_KEEP_RESET: Serial.println("Stops the clock and keeps the timing generator in reset"); 
break;
case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference");
break;
case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); 
break;
case MPU6050_CLOCK_PLL_ZGYRO: Serial.println("PLL with Z axis gyroscope reference"); 
break;
case MPU6050_CLOCK_PLL_YGYRO: Serial.println("PLL with Y axis gyroscope reference");
break;
case MPU6050_CLOCK_PLL_XGYRO: Serial.println("PLL with X axis gyroscope reference"); 
break;
case MPU6050_CLOCK_INTERNAL_8MHZ: Serial.println("Internal 8MHz oscillator");
break;
}
Serial.print(" * Accelerometer offsets: ");
Serial.print(mpu.getAccelOffsetX());
Serial.print(" / ");
Serial.print(mpu.getAccelOffsetY());
Serial.print(" / ");
Serial.println(mpu.getAccelOffsetZ());
Serial.println();
}

void printfun(char cp) //to avoid printing repeating symbols
{
if(cp!=temp)
{
Serial.print(cp);
temp=cp;
}
}


void loop() 
 {
  
  

int flex1=analogRead(adc1);
//Serial.print("F1:");
BTserial.println(flex1); // Print on Bluetooth Term.
//Serial.println(flex1);
//delay(500); 
//Read Thumb and Print on serial mon.
 
flex2=analogRead(adc2);
//Serial.print("F2:"); 
//Serial.println(flex2);
//delay(500);
//Read Index finger and Print on serial mon.

flex3=analogRead(adc3);
//Serial.print("F3:"); 
//Serial.println(flex3);
//delay(500);
//Read Middle finger and Print on serial mon.

flex4=analogRead(adc4);
//Serial.print("F4:"); 
//Serial.println(flex4);
//delay(500);
//Read ring finger and Print on serial mon.

flex5=analogRead(adc5);
//Serial.print("F5:"); 
//Serial.println(flex5);
//Read pinky finger and Print on serial mon.

//Serial.println("------------------------------------");
 
     /* If data is available on serial port */
  
//Serial.println("-----BLUETOOTH AVAILABLE--------");

if (exit_loop == 120 || exit_loop == 121 || exit_loop == 122)
Mode = exit_loop;
else
Mode = bt.read();

/* Print character received on to the serial monitor */
//Serial.println(Mode);
delay(500);


    
while( Mode == 120)  //Select SignLanguage mode
{

Serial.println("-----MODE 1 EXECUTING--------");

exit_loop = bt.read();
if (exit_loop == 121 || exit_loop == 122 )                                          // to exit current loop id mode is switched
     break;
 

if ((( flex1>=500)&&( flex1<=600))&&((flex2>=500)&&(flex2<=600))&&((flex3>=500)&&(flex3<=600))&&((flex4>=500)&&(flex4<=600))&&((flex5>=500)&&(flex5<=600))){Serial.println('A'); delay(2000); bt.println('A');printfun('A')}
//haat dhilla
else if ((( flex1>=565)&&( flex1<=567))&&((flex2>=578)&&(flex2<=584))&&((flex3>=554)&&(flex3<=563))&&((flex4>=547)&&(flex4<=556))&&((flex5>=533)&&(flex5<=540))){Serial.println('B'); bt.println('B');printfun('B')}
 //haat dhilla

else if ((( flex1>=558)&&( flex1<=564))&&((flex2>=580)&&(flex2<=585))&&((flex3>=559)&&(flex3<=565))&&((flex4>=550)&&(flex4<=558))&&((flex5>=534)&&(flex5<=542))){Serial.println('C'); bt.println('C');printfun('C')}

 //haat dhilla
else if ((( flex1>=565)&&( flex1<=573))&&((flex2>=575)&&(flex2<=582))&&((flex3>=560)&&(flex3<=569))&&((flex4>=554)&&(flex4<=561))&&((flex5>=541)&&(flex5<=546))){Serial.println('D'); bt.println('D');printfun('D')}
 //haat tight 
}


while (Mode == 121)
{                                                                          // Select homeAutomation mode
Serial.println("-----MODE 2 EXECUTING--------");

exit_loop = bt.read();
if (exit_loop == 120 || exit_loop == 122 )                                // to exit current loop id mode is switched
    break;


    
// This is what we receive from the other device (the transmitter)

//int button_Light1=bt.read();    //to receive data from light 1 button
//int button_Light2=bt.read();    //to receive data from light 1 button

// Ensure we have stopped listening (even if we're not) or we won't be able to transmit
radio.stopListening();

Vector rawAccel = mpu.readRawAccel();
Vector normAccel = mpu.readNormalizeAccel(); 

if (!radio.write( &transmit_bit, sizeof(transmit_bit) )) {              // if no data is being trasmit, print the following message
    Serial.println("No acknowledgement of transmission");    
}

 
if(flex1<1019)
{ 
if(normAccel.XAxis>3){
Serial.println("LIGHT1_ON");
transmit_bit [0] = 1; transmit_bit [1] = 0; transmit_bit [2] = 0; transmit_bit [3] = 0;  // store 1 at "transmit_bit[0]" and store 0 in all others
radio.write(&transmit_bit, sizeof(transmit_bit));         //transmit the data stored in "transmit_bit"
Serial.print("Transmitting Data : ");                     // print the data stored in "transmit_bit"
Serial.println(transmit_bit [0]);                         // print the data stored in "transmit_bit[0]"
delay(1000);  
}

else if(normAccel.XAxis<-3){
Serial.println("LIGHT1_OFF");
transmit_bit [0] = 0; transmit_bit [1] = 2; transmit_bit [2] = 0; transmit_bit [3] = 0;  // store 2 at "transmit_bit[1]" and store 0 in all others
radio.write(&transmit_bit, sizeof(transmit_bit));
Serial.print("Transmitting Data : ");
Serial.println(transmit_bit [1]);               // print the data stored in "transmit_bit[1]"
delay(1000);
}

else if(normAccel.YAxis>3){
Serial.println("LIGHT2_ON");
 transmit_bit [0] = 0; transmit_bit [1] = 0; transmit_bit [2] = 3; transmit_bit [3] = 0; // store 3 at "transmit_bit[2]" and store 0 in all others
radio.write(&transmit_bit, sizeof(transmit_bit));
Serial.print("Transmitting Data : ");
Serial.println(transmit_bit [2]);             // print the data stored in "transmit_bit[2]"
delay(1000);
}

else if(normAccel.YAxis<-3){
Serial.println("LIGHT2_OFF");
transmit_bit [0] = 0; transmit_bit [1] = 0; transmit_bit [2] = 0; transmit_bit [3] = 4; // store 4 at "transmit_bit[3]" and store 0 in all others
radio.write(&transmit_bit, sizeof(transmit_bit));
Serial.print("Transmitting Data : ");
Serial.println(transmit_bit [3]);           // print the data stored in "transmit_bit[3]"
delay(1000);
}

ALL_ON = bt.read();// ALL LIGHTS ON BUTTON
Serial.println (ALL_ON);

 if ( ALL_ON == 66){
transmit_bit [0] = 0; transmit_bit [1] = 0; transmit_bit [2] = 0; transmit_bit [3] = 0; transmit_bit [4] = 5; transmit_bit [5] = 0;
radio.write(&transmit_bit, sizeof(transmit_bit));         //transmit the data stored in "transmit_bit"
Serial.print("Transmitting Data : ");                     // print the data stored in "transmit_bit"
Serial.println(transmit_bit [4]);                         // print the data stored in "transmit_bit[0]"
delay(1000);
}


 ALL_OFF = bt.read();    // ALL LIGHTS OFF BUTTON
Serial.println (ALL_OFF);


 if ( ALL_OFF == 67){
transmit_bit [0] = 0; transmit_bit [1] = 0; transmit_bit [2] = 0; transmit_bit [3] = 0; transmit_bit [4] = 0; transmit_bit [5] = 6;
radio.write(&transmit_bit, sizeof(transmit_bit));         //transmit the data stored in "transmit_bit"
Serial.print("Transmitting Data : ");                     // print the data stored in "transmit_bit"
Serial.println(transmit_bit [5]);                         // print the data stored in "transmit_bit[0]"
delay(1000);
}



} //end of  if(flex1<1019)

 
} //end of  mode 2


while (Mode == 122){        // Select HealthMonitoring mode
Serial.println("-----MODE 3 EXECUTING--------");

exit_loop = bt.read();
if (exit_loop == 120 || exit_loop == 121 )                                // to exit current loop id mode is switched
    break;

    

 int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
                                               // "myBPM" hold this BPM value now. 

if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened". 
 Serial.println("  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
 Serial.print("BPM: ");                        // Print phrase "BPM: " 
 Serial.println(myBPM);                        // Print the value inside of myBPM. 
 bt.println(myBPM);
}

  delay(1000);                    // considered best practice in a simple sketch.





}

 
} //end of void loop()
