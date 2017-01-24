
/*
    DCCpp_ESPv1.ino

    This Sketch is designed to operate as a WiFi onboard decoder for
    a model railway. It will accept throttle and some function commands in 
    accordance with DCC++, as per https://github.com/DccPlusPlus/BaseStation/wiki/Commands-for-DCCpp-BaseStation

    Hardware was a 
      *WeMos D1 Mini 
      *Pololu DRV8833 Dual Motor Driver Carrier,
      *Pololu 5V stepup voltage U3V12F5 attached to 5V and GND,
      *Pololu 9V stepup voltage U3V12F9 attached to Vin and GND on the DRV8833,
      *Slaters RG7 40:1 gearmotor (Mashima 1844 + gearbox)
      *9g micro servo x 3
      *LEDs
    
    It is designed for a steam locomotive with external valve gear, with the following applicable;
      *F0 - enable lights, including directional lighting,
      *F1 - not used
      *F2 - front coupler servo
      *F3 - rear coupler servo
      *F4 - firebox, dim when firebox door shut, opened periodically under motion
      *F5 - F8 - not used
      *F9 - F12 - valve gear reverser servo, anticipate control from a custom throttle, values 160 - 175
      *F13 - F28 - not used

    No momentum was designed in for motor control, as offstage auto operation by JMRI is anticipated requiring 1:1 control. 
    Momentum and braking will come from a custom JMRI hardware throttle.

    It would be possible to enable OTA and CV command writing to Eeprom, however as I was happy programming over serial, I haven't done this yet.

    Copyright (c) 2017 Simon Mitchell.

    Licensed under the EUPL, Version 1.1 only (the "Licence");
    You may not use this work except in compliance with the Licence.
    You may obtain a copy of the Licence at:

      http://joinup.ec.europa.eu/software/page/eupl

    Unless required by applicable law or agreed to in writing, software
    distributed under the Licence is distributed on an "AS IS" basis,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

    See the Licence for the specific language governing permissions and
    limitations under the Licence.

  ===============================
    This licence is similar to the GNU General Public License v.2,  the
      Eclipse Public License v. 1.0 etc.
  ===============================

*/
#include <ESP8266WiFi.h>      //specific to the ESP8266 board, eg WeMos D1 Mini
#include <Servo.h>            //std Arduino library for servos

//===================================================================================
// WiFi Communication
IPAddress ip(192, 168, 54, 65);                     //DEFINE STATIC IP ADDRESS *OR* COMMENT OUT TO USE DHCP
IPAddress gateway(192, 168, 54, 1);                 //SET GATEWAY TO MATCH YOUR NETWORK
IPAddress subnet(255, 255, 255, 0);                 //SET SUBNET MASK TO MATCH YOUR NETWORK

#define ETHERNET_PORT 2560                          // DEFINE PORT TO USE FOR ETHERNET COMMUNICATIONS INTERFACE
//#define MAC_ADDRESS {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF }// DEFINE MAC ADDRESS ARRAY FOR ETHERNET COMMUNICATIONS INTERFACE

const char* ssid     = "XXXXXXXXXX";       // Wi-Fi login - insert your own details here
const char* password = "YYYYYYYYYY";       // Wi-Fi password - insert your own details here

/* WiFi Socket Server and Client Class instance */
WiFiServer server(ETHERNET_PORT);                   // Create and instance of an WIFIServer
WiFiClient Client;                                  // Create instance of an WIFIClient.

//===================================================================================
//DCC++ Communications and Configurations
char instring [23];         // command string creation
byte ndx = 0;               // command string index
int DEFAULTCAB = 3;         // DEFINE DEFAULT CAB ADDRESS
int nCABAddr = 5565;        // CAB:  the short (1-127) or long (128-10293) address of the engine decoder, will store this in eeprom again later
int nSpeed = 0;             // throttle command from ndx[4]
byte nDir = 1;              // throttle command from ndx[5] (fwd)
byte fByte = 0;             // function command F0-F12
byte eByte = 0;             // function command F13-F28
uint32_t startTime = 0;     // timing function for F4 Firebox LED
uint32_t waitUntil = 0;     // timing function for F4 Firebox LED
int ledState = 0;           // ledState used to set F4 Firebox LED, HIGH is actually off
int motorSpeed = 0;         // actual speed for motor
int prevSpeed = 0;          // previous speed setting
int prevDir = 1;            // previous motor direction (fwd)


//===================================================================================
//PIN Definitions
#define ANALOGINPUT       A0 //analog input, not used yet
#define UNUSED            D0  //GPIO16     IO (no PWM)                        Unused
#define SERVOFRONT        D1  //GPIO5      IO, SCL I2C pin                    Front Coupler Servo
#define SERVOREAR         D2  //GPIO4      IO, SDA I2C pin serial on startup  Rear Coupler Servo
#define SERVOREVERSER     D3  //GPIO0      IO, 10k Pull-up                    Reverser Servo for valve gear
#define LED0              D4  //GPIO2      IO, 10k Pull-up, BUILTIN_LED       Front lamp  
#define LED1              D5  //GPIO14     IO, SCK                            Rear lamp
#define LED2              D6  //GPIO12     IO, MISO                           Firebox Glow                     
#define inputA2           D7  //GPIO13     IO, MOSI                           DRV8833 input pin Ain2 must be a PWM pin
#define inputA1           D8  //GPIO15     IO, 10k Pull-down, SS              DRV8833 input pin Ain1 must be a PWM pin
//#define inputB1                   //                                      DRV8833 input pin Bin1 must be a PWM pin
//#define inputB2                   //                                      DRV8833 input pin Bin2 must be a PWM pin

//===================================================================================
// Servo Motor control instance
Servo Front_CouplerServo;   // (Front Coupler)
Servo Rear_CouplerServo;    // (Rear Coupler)
Servo ReverserServo;        // (Reverser)

// Initial Positions of Servo Motors
unsigned short nFrontPos = 90;    //90 is mid position, not up against a stop in the servo
unsigned short nRearPos = 90;     //90 is mid position, not up against a stop in the servo
unsigned short nReverPos = 90;    //90 is mid position, not up against a stop in the servo

//===================================================================================
void functions()
{
  /*
        turns on and off engine decoder functions F0-F28 (F0 is sometimes called FL)
        NOTE: setting requests transmitted directly to mobile engine decoder --- current state of engine functions is not stored by this program

        CAB:  the short (1-127) or long (128-10293) address of the engine decoder

        To set functions F0-F4 on (=1) or off (=0):

        BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
        BYTE2:  omitted

        To set functions F5-F8 on (=1) or off (=0):

        BYTE1:  176 + F5*1 + F6*2 + F7*4 + F8*8
        BYTE2:  omitted

        To set functions F9-F12 on (=1) or off (=0):

        BYTE1:  160 + F9*1 +F10*2 + F11*4 + F12*8
        BYTE2:  omitted

        To set functions F13-F20 on (=1) or off (=0):

        BYTE1: 222
        BYTE2: F13*1 + F14*2 + F15*4 + F16*8 + F17*16 + F18*32 + F19*64 + F20*128

        To set functions F21-F28 on (=1) of off (=0):

        BYTE1: 223
        BYTE2: F21*1 + F22*2 + F23*4 + F24*8 + F25*16 + F26*32 + F27*64 + F28*128

        returns: NONE
  */
  // this is a request for functions FL,F1-F8
  if ((fByte & 0xE0) == 128)
  { // 128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
    Serial.println("F0-F4 Command");
    /*
        If F0 (Lights) is on
        Enable LED if DIRECTION = 1 (Forward)
        Enable LED if DIRECTION = 0 (Reverse)
        Enable LED and if SPEED>0, then brighten for 20s every 120s (firebox)
    */
    // F0 Command process
    if (bitRead(fByte, 4))
    {
      if (nDir == 1) { //Forward direction
        digitalWrite(LED0, LOW); //LOW is off
        digitalWrite(LED1, HIGH);//HIGH is on
      }
      if (nDir == 0) { //Reverse Direction
        digitalWrite(LED0, HIGH);//HIGH is on
        digitalWrite(LED1, LOW);//LOW is off
      }
      Serial.println("F0 Enable LED");
    }
    else
    {
      digitalWrite(LED0, LOW);//LOW is off
      digitalWrite(LED1, LOW);//LOW is off
      Serial.println("F0 Disable LED");
    }
    //F2 (Front Coupler) Command Process
    /*On - Control servo to decrease to 45 degrees
      Off– Control servo to return to 90 position*/
    if (bitRead(fByte, 1) && nFrontPos > 45)
    {
      for (nFrontPos = 90; nFrontPos >= 45; nFrontPos -= 1)
      {
        Front_CouplerServo.write(nFrontPos);
        Serial.print("F2 Front servo Pos:");
        Serial.println(nFrontPos);
        millis() + 30;
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    if (bitRead(fByte, 1) == 0 && nFrontPos < 90)
    {
      for (nFrontPos = 45; nFrontPos <= 90; nFrontPos += 1)
      {
        Front_CouplerServo.write(nFrontPos);
        Serial.print("F2 Front servo Pos:");
        Serial.println(nFrontPos);
        millis() + 30;
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    //F3 (Rear Coupler) Command Process
    /*  On - Control servo to decrease to 45 degrees
        Off – Control servo to return to 90 position*/
    if (bitRead(fByte, 2) && nRearPos > 45)
    {
      for (nRearPos = 90; nRearPos >= 45; nRearPos -= 1)
      {
        Rear_CouplerServo.write(nRearPos);
        Serial.print("F3 Rear servo Pos:");
        Serial.println(nRearPos);
        millis() + 30;
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    if (bitRead(fByte, 2) == 0 && nRearPos < 90)
    {
      for (nRearPos = 45; nRearPos <= 90; nRearPos += 1)
      {
        Rear_CouplerServo.write(nRearPos);
        Serial.print("F3 Rear servo Pos:");
        Serial.println(nRearPos);
        millis() + 30;
        yield();                   //get out of the way for WiFi connectivity
      }
    }
  }//end Fl, F0-F4 Commands
  // this is a request for functions F9-F12, which in this case control a servo for Walschearts Valve Gear
  if ((fByte & 0xF0) == 160)
  {
    Serial.println("F9-F12 Command");

    // F9-F12 Command Process (reverse gear = 120)
    if (nReverPos > map(fByte, 160, 175, 62, 122))
    {
      for ( ; nReverPos >= map(fByte, 160, 175, 62, 122); nReverPos -= 1)
      {
        ReverserServo.write(nReverPos);
        Serial.print("F9-F12 Reverser Reduce cut-off:");
        Serial.println(nReverPos);
        millis() + 60;
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    else if (nReverPos < map(fByte, 160, 175, 62, 122))
    {
      for ( ; nReverPos <= map(fByte, 160, 175, 62, 122); nReverPos += 1)
      {
        ReverserServo.write(nReverPos);
        Serial.print("F9-F12 Reverser Increase Cut-off:");
        Serial.println(nReverPos);
        millis() + 60;
        yield();                   //get out of the way for WiFi connectivity
      }
    }
  }//End F9-F12 Commands

  // this is a request for functions F13-F20, used to emulate Momentum without writing to eeprom. Still to be re-written
  if (fByte == 222)
  { // F13 - F20
    Serial.println("F13-F20 Command");
    // F13 command check
    /* If F13 Momentum_Value received ie < f 03 222 BYTE2 >, map(F13,0,255,0,200), */
    if (bitRead(eByte, 0))
    {
      Serial.print("F21-F28 not utilised");
    }
    //this is a request for functions F21-F28, used to emulate Braking without writing to eeprom Still to be re-written

    if (fByte == 223)
    { // F21 - F28
      Serial.println("F21-F28 Command");
      // F21 command check
      /* If F21 Brake_Value received ie < f 03 223 BYTE2 >, map(F21,0,255,0,200), reduce motor speed using delay(Brake_Value) */
      if (bitRead(eByte, 0))
      {
        Serial.print("F21-F28 not utilised");
      }
    }
  }
}
//===================================================================================
void firebox() {
  //F4 (Firebox) Command Process, couldn't get this to work in void functions()
  /*  treated as stand alone as this is visible at all times*/
  if (bitRead(fByte, 3)) { // Firebox, fire for 20s every 2 mins
    if (nSpeed > 0) {
      if (millis() - startTime >= waitUntil) {
        if (ledState == 255) {
          ledState = 50;
          startTime = millis();
          waitUntil = 15000;    //on for 2 sec
          Serial.println("Firebox door closed");
          yield();
        }
        else
        {
          ledState = 255;
          startTime = millis();
          waitUntil = 4000;  //off for 10 sec
          Serial.println("Firing");
          yield();
        }
      }
    }
    else {
      ledState = 50;
    }
  }
  else
  {
    ledState = 0;     //LED off when F4 is off
  }
  analogWrite(LED2, ledState);
}
//===================================================================================
void runMotor()  {
  if (nDir != prevDir)
  {
    prevDir = nDir;              //update direction
    analogWrite(inputA1, 0);    //stop the motor, no crash reversing thanks!
    analogWrite(inputA2, 0);    //stop the motor, no crash reversing thanks!
    //Serial.println("motor stopped on change of direction");
    yield();
  }
  if (motorSpeed == prevSpeed) {
    return; //leave it running as it is
  }
  if (motorSpeed > 0 && nDir == 1)                      //forward
  {
    prevSpeed = motorSpeed;             //update the stored speed value
    analogWrite(inputA1, 0);            //don't run this pin
    analogWrite(inputA2, motorSpeed);   //run this pin at desired speed
    Serial.print("motor forward at ");
    Serial.println(motorSpeed);
    yield();
  }
  if (motorSpeed > 0 && nDir == 0)                      //reverse
  {
    prevSpeed = motorSpeed;             //update the stored speed value
    analogWrite(inputA1, motorSpeed);            //don't run this pin
    analogWrite(inputA2, 0);   //run this pin at desired speed
    Serial.print("motor reverse at ");
    Serial.println(motorSpeed);
    yield();
  }
  if (motorSpeed == 0)                //NB nSpeed -1 is emergency stop
  {
    prevSpeed = motorSpeed;
    analogWrite(inputA1, 0);    //stop the motor
    analogWrite(inputA2, 0);    //stop the motor
    Serial.println("motor stopped, speed 0");
    yield();
  }
}
//===================================================================================
void parseCmdString() {
  while (Client.available() > 0) {

    uint32_t startTime = millis();
    char inChar = (char)Client.read();    //read data from client
    instring[ndx++] = inChar;             //add data to string
    if (inChar == '>') {                  // '>' is the terminating character for a DCC++ command
      instring[ndx] = '\0';               // terminating character added to the string
      ndx = 0;                            // reset ndx to 0 for the next string

      Serial.println(instring);             //print the raw string for de-bugging
      yield();                              //get out of the way for WiFi connectivity
      char delimiters[] = "< >\r\n";        //working delimiters,
      char* valPosition = strtok(instring, delimiters);
      int tndx[] = {0, 0, 0, 0};            //4 x int.
      /*Throttle Commands*/
      if (strchr(instring, 't')) {      // throttle command format <t reg CAB Speed Dir>
        for (int i = 0; i < 5; i++) {   // throttle commands should be 4 int long
          tndx[i] = atoi(valPosition);  // convert to int
          valPosition = strtok(NULL, delimiters);
          if (i == 4 && tndx[2] == nCABAddr) {//when all fields are allocated and check command is for this CAB
            nSpeed = tndx[3];            // gets index 4
            nDir = tndx[4];              // gets index 5
            if (nSpeed == -1) nSpeed = 0;   //emergency stop
            motorSpeed = map(nSpeed, 0, 126, 0, 1023);   //throttle commands are supposed to range from -1 to 126
            functions();                                //run through functions to ensure they update values, ie directionl lighting
            Serial.print("Throttle Command");
            Serial.print(" nCABAddr ");
            Serial.print(nCABAddr);
            Serial.print(" nSpeed ");
            Serial.print(nSpeed);
            Serial.print("motorSpeed ");
            Serial.print(motorSpeed);
            Serial.print(" nDir ");
            Serial.println(nDir);
            Serial.print("Time: ");
            Serial.print(millis() - startTime);
            Serial.println(" ms");
            yield();
          }
        }       //for
      } //Throttle Command
      /*Function Commands*/
      else if (strchr(instring, 'f')) {      // function command format <f CAB eByte fByte>
        for (int i = 0; i < 3; i++) {        // function commands should be 3 to 4 int long
          tndx[i] = atoi(valPosition);
          valPosition = strtok(NULL, delimiters);
          if (i == 2 && tndx[1] == nCABAddr) { //stop iterating before possible fByte null field and check command is for this CAB
            fByte = tndx[2];            // gets index 3
            Serial.print("Function Command");
            Serial.print(" nCABAddr ");
            Serial.print(nCABAddr);
            Serial.print(" fByte ");
            Serial.print(fByte);
            yield();
            if (tndx[2] > 221) {                 // fByte can be null, but is only invoked for eByte 222 or 223
              tndx[3] = atoi(valPosition);       // go on to obtain fByte
              valPosition = strtok(NULL, delimiters);
              eByte = tndx[3];              // gets index 4, may be NULL
              Serial.print(" eByte ");
              Serial.println(eByte);
              Serial.print("Time: ");
              Serial.print(millis() - startTime);
              Serial.println(" ms");
              functions();
            }
            else
            {
              Serial.println(" eByte NULL "); //stop the stack crashing by giving it something to do in the else case!
              Serial.print("Time: ");
              Serial.print(millis() - startTime);
              Serial.println(" ms");
              functions();
            }
          }
        }//for
      }//function command
      else
      {
        Serial.print ("invalid Command");     //stop the stack crashing by giving it something to do in the else case
      }
    }//string complete
  }//serial.available;
}//parseCmdString();
//===================================================================================
void serialCommandProcess() {
  //check if there are any new clients
  if (server.hasClient())
  {
    if (!Client || !Client.connected())
    {
      if (Client) Client.stop();
      Client = server.available();
      Serial.println("New client Connected");
    }
    else
    {
      WiFiClient serverClient = server.available();
      serverClient.stop();
    }
  }//server.hasClient()
} // SerialCommandProcess()

//===================================================================================
void setup() {

  // Pin configuration
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);

  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);

  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);  //see also 'ledState' in global decs

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Front_CouplerServo.attach(SERVOFRONT);
  Rear_CouplerServo.attach(SERVOREAR);
  ReverserServo.attach(SERVOREVERSER);

  pinMode(inputA1, OUTPUT);     //simple version of motor config
  analogWrite(inputA1, 0);      //set motor stop
  pinMode(inputA2, OUTPUT);    //simple version of motor config
  analogWrite(inputA2, 0);       //set motor stop

  Serial.begin(19200);            // configure serial interface
  Serial.println("\nSerial Begin 19200 Baud");
  Serial.flush();
  // WIFI Begin
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  uint8_t idx = 0;
  // If WIFI is not connected, 10 sec delay.
  while (WiFi.status() != WL_CONNECTED && idx++ < 20) delay(500);
  if (idx == 21)
  {
    Serial.print("Could not connect to ");
    Serial.println(ssid);
    while (1)
    { // When Wifi connection error, Led flickers per 0.5sec
      digitalWrite(LED_BUILTIN, LOW);
      millis() + 250;
      digitalWrite(LED_BUILTIN, HIGH);
      millis() + 250;
    }
  }

  // Server socket start.
  server.begin();
  server.setNoDelay(true);

  // Report Wi-Fi connection Status
  Serial.println("DCC++ ESP8266 Mobile Decoder is Ready!");
  Serial.print("Local IP is ");
  Serial.println(WiFi.localIP());
  Serial.print("Current CAB is ");
  Serial.println(nCABAddr);



}// end setup
//===================================================================================
void loop()
{
  serialCommandProcess();                 // check for new serial commands
  parseCmdString();                       // check for new serial commands
  runMotor();                             // wasn't working in the middle of parseCmdString()
  firebox();                              // wasn't working in the middle of 'functions()'
}




