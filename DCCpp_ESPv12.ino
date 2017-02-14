
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

    No momentum is designed as yet, however if braking and accel are modelled in the decoder, they will be activated
    by functions, not as CV's. As such, any script in JMRI can reset momentum to off.

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
int pwr=0;                  // DCC++ track power
char instring [23];         // command string creation
byte ndx = 0;               // command string index
int DEFAULTCAB = 3;         // DEFINE DEFAULT CAB ADDRESS
int nCABAddr = 5565;        // CAB:  the short (1-127) or long (128-10293) address of the engine decoder, will store this in eeprom again later
int nReg = 0;               // throttle command from ndx[1]
int nSpeed = 0;             // throttle command from ndx[4]
byte nDir = 1;              // throttle command from ndx[5] (fwd)
byte fByte = 0;             // function command F0-F12
byte eByte = 0;             // function command F13-F28
uint32_t startTime = 0;     // timing function for F4 Firebox LED
uint32_t waitUntil = 0;     // timing function for F4 Firebox LED
uint32_t firingTime = 4000; // time firebox door is open
uint32_t shutTime = 15000;  // time firebox door is shut
int ledState = 0;           // ledState used to set F4 Firebox LED,
int ledState1 = 50;         // ledState used to set F4 Firebox LED firebox door almost closed
int ledState2 = 255;        // ledState used to set F4 Firebox LED during firing,
uint32_t uncouple = 1250;   // Servo move to 45 to open the coupler
uint32_t couple = 1500;     // Servo move to 90 to close the coupler
uint32_t FWD = 1375;        // Servo move to FWD gear position
uint32_t REV = 1625;        // Servo move to REV gear position
float motorSpeed = 0;       // actual speed for motor, float as accel/brake may create proportions
float prevSpeed = 0;        // previous speed setting, float as accel/brake may create proportions
int prevDir = 1;            // previous motor direction (fwd)
int speedMax = 255;         // max speed
int speedMin = 25;          // min start speed for the motor, currently just under stall. need to review this!
int brake = 0;              // maybe get this from some of the unused F commands
float accel = 0;            // maybe get this from different F commands


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
uint32_t nFrontPos = 1500;    //90 or 1500 is mid position, not up against a stop in the servo
uint32_t nRearPos = 1500;     //90 or 1500 is mid position, not up against a stop in the servo
uint32_t nReverPos = 1500;    //90 or 1500 is mid position, not up against a stop in the servo

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
    //Serial.println("F0-F4 Command");
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
      //Serial.println("F0 Enable LED");
    }
    else
    {
      digitalWrite(LED0, LOW);//LOW is off
      digitalWrite(LED1, LOW);//LOW is off
      //Serial.println("F0 Disable LED");
    }
    //F2 (Front Coupler) Command Process
    /*On - Control servo to decrease to 45 degrees
      Off– Control servo to return to 90 position*/
    if (bitRead(fByte, 1) && nFrontPos > uncouple)
    {
      for (nFrontPos = couple; nFrontPos >= uncouple; nFrontPos -= 1)
      {
        Front_CouplerServo.writeMicroseconds(nFrontPos);
        //Serial.print("F2 Front servo Pos:");
        //Serial.println(nFrontPos);
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    if (bitRead(fByte, 1) == 0 && nFrontPos < couple)
    {
      for (nFrontPos = uncouple; nFrontPos <= couple; nFrontPos += 1)
      {
        Front_CouplerServo.writeMicroseconds(nFrontPos);
        //Serial.print("F2 Front servo Pos:");
        //Serial.println(nFrontPos);
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    //F3 (Rear Coupler) Command Process
    /*  On - Control servo to decrease to 45 degrees
        Off – Control servo to return to 90 position*/
    if (bitRead(fByte, 2) && nRearPos > uncouple)
    {
      for (nRearPos = couple; nRearPos >= uncouple; nRearPos -= 1)
      {
        Rear_CouplerServo.writeMicroseconds(nRearPos);
        //Serial.print("F3 Rear servo Pos:");
        //Serial.println(nRearPos);
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    if (bitRead(fByte, 2) == 0 && nRearPos < couple)
    {
      for (nRearPos = uncouple; nRearPos <= couple; nRearPos += 1)
      {
        Rear_CouplerServo.writeMicroseconds(nRearPos);
        //Serial.print("F3 Rear servo Pos:");
        //Serial.println(nRearPos);
        yield();                   //get out of the way for WiFi connectivity
      }
    }
  }    //end Fl, F0-F4 Commands
  // this is a request for functions F9-F12, which in this case control a servo for Walschearts Valve Gear
  if ((fByte & 0xF0) == 160)
  {
    Serial.println("F9-F12 Command");

    // F9-F12 Command Process (Byte values for F9-F12 range 160 - 175)
    if (nReverPos > map(fByte, 160, 175, FWD, REV))
    {
      for ( ; nReverPos >= map(fByte, 160, 175, FWD, REV); nReverPos -= 1)
      {
        ReverserServo.writeMicroseconds(nReverPos);
        //Serial.print("F9-F12 Reverser Reduce cut-off:");
        //Serial.println(nReverPos);
        yield();                   //get out of the way for WiFi connectivity
      }
    }
    else if (nReverPos < map(fByte, 160, 175, FWD, REV))
    {
      for ( ; nReverPos <= map(fByte, 160, 175, FWD, REV); nReverPos += 1)
      {
        ReverserServo.writeMicroseconds(nReverPos);
        //Serial.print("F9-F12 Reverser Increase Cut-off:");
        //Serial.println(nReverPos);
        yield();                   //get out of the way for WiFi connectivity
      }
    }
  }//End F9-F12 Commands

  // this is a request for functions F13-F20, used to emulate Momentum without writing to eeprom. Still to be re-written
  if (fByte == 222)
  { // F13 - F20
    //Serial.println("F13-F20 Command");
    // F13 command check
    /* If F13 Momentum_Value received ie < f 03 222 BYTE2 >, map(F13,0,255,0,200), */
    if (bitRead(eByte, 0))
    {
      //Serial.println("F21-F28 not utilised");
      return;
    }
    //this is a request for functions F21-F28, used to emulate Braking without writing to eeprom Still to be re-written

    if (fByte == 223)
    { // F21 - F28
      //Serial.println("F21-F28 Command");
      // F21 command check
      /* If F21 Brake_Value received ie < f 03 223 BYTE2 >, map(F21,0,255,0,200), reduce motor speed using delay(Brake_Value) */
      if (bitRead(eByte, 0))
      {
        //Serial.println("F21-F28 not utilised");
        return;
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
      if (millis() - startTime >= waitUntil)
      {
        if (ledState == ledState2)
        {
          ledState = ledState1;
          startTime = millis();
          waitUntil = shutTime;    //on for 2 sec
          //Serial.println("Firebox door closed");
          yield();
        }
        else
        {
          ledState = ledState2;
          startTime = millis();
          waitUntil = firingTime;  //off for 10 sec
          //Serial.println("Firing");
          yield();
        }
      }
    }
    else {
      ledState = ledState1;
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
    //Serial.print("motor forward at ");
    //Serial.println(motorSpeed);
    yield();
  }
  if (motorSpeed > 0 && nDir == 0)                      //reverse
  {
    prevSpeed = motorSpeed;             //update the stored speed value
    analogWrite(inputA1, motorSpeed);            //don't run this pin
    analogWrite(inputA2, 0);   //run this pin at desired speed
    //Serial.print("motor reverse at ");
    //Serial.println(motorSpeed);
    yield();
  }
  if (motorSpeed == 0)                //NB nSpeed -1 is emergency stop
  {
    prevSpeed = motorSpeed;
    analogWrite(inputA1, 0);    //stop the motor
    analogWrite(inputA2, 0);    //stop the motor
    //Serial.println("motor stopped, speed 0");
    yield();
  }
}
//===================================================================================
void parseCmdString() {
  while (Client.available() > 0) {

    //uint32_t startTime = millis();      // check how long it takes to process commands
    char inChar = (char)Client.read();    //read data from client
    instring[ndx++] = inChar;             //add data to string
    if (inChar == '>') // '>' is the terminating character for a DCC++ command
    {
      instring[ndx] = '\0';               // terminating character added to the string
      ndx = 0;                            // reset ndx to 0 for the next string

      //Serial.println(instring);             //print the raw string for de-bugging




      yield();                              //get out of the way for WiFi connectivity
      char delimiters[] = "< >\r\n";        //working delimiters,
      char* valPosition = strtok(instring, delimiters);
      int tndx[] = {0, 0, 0, 0};            //4 x int.

      if (strchr(instring, 's'))//in DCCpp_Uno see Serial.command.cpp line 345
      {
        Client.print("<p");
        Client.print(pwr);
        Client.print(">");
        Client.print("<iDCC++ BASE STATION FOR ARDUINO ");
        Client.print("UNO");                  //should be ARDUINO_TYPE, hardcoded so as not to confuse JMRI
        Client.print(" / ");
        Client.print("ARDUINO MOTOR SHIELD"); //should be MOTOR_SHIELD_NAME, hardcoded so as not to confuse JMRI
        Client.print(": V-");
        Client.print("1.2.1+");               //should be VERSION, matched to the DCC++ base station version
        Client.print(" / ");
        Client.print(__DATE__);
        Client.print(" ");
        Client.print(__TIME__);
        Client.print(">");
        Client.print("<N1:");                 // Communication type, 0 = Serial, 1 = Ethernet
        Client.print(WiFi.localIP());
        Client.print(">");
        return;
      }
      else if (strchr(instring, '1'))         // DCC++ power on command <1>
      {
        pwr = 1;
        Client.print("<p");
        Client.print(pwr);
        Client.print(">");                 // DCC++ expected power on response
        return;
      }
      else if (strchr(instring, '0'))         // DCC++ power off command <0>
      {
        pwr = 0;
        nSpeed = 0;                           // set expected speed to nil
        motorSpeed = 0;                       //immediately set motor speed to nil (overrides any momentum)
        Client.print("<p");
        Client.print(pwr);
        Client.print(">");                 // DCC++ expected power off response
        return;
      }
      else if (strchr(instring, 'T'))         // DCC++ Turnouts command
      {
        Client.print("<X>");                  // DCC++ null response
        return;
      }
      else if (strchr(instring, 'S'))         // DCC++ Sensors command
      {
        Client.print("<X>");
        return;
      }
      else if (strchr(instring, 'Z'))         // DCC++ IO Pin command
      {
        Client.print("<X>");                  // DCC++ null response
        return;
      }
      else if (strchr(instring, 't'))         // DCC++ throttle command format <t reg CAB Speed Dir>
      {
        for (int i = 0; i < 5; i++)           // throttle commands should be 4 int long
        {
          tndx[i] = atoi(valPosition);        // convert to int
          valPosition = strtok(NULL, delimiters);
          if (i == 4 && tndx[2] == nCABAddr) {//when all fields are allocated and check command is for this CAB
            nReg = tndx[1];              // gets index 4
            nSpeed = tndx[3];            // gets index 4
            nDir = tndx[4];              // gets index 5

            if (nSpeed == -1) nSpeed = 0;   //emergency stop
            motorSpeed = map(nSpeed, 0, 126, 0, 1023);   //throttle commands are supposed to range from -1 to 126
            /*Throttle Commands*/
            runMotor();                                 //runMotor() here to get instant response to throttle commands rather than wait for loop() to do it again
            Client.print("<T ");                        //response to DCC++ Base station
            Client.print(tndx[1]);                      //response to DCC++ Base station
            Client.print(" ");                          //response to DCC++ Base station
            Client.print(nSpeed);                       //response to DCC++ Base station
            Client.print(" ");                          //response to DCC++ Base station
            Client.print(nDir);                         //response to DCC++ Base station
            Client.println(">");                        //response to DCC++ Base station
            /*Serial.print("Throttle Command");
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
              Serial.println(" ms");*/
            functions();                                //run through functions to ensure they update values, ie directionl lighting
            yield();
          }
        }       //for
      } //Throttle Command
      /*Function Commands*/
      else if (strchr(instring, 'f'))                   // DCC++ function command format <f CAB eByte fByte>
      {
        for (int i = 0; i < 3; i++)                     // function commands should be 3 to 4 int long
        {
          tndx[i] = atoi(valPosition);
          valPosition = strtok(NULL, delimiters);
          if (i == 2 && tndx[1] == nCABAddr) {          //stop iterating before possible fByte null field and check command is for this CAB
            fByte = tndx[2];                            // gets index 3
            /*Serial.print("Function Command");
              Serial.print(" nCABAddr ");
              Serial.print(nCABAddr);
              Serial.print(" fByte ");
              Serial.print(fByte);*/
            yield();
            if (tndx[2] > 221) {                 // eByte can be null, but is only invoked for fByte 222 or 223
              tndx[3] = atoi(valPosition);       // go on to obtain eByte
              valPosition = strtok(NULL, delimiters);
              eByte = tndx[3];              // gets index 4, may be NULL
              /*Serial.print(" eByte ");
                Serial.println(eByte);
                Serial.print("Time: ");
                Serial.print(millis() - startTime);
                Serial.println(" ms");*/
              functions();
            }
            else
            {
              /*Serial.println(" eByte NULL "); //stop the stack crashing by giving it something to do in the else case!
                Serial.print("Time: ");
                Serial.print(millis() - startTime);
                Serial.println(" ms");*/
              functions();
            }
          }
        }//for
      }//function command
      else
      {
        //
        Serial.println ("invalid Command");     //stop the stack crashing by giving it something to do in the else case
        return;
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
  digitalWrite(LED2, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Front_CouplerServo.attach(SERVOFRONT);
  Rear_CouplerServo.attach(SERVOREAR);
  ReverserServo.attach(SERVOREVERSER);

  pinMode(inputA1, OUTPUT);     //simple version of motor config
  analogWrite(inputA1, 0);      //set motor stop
  pinMode(inputA2, OUTPUT);    //simple version of motor config
  analogWrite(inputA2, 0);       //set motor stop

  Serial.begin(115200);            // configure serial interface
  Serial.println("\nSerial Begin 115200 Baud");
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


