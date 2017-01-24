# DCCpp
 A first attempt at creating a WiFi onboard decoder for the DCC++ model railway base station
 
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
  ===============================.
