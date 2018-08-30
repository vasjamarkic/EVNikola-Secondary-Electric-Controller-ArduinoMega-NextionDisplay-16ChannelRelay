/*TRUE last version for NEXTION display with ARDUINO mega2560, connection working 
26.6.2018 (Ivan's Arduino MEGA 2560 :)
linked with HMI file "NEXTION_2_4_ver1_INVERTER"
in "nexConf.h" enable debug and must be set to Serial2 ...
Wiring the two data wires:            Wiring also the I2C protocol for SOC values from arduino NANO
NEXTION Rx --> Tx2 MEGA               A4 NANO --> 20 MEGA
NEXTION Tx --> Rx2 MEGA               A5 NANO --> 21 MEGA
The input signals are switching icons on display - I used ProgressBAR for that trick!
Make sure you import the ITEADlib library!!
The Relay module has to be connected to power a 10 seconds after the key power the Arduino & Nextion to prevent switching relays ON!
input tester: 
SWITCH DIP                           BUTTONS
[1 ]  ???
[2 ] 26 - parklight         [1] 30 - STOP
[3 ] 28 - daylight          [2] 27 -  dooropen switch ok
[4 ] 32 - long light        [3] 31 - right
[5 ] 24 - fog light         [4] 25 - left 
[6 ] 34 - frost             [1] 35 - HORN        open door switch
[7 ] 36 - emergency         [2] ??  -  ???           RIGHT
[8 ] 22 - handbrake
]9 ] 37 - backward drive light
[10] 23 - cabin switch light  ???
30.8.2018 changed the obsolete SOC to DIM trimmer use for dimming the nextion display, 
uses progress bar j13.
Author: Vasja Markič           
*/
#include <Wire.h>
#include <doxygen.h>
//#include <NexButton.h>
#include <NexConfig.h>
//#include <NexCrop.h>
//#include <NexGauge.h>
//#include <NexHardware.h>
//#include <NexHotspot.h>
//#include <NexObject.h>
//#include <NexPage.h>
#include <NexPicture.h>
#include <NexProgressBar.h>
//#include <NexSlider.h>
//#include <NexText.h>
#include <Nextion.h>
#include <NexTouch.h>
//#include <NexWaveform.h>
#include "Nextion.h"

// constants won't change. They're used here to set pin numbers, these are OUTPUTS:
const int ParkLight =  53;           // na relay 1
const int DayLight = 52;              // na relay 2
const int LongLight = 51;               // na relay 3
const int TurnLeft = 50;           // na relay 4 --> switching relay
const int TurnRight = 49;          // na relay 5 --> switcging relay CF14 flasher unit
const int StopLight = 48;            // na relay 6
const int Horn = 47;                // na relay 7
const int DefrostWindow = 42;      // relay 8 
const int BackwardLight = 45;        // relay 9
const int CabinLight = 44;         // relay 10
const int FogLight = 43;           // relay 11
const int EmergenceLight = 46;          // na relay 12
const int WindowWater = 41;        // relay 13
const int Wiper1Alternate = 38;     // relay 14 --> on auto wiper intermittent relay, timer
const int Wiper1 = 39;         // relay 15 --> on auto wiper intermittent relay
const int Wiper2 = 40;         // relay 16, direct to wiper motor

// INPUTS:
const int ParkLight_Switch = 26;    // with contact ??
const int DayLight_Switch = 28;      
const int LongLight_Switch = 32;       
const int TurnLeft_Switch = 25;   
const int TurnRight_Switch = 31;  
const int StopLight_switch = 30;    // switch on the brake foot
const int EmergenceLight_Switch = 36; 
const int Horn_Switch = 35;        // wheel drive button
const int DefrostWindow_Switch = 34; 
const int BackwardLight_Switch = 37;     // two-pole switch to change the drive direction!
const int CabinLight_Switch = 23;  // switch for light the cabin
const int DoorOpen_Switch = 27;   // micro switch on doors
const int Handbrake_Switch = 22;       // hadnbrake microswitch
const int FogLight_Switch = 24;   //
// 31 REZERVA
const int WindowWater_Switch = 12;
const int Wiper1_Switch = 10;
const int Wiper2_Switch = 11;
const int Wiper1Alternate_Switch = 9;
const int ChargeMode_input = 8;           // microswitch detects when AC plug cable is IN
const int Battery12V_input = A0;          // analog input, resistor divider 1:15 ratio with capacitor
const int DIM_input = A1;                 // analog input from BMS

// ventilation via PWM power module

// display SOC - using existing fuel gauge (0 ÷ 6 V) reading analog IN and drive with digital PWM and MOSFET BS 170 ali BS 107.
// display temp. motor (0 ÷ 6 V)

// variables will change - HIGH state relay OFF:
bool sign_rocna_ON = true;
bool ParkLight_Switch_ON = true;
bool DayLight_Switch_ON = true;
bool LongLight_Switch_ON = true;
bool TurnLeft_Switch_ON = true;
bool TurnRight_Switch_ON = true;
bool EmergenceLight_Switch_ON = true;
bool StopLight_switch_ON = true;
bool Horn_Switch_ON = true;
bool DefrostWindow_Switch_ON = true;
bool BackwardLight_Switch_ON = true;
bool CabinLight_Switch_ON = true;
bool DoorOpen_Switch_ON = true;
bool Handbrake_Switch_ON =true;
bool WindowWater_Button_ON = true;
bool Wiper1Alternate_Switch_ON = true;
bool Wiper1_Switch_ON = true;
bool Wiper2_Switch_ON = true;
bool FogLight_Switch_ON = true;
bool ChargeMode_ON = false;       // if we plug the AC charging
long DIM_Value = 0;      // initial value of the state of charge, from 0 to 1023  (measures 0 ÷ 5 V)
long DIM;                        // voltage transformed to int from 0 ... 100
unsigned int Battery12V_Value = 0;    // initial value od digital value of the 12 V battery (0 ÷ 1023) 
float Battery12V;
unsigned int Battery12V_treshold = 11;   // set the min. voltage for signaling battery low voltage on display
int wiper2_water = 0;
char object[10];
int SOC2;

// Declaration of objects in NEXTION
// NexProgressBar - progress bar --> value 100 (is pic1) or 0 (pic0)
// NexButton ...
NexProgressBar j0 = NexProgressBar(0, 1, "j0");   // day lights
NexProgressBar j1 = NexProgressBar(0, 2, "j1");   // long lights
NexProgressBar j2 = NexProgressBar(0, 3, "j2");   //  fog
NexProgressBar j3 = NexProgressBar(0, 4, "j3");   //  left
NexProgressBar j4 = NexProgressBar(0, 5, "j4");   // right
NexProgressBar j5 = NexProgressBar(0, 6, "j5");   //  EmergenceLight
NexProgressBar j6 = NexProgressBar(0, 7, "j6");   //  handbrake
NexProgressBar j7 = NexProgressBar(0, 8, "j7");   //  batery low
NexProgressBar j8 = NexProgressBar(0, 9, "j8");   //  defrozing window
NexProgressBar j9 = NexProgressBar(0, 11, "j9");   // AC recharge lightning
NexProgressBar j10 = NexProgressBar(0, 12, "j10");   // "Nicola" text diplay or "Charging" text display
NexProgressBar j11 = NexProgressBar(0, 14, "j11");    // Displaying the prog. bar value of SOC, "0" empty, "100" full.
NexProgressBar j12 = NexProgressBar(0, 15, "j12");    // icon charging inside the van
NexPtogressBar j13 = NexProgressBar(0, 16, "j13");

// Declaration of touch events: (temp. the list is empty)
NexTouch *nex_listen_list[] =
{
  NULL
};

//Touch events:
// if needed write down here 


// setup program/sequence:
void setup() {
  nexInit();               // Nextion initalization
  Wire.begin(8);   // I2C !!
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);      // speed of comunication with Arduino

//register the event callback functions of each event
// OUTPUTS:
  pinMode(ParkLight, OUTPUT);          // relay 1
  pinMode(DayLight, OUTPUT);           // relay 2
  pinMode(LongLight, OUTPUT);          // na relay 3
  pinMode(TurnLeft, OUTPUT);           // na relay 4: --> 
  pinMode(TurnRight, OUTPUT);          // na relay 5: -->
  pinMode(StopLight, OUTPUT);          // na relay 6: 2
  pinMode(Horn, OUTPUT);               // na relay 7: 1 x Horn
  pinMode(DefrostWindow, OUTPUT);       // relay 8
  pinMode(BackwardLight, OUTPUT);       // relay 9: 1 
  pinMode(CabinLight, OUTPUT);          // relay 10: more lights with optional progrmaming
  pinMode(FogLight, OUTPUT);            // relay 11
  pinMode(EmergenceLight, OUTPUT);      // na relay 12 - to all flashes!
  pinMode(WindowWater, OUTPUT);         // na relay 13 --> motor of water pump
  pinMode(Wiper1Alternate, OUTPUT);     // relay 14
  pinMode(Wiper1, OUTPUT);              // relay 15
  pinMode(Wiper2, OUTPUT);              // relay 16 
// INPUTS:
  pinMode(ParkLight_Switch, INPUT);
  pinMode(DayLight_Switch, INPUT);
  pinMode(LongLight_Switch, INPUT);       
  pinMode(TurnLeft_Switch, INPUT);   
  pinMode(TurnRight_Switch, INPUT);  
  pinMode(StopLight_switch, INPUT);    // foot pedal microbutton
  pinMode(Horn_Switch, INPUT);        // steering wheel button
  pinMode(DefrostWindow_Switch, INPUT); 
  pinMode(BackwardLight_Switch, INPUT);     // Two-poles switch 
  pinMode(CabinLight_Switch, INPUT);  
  pinMode(EmergenceLight_Switch, INPUT);   
  pinMode(DoorOpen_Switch, INPUT); 
  pinMode(Handbrake_Switch, INPUT);
  pinMode(FogLight_Switch, INPUT);
  pinMode(WindowWater_Switch, INPUT);       
  pinMode(Wiper1Alternate_Switch, INPUT);  
  pinMode(Wiper1_Switch, INPUT);  // speed 1
  pinMode(Wiper2_Switch, INPUT);  // speed 2 
  pinMode(ChargeMode_input, INPUT); // if the AC cable is pludged
  pinMode(Battery12V_input, INPUT); // analog measurment of voltage 0 ÷ 12 V - wire a voltage divider with resistors (or Zener)!
  pinMode(DIM_input, INPUT);    // dimming with trimmer, 0 - 5 V LCD brightness

//main loop/program :
void loop() {  
  readwrite();      // reading the input states
  nexLoop(nex_listen_list);    // reading the touch events on NEXTION display (curently not used)
}

//function for reading digital and analog signals in Arduino:
void readwrite() {

// read the state of the pushbutton value:
  DayLight_Switch_ON = digitalRead(DayLight_Switch);
  ParkLight_Switch_ON = digitalRead(ParkLight_Switch);
  LongLight_Switch_ON = digitalRead(LongLight_Switch);
  TurnLeft_Switch_ON = digitalRead(TurnLeft_Switch);
  TurnRight_Switch_ON = digitalRead(TurnRight_Switch);
  StopLight_switch_ON = digitalRead(StopLight_switch);
  EmergenceLight_Switch_ON = digitalRead(EmergenceLight_Switch);
  SOC_Battery_Value = analogRead(SOC_input); // reading values from 0 - 5 V and converts to int value from 0 to 1023 .
  Battery12V_Value = analogRead(Battery12V_input);
  ChargeMode_ON = digitalRead(ChargeMode_input);
  Horn_Switch_ON = digitalRead(Horn_Switch);
  DefrostWindow_Switch_ON = digitalRead(DefrostWindow_Switch);
  BackwardLight_Switch_ON = digitalRead(BackwardLight_Switch);
  CabinLight_Switch_ON = digitalRead(CabinLight_Switch);
  DoorOpen_Switch_ON = digitalRead(DoorOpen_Switch);
  Handbrake_Switch_ON = digitalRead(Handbrake_Switch);
  FogLight_Switch_ON = digitalRead(FogLight_Switch);
  WindowWater_Button_ON = digitalRead(WindowWater_Switch);
  Wiper1Alternate_Switch_ON = digitalRead(Wiper1Alternate_Switch);
  Wiper1_Switch_ON = digitalRead(Wiper1_Switch);
  Wiper2_Switch_ON = digitalRead(Wiper2_Switch);
  

// wiper program - OFF --> speed 1 alternating --> speed 1 --> speed 2. When watering the window, makes 1 alternate loop
 

// when contact turns park lights on - relay 1
  if (ParkLight_Switch_ON == LOW) {
    digitalWrite(ParkLight, HIGH);
  } else {
    digitalWrite(ParkLight, LOW);
  }
// switch on armature for day lights - relay 2
  if (DayLight_Switch_ON == HIGH) {
    j0.setValue(100);
    digitalWrite(DayLight, LOW);
  } else {
    j0.setValue(0);
    digitalWrite(DayLight, HIGH);
  }
// switch on long lights on steering wheel - relay 3
  if (LongLight_Switch_ON == LOW) {
    j1.setValue(0);
    digitalWrite(LongLight, HIGH);
  } else {
    j1.setValue(100);
    digitalWrite(LongLight, LOW);
  }  
// flashing lights LEFT - relay 4
  if (TurnLeft_Switch_ON == LOW) {
    digitalWrite(TurnLeft, HIGH);
    j3.setValue(0);
  } else {
    digitalWrite(TurnLeft, LOW);
    j3.setValue(100);
  }
// flashing lights RIGHT - relay 5
  if (TurnRight_Switch_ON == LOW) {
    digitalWrite(TurnRight, HIGH);
    j4.setValue(0);
  } else {
    digitalWrite(TurnRight, LOW);
    j4.setValue(100);
  }
// micro switch on foot brake - relay 6
  if (StopLight_switch_ON == LOW) {
    digitalWrite(StopLight, HIGH);
  } else {
    digitalWrite(StopLight, LOW);
  }
// button on steering wheel - relay 7 
  if (Horn_Switch_ON == LOW) {
    digitalWrite(Horn, HIGH);
  } else {
    digitalWrite(Horn, LOW);
  }
// defrost switch on armature board - relay 8
  if (DefrostWindow_Switch_ON == HIGH) {
    digitalWrite(DefrostWindow, LOW);
    j8.setValue(100);
  } else {
    digitalWrite(DefrostWindow, HIGH);
    j8.setValue(0);
  }
// two-pole switch for changing drive direction/connected with Emsiso - relay 9
  if (BackwardLight_Switch_ON == HIGH) {
    digitalWrite(BackwardLight, LOW);
    //j?.setValue(100); //display on nextion going backward!
  } else {
    digitalWrite(BackwardLight, HIGH);
    //j?.setValue(0);
  }
// 3 position switch: 1. OFF, cabin light on when door open; 2. OFF always; 3. ON always - relay 10
  if (CabinLight_Switch_ON == LOW or DoorOpen_Switch_ON == LOW) {
    digitalWrite(CabinLight, HIGH);
  } else {
    digitalWrite(CabinLight, LOW);
  }
// Fog lights on armature switch - relay 11
  if (FogLight_Switch_ON == LOW) {
    digitalWrite(FogLight, HIGH);
    j2.setValue(0);
  } else {
    digitalWrite(FogLight, LOW);
    j2.setValue(100);
  } 
// Emergence light on armature switch - relay 12
  if (EmergenceLight_Switch_ON == HIGH) {
    digitalWrite(EmergenceLight, LOW);
    j5.setValue(100);
  } else {
    digitalWrite(EmergenceLight, HIGH);
    j5.setValue(0);
  }
// Handbrake micro switch, status icon on display
  if (Handbrake_Switch_ON == HIGH) {
    j6.setValue(100);
  } else {
    j6.setValue(0);
  }

// If AC cable plug-in, display lightning and battery icon
  if (ChargeMode_ON == LOW) {
    j9.setValue(100);   //Strela
    j10.setValue(0);   // Show "charging" 
    j12.setValue(0);    // icon charging
  } else {
    j9.setValue(0);        // strela se skrije
    j10.setValue(100);      //prikaže se napis BALKAN CAMPERS
    j12.setValue(100);       // blank
  }

 if (Wiper1Alternate_Switch_ON == HIGH)
  {
    digitalWrite(Wiper1Alternate, HIGH);   // turn relay 14, over time relay for speed 1
  } else {
    digitalWrite(Wiper1Alternate, LOW);
    }   
  if (Wiper1_Switch_ON == HIGH)  
  {
    digitalWrite(Wiper1, HIGH);      // turn relay 15, speed 1
  } else {
    digitalWrite(Wiper1, LOW);
    }
  wiper2_water = 2*Wiper2_Switch_ON + 1*WindowWater_Button_ON;
   
  if (WindowWater_Button_ON == LOW)
  {
    //digitalWrite(Wiper2, LOW);
    digitalWrite(WindowWater, LOW);   // turn relay 13, to water pump, also via DIODE wiper speed 2
  }  else {
    //digitalWrite(Wiper2, HIGH);
    digitalWrite(WindowWater, HIGH);
    }
  if (Wiper2_Switch_ON == HIGH) {
    digitalWrite(Wiper2, HIGH);
    }
    else {
    digitalWrite(Wiper2, LOW);
    }
  
// dim the display - reading DIM_value, using potenciometer on A0
// On nextion HMI file the page0 postinitialize event has code: j13.value=dim
  DIM = DIM_Value*100/1023;
  j13.setValue(DIM);
// Displaying SOC graphic via wire.h on NANO 
  j11.setValue(SOC2);    // when aquiring data from Arduino NANO, change to SOC2! 
 
//  Serial.println(DIM);         // for diagnostic!
//  Serial.println(DIM_Value);

/* display of low 12v battery icon, if voltage is lower than 11 V reading from A11
  0 - ~17 V voltmeter
  works with 3.3volt and 5volt Arduinos
  uses the stable internal 1.1volt reference
  10k resistor from A0 to ground, and 150k resistor from A0 to +batt
  (1k8:27k or 2k2:33k are also valid 1:15 ratios)
  100n capacitor from A0 to ground for stable readings    */
  Battery12V=Battery12V_Value/19.5533;  // calculation to voltage value
  //Serial.println(Battery12V_Value);   // for monitoring purpouses
  //Serial.println(Battery12V);
  if (Battery12V < Battery12V_treshold) {      //set the voltage treshold
    j7.setValue(100);
    }
  else {
    j7.setValue(0);
    }
}

void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    //Serial.print(c);         // print the character
  }
  SOC2 = Wire.read();    // receive byte as an integer
  Serial.println(SOC2);         // print the integer
}

