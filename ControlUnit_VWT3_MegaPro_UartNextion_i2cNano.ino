/*FIRST version NEXTION display with ARDUINO mega2560 PRO, connection working 
linked with HMI file "NEXTION_2_4_ver1_INVERTER"
in "nexConf.h" enable debug and must be set to Serial2 ...
Wiring the two data wires:            Wiring also the I2C protocol for SOC values from arduino NANO
NEXTION Rx --> Tx2 MEGA (16)              A4 NANO --> 20 MEGA
NEXTION Tx --> Rx2 MEGA (17)              A5 NANO --> 21 MEGA
The input signals are switching icons on display - I used ProgressBAR for that trick!
Make sure you import the ITEADlib library in arduino IDE!!
The Relay module has to be connected to power a 10 seconds after the key power the Arduino & Nextion to prevent switching relays ON!
Dimming the display with tuch function (Nextion code)
Dimming the dash lights with potentimeter on dashboard SWITCH1
11.9. 2018 fix the code (I/O pins) for MEga PRO CH340G. PINS from 2 ÷ 13 are PWM. 
13.9.2018 Upddated the pin numbers for the Shield (fritzing PCB sheme), emergency lights will turn with both Relay 4 and 5 
via 103 131 topran 12V flashing relay: pins - 49 (+12V), 49a (LOAD), 31 (GND)
Added MotorCover microswitch Input and Motor compartment light (Output)
Relay 12 turns the motorlights.
30.9.2018 Updated the Output pins with shematic "EVnikola Omarca Robodyn krmilje postavitev" (VISIO)
4.10. 
NAREDI DA D4PWM zasvetijo programsko ob pritisku park luči!!!!!!!!


Author: Vasja Markič           
*/
#include <Wire.h>
#include <doxygen.h>
//#include <NexButton.h>
#include <NexConfig.h>
//#include <NexCrop.h> //#include <NexGauge.h> //#include <NexHardware.h> //#include <NexHotspot.h> //#include <NexObject.h> //#include <NexPage.h>
#include <NexPicture.h>
#include <NexProgressBar.h>
//#include <NexSlider.h> //#include <NexText.h>
#include <Nextion.h>
#include <NexTouch.h>
//#include <NexWaveform.h>
#include "Nextion.h"

// OUTPUTS pin declaration:
const int TurnLeft = 22;             // relay 1 - 3x left turning lights (via flashing relay 103 131)
const int TurnRight = 14;            // relay 2 - 3x right turning lights (via flashing relay 103 131) (both turns on via SWITCH3)
const int ParkLight =  12;           // relay 3 - 2x front parklights, 2x back parklights + registar tables lights
const int DayLight = 19;             // relay 4 - 2x front daylights H4
const int LongLight = 25;            // relay 5 - 2x front longlights H4
const int StopLight = 29;            // relay 6 - 2x back stoplights
const int Horn = 30;                 // relay 7 - to horn acuator
const int DefrostWindow = 26;        // relay 8 - to rear window defrost
const int BackwardLight = 18;        // relay 9 - to rear backlight(s)
const int WiperRear = 11;            // relay 10 - motorpump rear AND motor wiper rear
const int FogLight = 15;             // relay 11 - 2x front foglights
const int MotorLight = 23;           // relay 12 - lights when cover motor is up ...
const int WindowWater = 27;          // relay 13 - motorpump front
const int Wiper1Alternate = 31;      // relay 14 - via auto wiper intermittent relay to motor wiper front, timer
const int Wiper1 = 28;               // relay 15 - wire to auto wiper intermittent relay
const int Wiper2 = 24;               // relay 16 - wire directly to wiper motor
// 21, 20 reserved for I2C comm with NANO soc-meter
// 17, 16 reserved for Tx2 and Rx2 comm with Nextion display
// 13 reserved integrated LED       // n.c.

// PWM OUTUPTS (from 0 ÷ 13)
const int DashLightPWM = 4;   // dimm dashboard lights
const int TempGaugePWM = 3;    // drive temp. gauge
const int FuelGaugePWM = 2;    // drive voltage pack (ex. fuel) gauge

// INPUTS:
const int ParkLight_Switch = 49;      // SWITCH1 (on dash)      **+ --> 3x singal wires, 1x GND and 1x 5V supply
const int DayLight_Switch = 45;       // SWITCH1 (on dash)      **+
const int LongLight_Switch = 41;      // WHEEL SWITCH1          ****+  --> 4x signal wires and 1x 5V supply
const int TurnLeft_Switch = 39;       // WHEEL SWITCH1          ****+
const int TurnRight_Switch = 37;      // WHEEL SWITCH1          ****+
const int Horn_Switch = 35;           // WHEEL SWITCH???1??    ****+
const int FogLight_Switch = 53;        //SWITCH2 (extra added on dash)   *+
const int EmergenceLight_Switch = 46;  //SWITCH3 (on dash)                   *+
const int DefrostWindow_Switch = 47;   //SWITCH4 (on dash)      *+
const int BackwardLight_Switch = 48;   // two-pole switch (2pSW) to change the drive direction! (on dash, added extra!)   *+
const int DoorOpen_Switch = 50;        // MICROSWITCH1 (4 x doors)    *+
const int Handbrake_Switch = 52;       // MICROSWITCH2 (handbrake)    *+   ----> directly to dash light!!!
const int ChargeMode_input = 51;       // MICROSWITCH3 (when AC plug cable is IN, rezervoir tank)   *+
const int StopLight_switch = 44;      // MICROSWITCH4 (on the brake foot)    *+
const int WindowWater_Switch = 36;      // WHEEL SWITCH2      *****+
const int Wiper1_Switch = 38;           // WHEEL SWITCH2      *****+
const int Wiper2_Switch = 40;           // WHEEL SWITCH2      *****+
const int Wiper1Alternate_Switch = 42;  // WHEEL SWITCH2      *****+
const int WiperRear_Switch = 34;      // WHEEL SWITCH2        *****+
const int MotorCover_Switch = 43;         // MICROSWITCH5 Motor Cover open/closed *+
// analoge INPUTS (from A0 ÷ A15)
const int Battery12V_input = A1;          // analog input, resistor divider 1:15 ratio with capacitor   *-+
const int DIM_input = A0;                 // potentiometer on SWITCH1, for dimming the dash Lights    ***-+
const int BatteryPackVolt_input = A3;     // analog input, 86V ÷ 125V range via resistor divider R1 = 12K, R2 = 500E --> input range 3.44V ÷ 5V   ???? (GND NOT common!)
const int Temperature_input = A2;         // depends on the PTK probe!   ???

// ventilation via PWM power module

// display SOC - using existing fuel gauge (0 ÷ 6 V) reading analog IN and drive with digital PWM and MOSFET BS 170 ali BS 107.
// display temp. motor (0 ÷ 6 V)

// variables will change - HIGH state relay OFF:
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
bool Handbrake_Switch_ON = true;
bool WiperRear_Switch_ON = true;
bool WindowWater_Button_ON = true;
bool Wiper1Alternate_Switch_ON = true;
bool Wiper1_Switch_ON = true;
bool Wiper2_Switch_ON = true;
bool FogLight_Switch_ON = true;
bool ChargeMode_ON = false;       // if we plug the AC charging
long DIM_Value = 0;      // initial value of the dimming display, from 0 to 1023  (measures 0 ÷ 5 V)
long DIM;                        // DIM transformed to int from 0 ... 100
unsigned int Battery12V_Value = 0;    // initial value od digital value of the 12 V battery (0 ÷ 1023) 
unsigned int BatteryPackVolt_Value = 0;
unsigned int Temperature_Value = 0;
float Battery12V;
long BatteryPackVolt;      // transform 3.44V to 0 and 5.00V to 40
long Temperature;         // transform 0.00V to 0 and 5.00V to 40
unsigned int Battery12V_treshold = 11;   // set the min. voltage for signaling battery low voltage on display
int wiper2_water = 0;
char object[10];
int SOC2;         //aquire from arduino NANO SOC Meter

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
NexProgressBar j9 = NexProgressBar(0, 11, "j9");      // AC recharge lightning
NexProgressBar j10 = NexProgressBar(0, 12, "j10");    // "Nicola" text diplay or "Charging" text display
NexProgressBar j11 = NexProgressBar(0, 14, "j11");    // Displaying the prog. bar value of SOC, "0" empty, "100" full.
NexProgressBar j12 = NexProgressBar(0, 15, "j12");    // icon charging inside the van
NexProgressBar j13 = NexProgressBar(0, 16, "j13");    // slider of Dimming brightness of dashboard lights

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
  pinMode(LongLight, OUTPUT);          // relay 3
  pinMode(TurnLeft, OUTPUT);           // relay 4
  pinMode(TurnRight, OUTPUT);          // relay 5
  pinMode(StopLight, OUTPUT);          // relay 6
  pinMode(Horn, OUTPUT);               // relay 7: 1 x Horn
  pinMode(DefrostWindow, OUTPUT);       // relay 8
  pinMode(BackwardLight, OUTPUT);       // relay 9
  pinMode(WiperRear, OUTPUT);          // relay 10: back wiper and watering
  pinMode(FogLight, OUTPUT);            // relay 11
  pinMode(MotorLight, OUTPUT);      // relay 12 Lights in motor compartment
  pinMode(WindowWater, OUTPUT);         // relay 13 motor of water pump front
  pinMode(Wiper1Alternate, OUTPUT);     // relay 14
  pinMode(Wiper1, OUTPUT);              // relay 15
  pinMode(Wiper2, OUTPUT);              // relay 16 
  pinMode(DashLightPWM, OUTPUT);          // to MOSFET 1  drives dash lights via RELAY 1 (5 V), mosfet driven with 12 V
  pinMode(TempGaugePWM, OUTPUT);          // to MOSFET 2  drives tempertature GAUGE (6 V), mosfet driven on 12 V
  pinMode(FuelGaugePWM, OUTPUT);          // to MOSFET 3 drives fuel GAUGE (6 V), mosfet driven on 12 V
  
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
  pinMode(WiperRear_Switch, INPUT);  
  pinMode(EmergenceLight_Switch, INPUT);   
  pinMode(DoorOpen_Switch, INPUT); 
  pinMode(Handbrake_Switch, INPUT);
  pinMode(FogLight_Switch, INPUT);
  pinMode(WindowWater_Switch, INPUT);       
  pinMode(Wiper1Alternate_Switch, INPUT);  
  pinMode(Wiper1_Switch, INPUT);  // speed 1
  pinMode(Wiper2_Switch, INPUT);  // speed 2 
  pinMode(ChargeMode_input, INPUT); // if the AC cable is pludged
  pinMode(Battery12V_input, INPUT); // analog measurment of voltage 0 ÷ 12 V - wire a voltage divider with resistors 81.3k + (8.25k || 100nF)
  pinMode(DIM_input, INPUT);    // dimming with trimmer on SWITCH1 (lights), 0 - 5V (1023) LCD brightness    (ANALOG)
  pinMode(BatteryPackVolt_input, INPUT);   // range from 3.44V (703) to 5V (1023)   (ANALOG)  // GNDs must be separated!!!
  pinMode(Temperature_input, INPUT);    // from 0V (0) to 5V (1023) via PTK   (ANALOG)
}
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
  ChargeMode_ON = digitalRead(ChargeMode_input);
  Horn_Switch_ON = digitalRead(Horn_Switch);
  DefrostWindow_Switch_ON = digitalRead(DefrostWindow_Switch);
  BackwardLight_Switch_ON = digitalRead(BackwardLight_Switch);
  WiperRear_Switch_ON = digitalRead(WiperRear_Switch);
  DoorOpen_Switch_ON = digitalRead(DoorOpen_Switch);
  Handbrake_Switch_ON = digitalRead(Handbrake_Switch);
  FogLight_Switch_ON = digitalRead(FogLight_Switch);
  WindowWater_Button_ON = digitalRead(WindowWater_Switch);
  Wiper1Alternate_Switch_ON = digitalRead(Wiper1Alternate_Switch);
  Wiper1_Switch_ON = digitalRead(Wiper1_Switch);
  Wiper2_Switch_ON = digitalRead(Wiper2_Switch);
  DIM_Value = analogRead(DIM_input); // reading values from 0 - 5 V and converts to int value from 0 to 1023 .
  Battery12V_Value = analogRead(Battery12V_input);
  BatteryPackVolt_Value = analogRead(BatteryPackVolt_input);
  Temperature_Value = analogRead(Temperature_input);
  
  // 830 --> 100, 1023 --> 255; ???? resistor divider - 100 Ohm and 23 Ohm pot. , 5V --> Max 5 V (1023), Min 4.06 V (830)
  DIM = DIM_Value*255/1023;
  //equations for Fuel gauge and Temperature gauge:
  // 703 --> 0, 1023 --> 255    BUT - GND of the pack is insulated from the circuits!!!
  BatteryPackVolt = (BatteryPackVolt_Value-703)*255/(1023-703);
  // 0 --> 0, 1023 --> 255
  Temperature = Temperature_Value*255/1023;
  
  // sending PWM singal to dash lights, temperature gauge and fuel gauge:
  analogWrite(DashLightPWM, DIM);
  analogWrite(FuelGaugePWM, BatteryPackVolt);
  analogWrite(TempGaugePWM, Temperature);

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
// Activation of rear wiper and watering pump
  if (WiperRear_Switch_ON == LOW) {
    digitalWrite(WiperRear, HIGH);
  } else {
    digitalWrite(WiperRear, LOW);
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
    //digitalWrite(MotorLight, LOW);
    digitalWrite(TurnLeft, HIGH);
    digitalWrite(TurnRight, HIGH);
    j5.setValue(100);
  } else {
    //digitalWrite(MotorLight, HIGH);
    digitalWrite(TurnLeft, HIGH);
    digitalWrite(TurnRight, HIGH);
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
  
  j13.setValue(DIM);    //displaying the brigtness of the dash lights. 
// Displaying SOC graphic via wire.h on NANO 
  j11.setValue(SOC2);    // when aquiring data from Arduino NANO, change to SOC2! 
 
//  Serial.println(SOC2);         // for diagnostic!

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
//function for reading via I2C the SOC from Nano
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    //Serial.print(c);         // print the character
  }
  SOC2 = Wire.read();    // receive byte as an integer
  Serial.println(SOC2);         // print the integer
}


