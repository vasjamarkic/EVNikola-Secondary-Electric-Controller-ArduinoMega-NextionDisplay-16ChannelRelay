/*FIRST version NEXTION display with ARDUINO mega2560 PRO, connection working 
linked with HMI file "NEXTION_Nikola.hmi"
in "nexConf.h" enable debug and must be set to Serial2 ...
Wiring the two data wires:            Wiring also the I2C protocol for SOC values from arduino NANO
NEXTION Rx --> Tx2 MEGA (16)              A4 NANO --> 20 MEGA
NEXTION Tx --> Rx2 MEGA (17)              A5 NANO --> 21 MEGA
The input signals are Swing icons on display - I used ProgressBAR for that trick!
Make sure you import the ITEADlib library in arduino IDE!!
The Relay module has to be connected to power a 10 seconds after the key power the Arduino & Nextion to prevent Swing relays ON!
Dimming the display with tuch function (Nextion code)
Dimming the dash lights with potentimeter on dashboard Sw1
11.9. 2018 fix the code (I/O pins) for MEga PRO CH340G. PINS from 2 ÷ 13 are PWM. 
13.9.2018 Upddated the pin numbers for the Shield (fritzing PCB sheme), emergency lights will turn with both Relay 4 and 5 
via 103 131 topran 12V flashing relay: pins - 49 (+12V), 49a (LOAD), 31 (GND)
Added MotorCover microSw Input and Motor compartment light (Output)
Relay 12 turns the motorlights.
30.9.2018 Updated the Output pins with shematic "EVnikola Omarca Robodyn krmilje postavitev" (VISIO)
24.10. Solved the sloow switching - using an Array for storing input states, then aktivating the logic only when the change is commited, 
Using indexing and switch-case! Also the emergency outputs works fine now!!!
26.10. eliminated all small errata in I/O displaying Nextion, added a graphic shiny LED!

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
#define TurnLeft 22              // relay 1 - 2x left turning lights (via flashing relay 103 131)
#define TurnRight 25             // relay 2 - 2x right turning lights (via flashing relay 103 131) (both - emergency stop - turns on via Sw3)
#define ParkLight 18             // relay 3 - 2x front parklights, 2x back parklights + registratiton tables lights, PWM dashboard ON
#define DayLight  27             // relay 4 - 2x front daylights H4
#define LongLight  14            // relay 5 - 2x front longlights H4
#define StopLight  29            // relay 6 - 2x back stoplights
#define Horn  11                 // relay 7 - to horn acuator
#define DefrostWindow  31        // relay 8 - to rear window defrost
#define BackwardLight  12        // relay 9 - to rear backlight(s)
#define WiperRear  30            // relay 10 - motorpump rear AND motor wiper rear
#define FogLight  15             // relay 11 - 2x front foglights
#define MotorLight  28           // relay 12 - lights when cover motor is up ...
#define WindowWater  19          // relay 13 - motorpump front
#define Wiper1Alternate  26      // relay 14 - via auto wiper intermittent relay to motor wiper front, timer
#define Wiper1  23               // relay 15 - wire to auto wiper intermittent relay
#define Wiper2  24               // relay 16 - wire directly to wiper motor
// 21, 20 reserved for I2C comm with NANO soc-meter
// 17, 16 reserved for Tx2 and Rx2 comm with Nextion display
// 13 reserved integrated LED       // n.c.

// PWM OUTUPTS (from 0 ÷ 13)
#define DashLightPWM  4   // dimm dashboard lights
#define TempGaugePWM  3    // drive temp. gauge
#define FuelGaugePWM  2    // drive voltage pack (ex. fuel) gauge

// INPUTS:
#define ParkLight_Sw 49        // Sw1 (on dash)                 **+ --> 3x singal wires, 1x GND and 1x 5V supply
#define DayLight_Sw  45        // Sw1 (on dash)                 **+
#define LongLight_Sw  41       // WHEEL Sw1 Left side           ****+  --> 4x signal wires and 1x 5V supply
#define TurnLeft_Sw  39        // WHEEL Sw1 Right side          ****+
#define TurnRight_Sw  37       // WHEEL Sw1 Right side          ****+
#define Horn_Sw  35            // WHEEL Sw1 Right side (??)     ****+
#define FogLight_Sw  53        // Sw2 (extra added on dash)        *+
#define EmergenceLight_Sw  46  // Sw3 (on dash)                    *+
#define DefrostWindow_Sw  47   // Sw4 (on dash)                    *+
#define BackwardLight_Sw  48   // two-pole Sw (2pSW) to change the drive direction! (on dash, added extra!)   *+
#define Reserve_uSw1  50       //                     *+  
#define Handbrake_Sw  52       // uSw2 (handbrake)    *+  (!) icon on NEXTION
#define ChargeMode_input  51   // uSw3 (when AC plug cable is IN, rezervoir tank)   *+
#define StopLight_Sw  44       // uSw4 (on the brake foot)    *+
#define WindowWater_Sw  36     // WHEEL Sw2      *****+
#define Wiper1_Sw  38          // WHEEL Sw2      *****+
#define Wiper2_Sw  40          // WHEEL Sw2      *****+
#define Wiper1Alternate_Sw  42 // WHEEL Sw2      *****+
#define WiperRear_Sw  34       // WHEEL Sw2      *****+
#define MotorCover_Sw  43      // uSw5 Motor Cover open/closed *+

// analoge INPUTS (from A0 ÷ A15)
#define DIM_input  A0                 // potentiometer on Sw1, for dimming the dash Lights    ***-+
#define Battery12V_input  A1          // analog input, resistor divider 1:15 ratio with capacitor   *-+
#define BatteryPackVolt_input  A3     // analog input, 86V ÷ 125V range via resistor divider R1 = 12K, R2 = 500E --> input range 3.44V ÷ 5V   ???? (GND NOT common!)
#define Temperature_input  A2         // depends on the PTK probe!   ???

// ventilation via PWM power module
// heating via high voltage heater???
// display SOC - using existing fuel gauge (0 ÷ 6 V) reading analog IN and drive with digital PWM and MOSFET BS 170 ali BS 107.
// display temp. motor (0 ÷ 6 V)

// variables will change - HIGH state relay OFF:
int ParkLight_Sw_ON = true;
int DayLight_Sw_ON = true;
int LongLight_Sw_ON = true;
int TurnLeft_Sw_ON = true;
int TurnRight_Sw_ON = true;
int EmergenceLight_Sw_ON = true;
int StopLight_Sw_ON = true;
int Horn_Sw_ON = true;
int DefrostWindow_Sw_ON = true;
int BackwardLight_Sw_ON = true;
int CabinLight_Sw_ON = true;
int Reserve_uSw1_ON = true;
int MotorCover_Sw_ON = true;
int Handbrake_Sw_ON = true;
int WiperRear_Sw_ON = true;
int WindowWater_Button_ON = true;
int Wiper1Alternate_Sw_ON = true;
int Wiper1_Sw_ON = true;
int Wiper2_Sw_ON = true;
int FogLight_Sw_ON = true;
int ChargeMode_ON = true;       // if we plug the AC charging
long DIM_Value = 0;      // initial value of the dimming display, from 0 to 1023  (measures 0 ÷ 5 V)
long DIM;                        // DIM transformed to int from 0 ... 100
unsigned int Battery12V_Value = 0;    // initial value od digital value of the 12 V battery (0 ÷ 1023) 
unsigned int BatteryPackVolt_Value = 0;
unsigned int Temperature_Value = 0;
unsigned int Battery12V_treshold = 11;   // set the min. voltage for signaling battery low voltage on display
float Battery12V;
long BatteryPackVolt;      // transform 3.44V to 0 and 5.00V to 40
long Temperature;         // transform 0.00V to 0 and 5.00V to 40
char object[10];
int SOC2;         //aquire from arduino NANO SOC Meter
volatile double ARRAY[20];          // for storing input from switches
volatile double ARRAY_OLD[20];      // the loop before, for trigering an event!

// Declaration of objects in NEXTION
// NexProgressBar - progress bar --> value 100 (is pic1) or 0 (pic0)
// NexButton ...
NexProgressBar j0 = NexProgressBar(0, 3, "j0");   // day lights
NexProgressBar j1 = NexProgressBar(0, 4, "j1");   // long lights
NexProgressBar j2 = NexProgressBar(0, 5, "j2");   //  fog
NexProgressBar j3 = NexProgressBar(0, 6, "j3");   //  left
NexProgressBar j4 = NexProgressBar(0, 7, "j4");   //  right
NexProgressBar j5 = NexProgressBar(0, 8, "j5");   //  EmergenceLight
NexProgressBar j6 = NexProgressBar(0, 9, "j6");   //  handbrake
NexProgressBar j7 = NexProgressBar(0, 10, "j7");   //  batery low
NexProgressBar j8 = NexProgressBar(0, 11, "j8");   //  defrozing window
NexProgressBar j9 = NexProgressBar(0, 12, "j9");      // AC recharge lightning
NexProgressBar j10 = NexProgressBar(0, 13, "j10");    // "Nicola" text diplay or "Charging" text display
NexProgressBar j11 = NexProgressBar(0, 14, "j11");    // Displaying the progress bar value of SOC, "0" empty, "100" full.
// write here for displaying SOC value ...
NexProgressBar j12 = NexProgressBar(0, 15, "j12");    // icon charging inside the van
NexProgressBar j13 = NexProgressBar(0, 16, "j13");    // slider of Dimming brightness of dashboard lights
NexProgressBar j14 = NexProgressBar(0, 19, "j14");    // Cover motor open!
NexProgressBar j15 = NexProgressBar(0, 17, "j15");    // Backward icon on display
NexProgressBar j16 = NexProgressBar(0, 20, "j16");    // PARK light

// Declaration of touch events: (temp. the list is empty)
NexTouch *nex_listen_list[] =
{
  NULL
};

//Touch events:
// if needed write down here 

//*******************************************************************************************************
// setup program/sequence:
void setup() {
  nexInit();               // Nextion initalization
  Wire.begin(8);   // I2C !!
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);      // speed of comunication with Arduino

//register the event callback functions of each event
// OUTPUTS:
  pinMode(TurnLeft, OUTPUT);           // relay 1
  pinMode(TurnRight, OUTPUT);          // relay 2
  pinMode(ParkLight, OUTPUT);          // relay 3
  pinMode(DayLight, OUTPUT);           // relay 4
  pinMode(LongLight, OUTPUT);          // relay 5
  pinMode(StopLight, OUTPUT);          // relay 6
  pinMode(Horn, OUTPUT);               // relay 7: 1 x Horn
  pinMode(DefrostWindow, OUTPUT);      // relay 8
  pinMode(BackwardLight, OUTPUT);      // relay 9
  pinMode(WiperRear, OUTPUT);          // relay 10: back wiper and watering
  pinMode(FogLight, OUTPUT);            // relay 11
  pinMode(MotorLight, OUTPUT);          // relay 12 Lights in motor compartment
  pinMode(WindowWater, OUTPUT);         // relay 13 motor of water pump front
  pinMode(Wiper1Alternate, OUTPUT);     // relay 14
  pinMode(Wiper1, OUTPUT);              // relay 15
  pinMode(Wiper2, OUTPUT);              // relay 16 
  pinMode(DashLightPWM, OUTPUT);        // to MOSFET 1  drives dash lights via RELAY 1 (5 V), mosfet driven with 12 V
  pinMode(TempGaugePWM, OUTPUT);        // to MOSFET 2  drives tempertature GAUGE (6 V), mosfet driven on 12 V
  pinMode(FuelGaugePWM, OUTPUT);        // to MOSFET 3 drives fuel GAUGE (6 V), mosfet driven on 12 V
  
// INPUTS:
  pinMode(ParkLight_Sw, INPUT);
  pinMode(DayLight_Sw, INPUT);
  pinMode(LongLight_Sw, INPUT);       
  pinMode(TurnLeft_Sw, INPUT);   
  pinMode(TurnRight_Sw, INPUT);  
  pinMode(StopLight_Sw, INPUT);    // foot pedal microbutton
  pinMode(Horn_Sw, INPUT);         // steering wheel button
  pinMode(DefrostWindow_Sw, INPUT); 
  pinMode(BackwardLight_Sw, INPUT);     // Two-poles Sw 
  pinMode(WiperRear_Sw, INPUT);  
  pinMode(EmergenceLight_Sw, INPUT);   
  pinMode(Reserve_uSw1, INPUT); 
  pinMode(MotorCover_Sw, INPUT);
  pinMode(Handbrake_Sw, INPUT);
  pinMode(FogLight_Sw, INPUT);
  pinMode(WindowWater_Sw, INPUT);       
  pinMode(Wiper1Alternate_Sw, INPUT);  
  pinMode(Wiper1_Sw, INPUT);  // speed 1
  pinMode(Wiper2_Sw, INPUT);  // speed 2 
  pinMode(ChargeMode_input, INPUT); // if the AC cable is pludged
  pinMode(Battery12V_input, INPUT); // ANALOG measurment of voltage 0 ÷ 12 V - wire a voltage divider with resistors 81.3k + (8.25k || 100nF)
  pinMode(DIM_input, INPUT);    // dimming with trimmer on Sw1 (lights), 0 - 5V (1023), could also trim the LCD brightness (ANALOG)
  pinMode(BatteryPackVolt_input, INPUT);   // range from 3.44V (703) to 5V (1023)   (ANALOG)  // GNDs must be separated!!!
  pinMode(Temperature_input, INPUT);    // from 0V (0) to 5V (1023) via PTK   (ANALOG)
  
  relayOFF();       //sets all relay to OFF!
  testDisplay();     // test all the icons for 0.5 sec
}
//***************************************************************************************************
//main loop/program :
void loop() {  
  readDigital();      // reading the input states into an ARRAY
  readAnalog();       // reading the input ANALOG 
  equations();
  writeDigital();    //Solved with software interrupt - if change in switches! Used Switch-Case
  writeAnalog();    
  nexLoop(nex_listen_list);    // reading the touch events on NEXTION display (curently not used)
}
//*********************************************************************************************
//function for reading digital and analog signals in Arduino:
void readDigital() {

// read the state of the inputs and store it in a Array size 20

  for (int i = 0; i < 20; i++) {
  ARRAY_OLD[i]=ARRAY[i];        // store the last data to the ARRAY_OLD.
  }
  ARRAY[0] = digitalRead(TurnLeft_Sw);     //New readings and storing in ARRAY
  ARRAY[1] = digitalRead(TurnRight_Sw);
  ARRAY[2] = digitalRead(ParkLight_Sw);
  ARRAY[3] = digitalRead(DayLight_Sw);
  ARRAY[4] = digitalRead(LongLight_Sw);
  ARRAY[5] = digitalRead(StopLight_Sw);
  ARRAY[6] = digitalRead(EmergenceLight_Sw); 
  ARRAY[7] = digitalRead(ChargeMode_input);
  ARRAY[8] = digitalRead(Horn_Sw);
  ARRAY[9] = digitalRead(DefrostWindow_Sw);
  ARRAY[10] = digitalRead(BackwardLight_Sw);
  ARRAY[11] = digitalRead(WiperRear_Sw);
  ARRAY[12] = digitalRead(Reserve_uSw1);
  ARRAY[13] = digitalRead(MotorCover_Sw);
  ARRAY[14] = digitalRead(Handbrake_Sw);
  ARRAY[15] = digitalRead(FogLight_Sw);
  ARRAY[16] = digitalRead(WindowWater_Sw);
  ARRAY[17] = digitalRead(Wiper1Alternate_Sw);
  ARRAY[18] = digitalRead(Wiper1_Sw);
  ARRAY[19] = digitalRead(Wiper2_Sw);
}
 
//******************************************************************************************
void writeDigital() {
  if (ARRAY != ARRAY_OLD) {              // if any switch activated, one of the digit is different, compare the OLD and NEW ARRAY
    for (int i = 0; i < 20; i++) {       // check all inputs in the array
    if (ARRAY_OLD[i]!=ARRAY[i]){          // when found the index (changed input), change the status!
      switch (i) {
        case 0:                         // turn left
        j3.setValue(ARRAY[i]*100);
        digitalWrite(TurnLeft, !ARRAY[i]);
        break;
        case 1:                       // turn right
        j4.setValue(ARRAY[i]*100);
        digitalWrite(TurnRight, !ARRAY[i]);
        break;
        case 2:                           // park light
        digitalWrite(ParkLight, !ARRAY[i]);
        j16.setValue(ARRAY[i]*100);
        // set PWM D4 LED cockpit ON DIM Value in the function writeAnalog()
        //analogWrite(DashLightPWM,!ARRAY[i]*DIM); --> into the writeAnalog() function
        break;
        case 3:                         // day lights
        j0.setValue(ARRAY[i]*100);
        digitalWrite(DayLight, !ARRAY[i]);
        break;
        case 4:                         // long lights
        j1.setValue(ARRAY[i]*100);
        digitalWrite(LongLight, !ARRAY[i]);
        break;
        case 5:                         // stop light
        digitalWrite(StopLight, !ARRAY[i]);
        break;
        case 6:                         // Emergence blinkers
        digitalWrite(TurnRight, !ARRAY[i]);
        digitalWrite(TurnLeft, !ARRAY[i]);
        j5.setValue(ARRAY[i]*100);
        break;
        case 7:                          // CHARGE mode input
        j9.setValue(ARRAY[i]*100);   //Strela
        j10.setValue(!ARRAY[i]*100);   // Show "charging" 
        j12.setValue(ARRAY[i]*100);    // icon charging
        break;
        case 8:                       // HORN
        digitalWrite(Horn, !ARRAY[i]);
        break;
        case 9:                        // DEFROST
        j8.setValue(ARRAY[i]*100);
        digitalWrite(DefrostWindow, !ARRAY[i]);
        break;
        case 10:                   // BackWard Light
        j15.setValue(ARRAY[i]*100);         // icon going back!!!
        digitalWrite(BackwardLight, !ARRAY[i]);
        break;
        case 11:                          // Wiper REAR + WATER
        digitalWrite(WiperRear, !ARRAY[i]);
        break;
        case 12:                          // RESERVE uSw1
        break;
        case 13:                          // Motor cover - light back motor compartment
        j14.setValue(ARRAY[i]*100);
        digitalWrite(MotorLight, !ARRAY[i]);
        break;
        case 14:                        // Handbrake
        j6.setValue(ARRAY[i]*100);
        break;
        case 15:                        // Fog Light
        j2.setValue(ARRAY[i]*100);
        digitalWrite(FogLight, !ARRAY[i]);
        break;
        case 16:                        // Window WATER front
        digitalWrite(WindowWater, !ARRAY[i]);
        break;
        case 17:                        // Wiper 1 ALT
        digitalWrite(Wiper1Alternate, !ARRAY[i]);
        break;
        case 18:                        // WIPER 1
        digitalWrite(Wiper1, !ARRAY[i]);
        break;
        case 19:                        // WIPER 2
        digitalWrite(Wiper2, !ARRAY[i]);
        break;       
        }
      }
    }
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

void writeAnalog() {
  analogWrite(FuelGaugePWM, BatteryPackVolt);   // wire to gauge fuel
  analogWrite(TempGaugePWM, Temperature);       // wire to Temp. gauge. 
  analogWrite(DashLightPWM, ARRAY[2]*DIM);      //the third field in the array is ParkLight input switch.
  }
  
void readAnalog() {
  DIM_Value = analogRead(DIM_input); // reading values from 0 - 5 V and converts to int value from 0 to 1023 .
  Battery12V_Value = analogRead(Battery12V_input);
  BatteryPackVolt_Value = analogRead(BatteryPackVolt_input);
  Temperature_Value = analogRead(Temperature_input);
  }

void equations() {
  //equations for DIMMING LED cockpit lights:
  // 950 --> 100, 1023 --> 255; ???? resistor divider - 300 Ohm and 23 Ohm pot. , 5V --> Max 5 V (1023), Min 4.64 V (950)
  DIM=3.49*(1023-DIM_Value);   //min 0 max 73 --> multiply faktor 3.49 --> min 0 max 255
  //equations for Fuel gauge:
  // 703 --> 0, 1023 --> 255    BUT - GND of the pack is insulated from the circuits!!!
  BatteryPackVolt = (BatteryPackVolt_Value-703)*255/(1023-703);
  //equations for Temperature gauge:
  // 0 --> 0, 1023 --> 255
  Temperature = Temperature_Value*255/1023;
  // sending PWM singal temperature gauge and fuel gauge:
}

//function for reading via I2C the SOC from Nano
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    //Serial.print(c);         // print the character
  }
  SOC2 = Wire.read();    // receive byte as an integer
  //Serial.println(SOC2);         // print the integer
}

// set all relay OFF - for begin sequence!
void relayOFF() {
  digitalWrite(TurnLeft, HIGH);
  digitalWrite(TurnRight, HIGH);
  digitalWrite(ParkLight, HIGH);
  digitalWrite(DayLight, HIGH);
  digitalWrite(LongLight, HIGH);
  digitalWrite(StopLight, HIGH);
  digitalWrite(Horn, HIGH);
  digitalWrite(DefrostWindow, HIGH);
  digitalWrite(BackwardLight, HIGH);
  digitalWrite(WiperRear, HIGH);
  digitalWrite(FogLight, HIGH);
  digitalWrite(MotorLight, HIGH);
  digitalWrite(WindowWater, HIGH);
  digitalWrite(Wiper1Alternate, HIGH);
  digitalWrite(Wiper1, HIGH);
  digitalWrite(Wiper2, HIGH);
  }

void testDisplay() {
  j0.setValue(100);
  j1.setValue(100);
  j2.setValue(100);
  j3.setValue(100);
  j4.setValue(100);
  j5.setValue(100);
  j6.setValue(100);
  j7.setValue(100);
  j8.setValue(100);
  j9.setValue(100);
  j10.setValue(0); 
  j11.setValue(100);
  j12.setValue(100);
  j13.setValue(100);
  j14.setValue(100);
  j15.setValue(100);
  j16.setValue(100);
  delay(500);
   j0.setValue(0);
  j1.setValue(0);
  j2.setValue(0);
  j3.setValue(0);
  j4.setValue(0);
  j5.setValue(0);
  j6.setValue(0);
  j7.setValue(0);
  j8.setValue(0);
  j9.setValue(0);
  j10.setValue(100); 
  j11.setValue(0);
  j12.setValue(0);
  j13.setValue(0);
  j14.setValue(0);
  j15.setValue(0);
  j16.setValue(0);
  }
