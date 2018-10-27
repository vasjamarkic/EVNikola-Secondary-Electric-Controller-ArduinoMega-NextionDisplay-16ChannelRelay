# EVNikola-Secondary-Electric-Controller-ArduinoMegaPRO-NextionDisplay-i2cNano-16ChannelRelay
With Arduino Mega Robodyn Mini and Nextion touch display creating controller for lights and other actuators in VW T3.

Still to do:
- using the blinknig relay
- recheck the code for wipers
- place the circuit box under the driver's seat
- place the battery and fusses under the co-driver's seat //insulate with plastic or plexi 
- wire the switches and microswitches in Nicola (test again the functions - wipers!!!)
- dismount the top dashboard
- get the front wipers and test it
- install the pwm module for the heating sistem. 
- clean and test the fan - putting the electric heater ??? use a fuse for that ...
- install the remaining front lights
- wire the key ignition switch with f10 and f6 and to MCU and Emsiso.
- place the wires for all dashboard switches (blue and white thin wires)
- solid the N.C. micro switch for handbrake
- solder the white LED and resistors, make connection types and place it in the dash, via PWM output. 

What was done:
- ordered the PCB board in Svet-elektronike (44 €), aquired.
- mount and assemble the pcb mega sheild (solder holders, mosfet, bridges, resistors, condensators, diodes, tube fuse ...)
- installed the rear LED lights (the small for targa not yet) (90 €)
- bought misc. material in EVJ (240 + 45 €)
- bought 4 relays (35 €), also from ebay for wipers
- wired the back lights and placed terminal under driver seat
- placed wires from fuses compartment (passenger seat) to central circuit compartments (driver seat)
- unmount fuel tank
- bought hexagonal spacers M3 >22 mm lenght (metal or nylon) - done, but will use 16-channel relay module!!! 
- mapping of all wires, matching with entities, dashboard, instrument board, ...
- Put in order de Relays with his functions.
- Choosed the fuses # for actuators 
- updated the drawing VISIO
- emergency stop lights with activating both turning lights - SOLVED with array and switch-case
- replaced the mega2560 with the new one because I accidently bricked 12V to the input pin --> UART is damaged, the procesor is not   
  communicating with pC anymore!
- wired the front-mask! 
- defined the 2d pack of fuses for PWM, Radio, cabin lights, other, Emsiso controller
- wired the wires and second fuse unit (f11 ÷ f16)
- placed wires from second pack fuses (f11 ÷ f16) to dashboard
- 


by Vasja Markič,
CEO at Elec3go Institute, SI
