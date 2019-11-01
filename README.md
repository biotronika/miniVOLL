## miniVOLL & qiWELLNESS

One Arduino ino file for beginners. Full functional software for miniVOLL and qiWELLNESS device. 
See: [https://biotronics.eu](https://biotronics.eu/what-is-qiWELLNESS) or for Polish:  [https://biotronika.pl](https://biotronika.pl/what-is-qiWELLNESS) 


### To compile code and upload using Arduino IDE:
1. Download miniVOLL.ino file and put it into miniVOLL folder (it must has exactly that name). 
2. Open miniVOLL.ino file in Arduino IDE.
3. Check if you have EEPROM library already installed (Sketch->Include Library-> see on list: EEPROM).
4. Configure board: Tools->Board->Arduino Nano  and **Tools->Processor->ATmega328P (Old Bootloader)**.
5. Install Arduino Nano driver - **biotronics.eu** website: [CH341SER.ZIP]( https://biotronika.pl/sites/default/files/2016-12/CH341SER.ZIP).
6. Configure serial port. Plug USB cable to PC and miniVOLL or Adruino Nano board. Then Tools->Port->select proper COM port.
7. Compile and upload. Sketch->Upload. Wait until on down side of Arduino IDE window see **Done uploading**.

### Download qiWELLNESS software for PC:
[https://biotronics.eu/download](https://biotronics.eu/download)


### Supported commands:
* freq [freq] [pwm]  -  Generate impulse signal. [freq] - frequency (0.00Hz - 1kHz). Set multiple frequency by 100 e.g. 13.5Hz : `freq 1350`  
pwm - duty cycle, it is optional parameter (0.0 - 100%) e.g.: `freq 1350 5.5`

* sfreq  -  Stop generate signal. Change signal to DC
 
* pwm [pwm]  -  Duty cycle.  [pwm] - duty cycle (0.0 - 100%) e.g. 5.5% : `pwm 5.5`
  
* chp [0|1|~]  -  Change polarity

* beep [ms]  -  Make by ms milliseconds signal.  

* mode  -  show current mode:  eap, eav, veg

* eap  -  change to EAP mode.

* eav  -  change to EAV & RYODORAKU mode.

* veg  -  change to Vegatest mode.

* calib  -  Calibration of EAV & Veagtest diagnose circuit. Remember of change EAV/VEGATES switch in right position. PCB hardware makes it automatically.


#### TODO:

* pd [0|1]  -  Point Detection. The biological active points (BAP) auto detection by generating voice signal in Vegatest and EAV modes. 


### Supported communicats:

* (REMOVED) :btn - Button in active electrode was pressed in diagnose circuit. Works up to 2019-07-27 version.

* :estart  -  Start of measure in EAV mode.

* :vstart  -  Start of measure in Vegatest mode.

* :cstart  -  Start of current measure in EAP mode. 

* :stop  -  Stop of measure. 

* :vxxxx  -  Voltage [V] was measured in Vegatest diagnose circuit. 

* :exxxx  -  Voltage [V] was measured in EAV diagnose circuit.

* :cxxxx yyyy  - Current [uA] and pwm [%] was measured in EAP therapy circuit.


### Hardware calibration method: (You have to do once at least with new device with firmware 2019-11-01 and newer):
1. Turn qiWELLNESS device of by putting out USB cable out.
2. Be sure that internal ION battery is charged. Remove it and recharge the battery if voltage is less then 3.6V
3. Connect together active and auxiliary socket by inserting the banana cable to diagnose circuit.
4. If you have prototypes based device put mode switch to EAV/RYODORAKU position. In PCB version it makes automatically.
5. Insert USB cable and turn the device on.
6. Wait until one long beep signal stops and change mode switch to VEGATEST position. You do nothing with PCB device.
7. Wait for two long beep signal and put USB cable off. Put the banana cable out from diagnose circuit. 
8. You have done :)


