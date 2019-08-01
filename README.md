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

* eav  -  change to EAV mode.

* veg  -  change to Vegatest mode.

* vegcalib  -  Calibration of Veagtest diagnose circuit. Electrode connect by 2.1V Zener diode.

* eavcalib  -  Calibration of EAV diagnose circuit.


#### TODO:

* pd [0|1]  -  Point Detection. The biological active points (BAP) auto detection by generating voice signal in Vegatest and EAV modes. 


### Supported communicats:

* :btn - Button in active electrode was pressed in diagnose circuit.

* :estart  -  Start of measure in EAV mode.

* :vstart  -  Start of measure in Vegatest mode.

* :cstart  -  Start of current measure in EAP mode. 

* :stop  -  Stop of measure. 

* :vxxxx  -  Voltage [V] was measured in Vegatest diagnose circuit. 

* :exxxx  -  Voltage [V] was measured in EAV diagnose circuit.

* :cxxxx yyyy  - Current [uA] and pwm [%] was measured in EAP therapy circuit.




