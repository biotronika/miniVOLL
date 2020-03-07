## miniVOLL & qiWELLNESS

One Arduino ino file for beginners. Full functional software for miniVOLL and qiWELLNESS device. 
See: [https://biotronics.eu](https://biotronics.eu/what-is-qiWELLNESS) or for Polish:  [https://biotronika.pl](https://biotronika.pl/what-is-qiWELLNESS) 


### To compile code and upload using Arduino IDE:
1. Download miniVOLL.ino file and put it into miniVOLL folder (it must has exactly that name). 
2. Open miniVOLL.ino file in Arduino IDE.
3. Check if you have EEPROM library already installed (Sketch->Include Library-> see on list: EEPROM).
4. Configure board: Tools->Board->Arduino Nano  and **Tools->Processor->ATmega328P (Old Bootloader)** or **ATmega328P** (it depends on your PCB board).
5. Install Arduino Nano driver - **biotronics.eu** website: [CH341SER.ZIP]( https://biotronika.pl/sites/default/files/2016-12/CH341SER.ZIP).
6. Configure serial port. Plug USB cable to PC and miniVOLL or Adruino Nano board. Then Tools->Port->select proper COM port.
7. Compile and upload. Sketch->Upload. Wait until on down side of Arduino IDE window see **Done uploading**.

### Download qiWELLNESS software for PC:
[https://biotronics.eu/download](https://biotronics.eu/download)


### Supported commands:
* freq [freq| ] [pwm| ]  -  Generate impulse signal. [freq] - frequency (0.00Hz - 50.00Hz). Set multiple frequency by 100 e.g. 13.5Hz = `freq 1350`  
pwm - duty cycle, it is optional parameter (0.0 - 100%) e.g. 15.5Hz 5.5% = `freq 1350 5.5`. `freq 0` or `freq [freq] 100` works as `ds` command. 
freq without parameter shows set frequency and pwm.

* (REMOVED) sfreq  -  Stop generate signal. Change signal to DC. See **ds**.

* ds - Generate DS (Direct Current) signal in therapy circuit. 5 - 40 V.
 
* pwm [pwm]  -  Duty cycle.  [pwm] - duty cycle (0.0 - 100%) e.g. 5.5% : `pwm 5.5`, In **ion** mode you can use pwm as: 100, 90, 50 only.
  
* chp [0|1|~]  -  Change polarity, 1 - active (red) electrode is positive, 0 - negative, ~ - change polarity to the opposite.

* beep [ms]  -  Make by ms milliseconds signal.  

* mode  -  Show current mode:  eap, eav, veg, ion.

* eap  -  Change to EAP mode. Default mode with 10Hz frequency and active output after turning on.

* eav  -  Change to EAV & RYODORAKU mode.

* veg  -  Change to Vegatest mode.

* ion  -  Change to iontophoresis, micro-iontophoresis & zapper mode.

* act [0|1] - Activate output in iontophoresis and EAP modes, 1 - active, 0 - inactive, EAP active (1) mode is set after turning on (default mode).

* calib  -  Calibration of EAV & Vegatest diagnose circuit. Remember of change EAV/VEGATES switch to right position. See calibration method chapter. PCB hardware makes it automatically.

#### TODO:

* pd [0|1]  -  Point Detection. The biological active points (BAP) auto detection by generating voice signal in EAP mode. 


### Supported communicats:

* (REMOVED) :btn - Button in active electrode was pressed in diagnose circuit. Works up to 2019-07-27 version.

* :estart  -  Start of measure in EAV (Voll's electroacupuncture) mode.

* :vstart  -  Start of measure in VEG (vegatest) mode.

* :cstart  -  Start of current measure in EAP (elecropuncture) mode. 

* :istart  -  Start of current measure in ION (ionophorese) mode. 

* :stop  -  Stop of measure. 

* :vxxxx  -  Level 0-100.0% measured in vegatest diagnose circuit. 

* :exxxx  -  Level 0-100.0% measured in EAV diagnose circuit.

* :cxxxx yyyy  - Current [uA] and duty cycle (pwm) [%] was measured in EAP therapy circuit.

* :ixxxx yyyy  - Current [uA] and duty cycle (pwm) [%] was measured in IONOPHORESE therapy circuit.


### Hardware calibration method: 
You have to do at least once with new device with firmware 2019-11-01 and newer.
1. Turn qiWELLNESS device of by putting USB cable out.
2. Be sure that internal ION battery is charged. Remove it and recharge the battery if voltage is less then 3.6V
3. Connect together active and auxiliary sockets by inserting the banana cable into diagnose circuit.
4. If you have prototypes based device put mode switch to EAV/RYODORAKU position. In PCB version it makes automatically.
5. Insert USB cable and turn the device on.
6. Wait until one long beep signal stops and change mode switch to VEGATEST position. You have 2 seconds only. You do nothing with PCB device.
7. Wait for two long beep signal and put USB cable off. Put the banana cable out from diagnose circuit. 
8. You have done :)


