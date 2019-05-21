See: https://biotronics.eu/what-is-qiWELLNESS

Supported commands:
* freq [freq] [pwm] 
Generate impulse signal. 
 [freq] - frequency (0.00Hz - 1kHz). Set multiple frequency by 100 e.g. 13.5Hz = 1350
 [ |pwm] - duty cycle (0.0 - 100%) e.g. 5.5

* sfreq
Stop generate signal. Change to DC
 
* pwm [pwm]
Duty cycle 
  [pwm] - duty cycle (0.0 - 100%) e.g. 5.5
  
* chp [0|1|~]
Change polarity

* beep

* mode  -  change mode to (eap, eav, veg) or show current mode.

* vegcalib

* eavcalib




Communicats:

:btn - Button in active electrode was pressed in diagnose circuit

:estart

:vstart

:stop

:vxxxx  -  Voltage [V] was measured in VEGATEST diagnose circuit 

:exxxx  -  Voltage [V] was measured in EAV diagnose circuit

:cxxxx yyyy  - Current [uA] and pwm [%] was measured in EAP therapy circuit




