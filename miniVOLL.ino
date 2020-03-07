/* miniVOLL is free and opensource firmware software for qiWELLNESS device
 * Author: Chris Czoba  krzysiek@biotronika.pl
 * Copyleft: 11/2019
 *
 * For more see: biotronics.eu in qiWELLNESS section
 *
 * Software works with PCB & prototypes version of qiWELLNESS device
 *
 */

#include <Arduino.h>      // For eclipse IDE only
#include <EEPROM.h>

//#define SERIAL_DEBUG

#define SOFT_VER "2020-03-07"
#define HRDW_VER "NANO 1.3"

#define diagnoseReadPin A3						// Analog input from optocoupler from EAV/vegatest circuit

#define buzzerPin 6								// Signal buzzer

#define analogCurrentPin A7 					// Analog input current in passive electrode: R330 with 3.3V ref. (II-2)

#define electrodePin 10  						// Active electrode EAP signal (II-3)
#define polarizationPin 2						// Polarization relay pin

#define modeRelayPin 11							// eav=LOW veg=HIGH for PCB board - control relay
#define modeTherapyDiagnoseRealyPin 3			// For jur's version.  therapy=LOW diagnose=HIGH
#define switchModePin 12						// eav=LOW veg=HIGH for prototypes board - read switch state

#define ONE_BIT_VOLTAGE 3.2258 					// 1023bits = 3.3V => 1bit = 3.2258mV
#define ONE_BIT_CURRENT 9.775 					// 1023bits = 3.3V on 330R => 1bit = 9.775uA

int 	MAX_EAV_INPUT_THRESHOLD_VOLTAGE = 2200; //2500; // Is used for calibration purpose. e.g.: 3000mV => 3.3V - 0.30V
#define MIN_EAV_INPUT_THRESHOLD_VOLTAGE 700 	// Threshold of estart communicate, default 700 mV

//unsigned int max_eav_input_treshold_voltage = MAX_EAV_INPUT_THRESHOLD_VOLTAGE / ONE_BIT_VOLTAGE;

int 	MAX_VEG_INPUT_THRESHOLD_VOLTAGE = 1000; // Is used for vegatest e.g.: 1000mV
#define MIN_VEG_INPUT_THRESHOLD_VOLTAGE 300 	// Threshold of vstart communicate, default 300 mV



#define MAX_EAP_OUTPUT_THRESHOLD_RMS_CURRENT 480// Above that value current be limited, default 480uA
#define MIN_EAP_OUTPUT_THRESHOLD_CURRENT 20 	// Threshold of cstart communicate, default 20uA

unsigned int max_eap_output_threshold_rms_current = MAX_EAP_OUTPUT_THRESHOLD_RMS_CURRENT / ONE_BIT_CURRENT;
unsigned int min_eap_output_threshold_current     = MIN_EAP_OUTPUT_THRESHOLD_CURRENT     / ONE_BIT_CURRENT;



#define MIN_ION_OUTPUT_THRESHOLD_CURRENT 100		// Threshold of istart communicate, default 100uA

unsigned int min_ion_output_threshold_current     = MIN_ION_OUTPUT_THRESHOLD_CURRENT     / ONE_BIT_CURRENT;



#define START_FREQ 1000							// Start pulse frequency, default 10Hz (1000) range: 1.00Hz - 1kHz
#define START_PWM 0.05							// Start duty cycle, default 5%, range: 0% - 100% (0 - 1.0)

#define EAV_CALIBRATION_ADDRESS 0				// EEPROM address of calibration value (long)
#define VEG_CALIBRATION_ADDRESS 4

#define WELCOME_SCR "bioZAP interpreter welcome! See http://biotronics.eu"
#define MAX_CMD_PARAMS 3    					// Number of command parameters

#define MODE_EAP 0								// Mode electro(acu)punture (EAP)
#define MODE_EAV 1								// Mode Voll's electroacupuncture (EAV) & RYODORAKU
#define MODE_VEG 2								// Mode VEGATEST
#define MODE_ION 4								// Mode ionophorese & zapper up to 1kHz???

#define BOARD_TYPE_UNKNOWN 0			//Device board types. PCB has relay instead of eav/vegatest switch
#define BOARD_TYPE_PROTOTYPES 1
#define BOARD_TYPE_PCB 2

#define DEVICE_TYPE_ADDRESS 10			// EEPROM address to save board type



String inputString = "";                // Serial, buffer string to hold incoming serial data
boolean stringComplete = false;         // Serial, whether the string is complete
String param[MAX_CMD_PARAMS];           // Serial, all parameters of commands. param[0] = command

float pwm = START_PWM;				// Duty cycle could be limited because of max range. set_pwm has saved original value.
float currentPwm = pwm; 			// Actual duty cycle of current
volatile unsigned int current = 0;		// Current measurement in therapy circuit. Is used in interruption
unsigned int ui;
int mode = MODE_EAP;			// Start default mode. Is used in interruption
long inputVoltage = 0;
boolean act = 0;						// Iontophoresis active
unsigned int currentFreq = 0;			// Current set frequency
boolean started = false;				// Measurement is started (after reached threshold)
int outputEAVPrecentage = 0;

byte boardType = BOARD_TYPE_UNKNOWN;	// Device board type

boolean lastPromptSign = true;			// What kind of char was send recently
boolean buttonPressed = false;			// Is button in active electrode of diagnose circuit pressed

//Function prototypes
void beep(int millis);
void beepWrong();
void calibrate();
void freq(unsigned long freq, float pwm);
void getParams(String &inputString);
int  executeCmd(String cmdLine);
void checkPrompt ();
long measureVoltage();
void activateOutput();



void setup() {

	// Pin configuration
	pinMode(buzzerPin, OUTPUT);
	pinMode(electrodePin, OUTPUT);
	pinMode(polarizationPin, OUTPUT);
	pinMode(modeRelayPin, OUTPUT);
	pinMode(modeTherapyDiagnoseRealyPin, OUTPUT);

	pinMode(switchModePin,INPUT_PULLUP);

	// Change to 3.3V external Arduino NANO reference
	analogReference( INTERNAL );

	Serial.begin( 115000 );

	beep(100);

	//Define board type of device: prototypes or pcb
	boardType = EEPROM.read(DEVICE_TYPE_ADDRESS);

	switch (boardType){
	case BOARD_TYPE_PROTOTYPES: break;
	case BOARD_TYPE_PCB: break;
	default:
		boardType = BOARD_TYPE_UNKNOWN;
	}



	//EAV startup calibration
	if (analogRead(diagnoseReadPin) * ONE_BIT_VOLTAGE > MAX_EAV_INPUT_THRESHOLD_VOLTAGE) {

		// New calibration
		calibrate();

	} else {

//TODO: remove it from that place
		// Set 100% voltage value from last calibration value
		EEPROM.get(EAV_CALIBRATION_ADDRESS, MAX_EAV_INPUT_THRESHOLD_VOLTAGE);
	}

    Serial.println(WELCOME_SCR);
    Serial.print("Device miniVOLL ");
    Serial.print(HRDW_VER);
    Serial.print(" ");

	switch (boardType){
	case BOARD_TYPE_PROTOTYPES: Serial.print("prototypes board"); break;
	case BOARD_TYPE_PCB: Serial.print("PCB board"); break;
	default:
		boardType = BOARD_TYPE_UNKNOWN;
		Serial.print("unknown board type"); break;
	}

    Serial.print(" ");
    Serial.println(SOFT_VER);

    // Prompt sign
    Serial.println(">");
    lastPromptSign=true;

    // Start therapy circuit with default parameters
    freq( START_FREQ, pwm );
}

void loop() {

  //Serial command support
  if ( stringComplete ) {

	executeCmd(inputString);

	//Prompt for new command
	checkPrompt();
	Serial.print('>');
	lastPromptSign=true;

	//Clear command string
	inputString = "";
	stringComplete = false;

  }

  //First voltage measurement in diagnose circuit for EAV and VEG modes
  if ( (mode == MODE_EAV) || (mode == MODE_VEG) ) {

	  inputVoltage = measureVoltage();
  }


  // Start measure of vegatest (1.2V) diagnose circuit
  if ( mode == MODE_VEG && inputVoltage > MIN_VEG_INPUT_THRESHOLD_VOLTAGE && !started ) {

		if ((boardType==BOARD_TYPE_PROTOTYPES) && (digitalRead(switchModePin)==LOW)) {

			Serial.println("Switch device to VEGATEST mode!");
			beepWrong();
			started = false;

		} else {

		//Second measurement in VEG mode after the signal stabilizes
			delay(20);
			inputVoltage=measureVoltage();

			started = true;

			checkPrompt();
			Serial.println(":vstart");

		}

}

  // Start measure of eav (3.3V) diagnose circuit (used for ryodoraku as well)
  if ( mode == MODE_EAV && 	inputVoltage > MIN_EAV_INPUT_THRESHOLD_VOLTAGE && !started ) {

		if ((boardType==BOARD_TYPE_PROTOTYPES) && (digitalRead(switchModePin)==HIGH)) {

			Serial.println("Switch device to EAV mode!");
			beepWrong();
			started = false;

		} else {

		    //Second measurement in EAV mode after the signal stabilizes
			delay(20);
			inputVoltage=measureVoltage();

			started = true;

			checkPrompt();
			Serial.println(":estart");

		}

  }


  // Start measurement of therapy circuit in EAP mode
  if ( mode == MODE_EAP && (current * ONE_BIT_CURRENT) >= MIN_EAP_OUTPUT_THRESHOLD_CURRENT  && !started  ) {

		started = true;

		checkPrompt();
		Serial.println(":cstart");


  }

  // Start measurement of therapy circuit in ionophoreses
  if ( mode == MODE_ION && (current * ONE_BIT_CURRENT) >= MIN_ION_OUTPUT_THRESHOLD_CURRENT  && !started  ) {

		started = true;

		checkPrompt();
		Serial.println(":istart");

  }

  // End of measure detection in eav diagnose circuit
  if ( mode == MODE_EAV && started && inputVoltage <= MIN_EAV_INPUT_THRESHOLD_VOLTAGE ){
	  	  started=false;

	  	  checkPrompt();
	  	  Serial.println(":stop");

	  	  delay(100);
  }


  // End of measure detection in vegatest diagnose circuit
  if ( mode == MODE_VEG && started && inputVoltage <= MIN_VEG_INPUT_THRESHOLD_VOLTAGE ){
	  	  started=false;

	  	  checkPrompt();
	  	  Serial.println(":stop");

	  	  delay(100);
  }


  // Sent value to serial in EAV diagnose circuit
  if ( mode == MODE_EAV  && started ) {

	if ( inputVoltage > MAX_EAV_INPUT_THRESHOLD_VOLTAGE ) inputVoltage = MAX_EAV_INPUT_THRESHOLD_VOLTAGE;
    outputEAVPrecentage = map(inputVoltage, MIN_EAV_INPUT_THRESHOLD_VOLTAGE , MAX_EAV_INPUT_THRESHOLD_VOLTAGE, 0 , 1000);

    		buttonPressed=false;

    		checkPrompt();
    		Serial.print(":e");
    		Serial.println( outputEAVPrecentage );

    		delay(20);
  }


  // Sent value to serial in VEG diagnose circuit
  if ( mode == MODE_VEG  && started ) {

	if ( inputVoltage > MAX_VEG_INPUT_THRESHOLD_VOLTAGE ) inputVoltage = MAX_VEG_INPUT_THRESHOLD_VOLTAGE;
    outputEAVPrecentage = map(inputVoltage, MIN_VEG_INPUT_THRESHOLD_VOLTAGE , MAX_VEG_INPUT_THRESHOLD_VOLTAGE, 0 , 1000);

    		buttonPressed=false;

    		checkPrompt();
    		Serial.print(":v");
    		Serial.println( outputEAVPrecentage );

    		delay(20);
  }


  // Sent value to serial in therapy circuit in EAP mode
  if ( (mode == MODE_EAP) && started){

	  //Check pressed button (more then 90%)
	  if ( int(current * ONE_BIT_CURRENT) < MIN_EAP_OUTPUT_THRESHOLD_CURRENT) {

		  started=false;

	  	  checkPrompt();
	  	  Serial.println(":stop");

		  delay(100);

	  } else {

		  // Control maximum level of RMS current by reducing duty cycle (PWM)
			if (( current * pwm ) > max_eap_output_threshold_rms_current ) {

				currentPwm = max_eap_output_threshold_rms_current / current ;

//TODO: Change control function because of wrong, oscillation behavior - use proportional controller algorithm

				// Set PWM duty cycle
				cli();
				OCR1B = currentPwm * OCR1A;
				sei();

			} else {

				if (currentPwm != pwm) {

					currentPwm = pwm;

					// Set PWM duty cycle
					cli();
					OCR1B = currentPwm * OCR1A;
					sei();
				}
			}



		  checkPrompt();
		  Serial.print(":c");
		  Serial.print(int(current * ONE_BIT_CURRENT));   //Serial.println(current);
		  Serial.print(" ");
		  Serial.println(currentPwm*100,1);

		  delay(20);
	  }
  }


  // Sent value to serial in therapy circuit in IPH mode
  if ( (mode == MODE_ION) && started){

	  //Check pressed button (more then 90%)
	  if ( int(current * ONE_BIT_CURRENT) < MIN_ION_OUTPUT_THRESHOLD_CURRENT) {

		  started=false;

	  	  checkPrompt();
	  	  Serial.println(":stop");

		  delay(100);

	  } else {

		  checkPrompt();
		  Serial.print(":i");
		  Serial.print(int(current * ONE_BIT_CURRENT));   //Serial.println(current);
		  Serial.print(" ");
		  Serial.println(pwm*100,1);

		  delay(20);
	  }
  }

} //loop

long measureVoltage(){
// Measurement and filter for diagnose circuit
// Use 8 samples instead of one

	  long inputVoltage = 0;

	  for( int i=0; i<8; i++ ){
		  inputVoltage +=analogRead(diagnoseReadPin);
	  }
	  inputVoltage = (inputVoltage >> 3) * ONE_BIT_VOLTAGE;
	return inputVoltage;
}


void checkPrompt (){

	if (lastPromptSign) Serial.println();
	lastPromptSign=false;

}

void beep(int millis){

	digitalWrite(buzzerPin,HIGH);
	delay(millis);
	digitalWrite(buzzerPin,LOW);

}

void beepWrong() {
//Something goes wrong voice signal
	beep(200);
	delay(200);
	beep(200);
	delay(200);
	beep(1000);
}

void calibrate(){
// Calibrate device to 100% in EAV mode and 100% in VEG mode with shorted electrodes
// Calibration method: see README.md

int beginingRelayState = digitalRead(modeRelayPin);

	//EAV
	digitalWrite(modeRelayPin, LOW);
	delay(2000);

	//Save board type in EEPROM
	if (digitalRead(switchModePin)==LOW)
		EEPROM.write(DEVICE_TYPE_ADDRESS, BOARD_TYPE_PROTOTYPES );
	else
		EEPROM.write(DEVICE_TYPE_ADDRESS, BOARD_TYPE_PCB);

	boardType = EEPROM.read(DEVICE_TYPE_ADDRESS);

	inputVoltage = analogRead(diagnoseReadPin)*ONE_BIT_VOLTAGE;

	for (int i=0;i<100;i++){
		inputVoltage = (3*inputVoltage + analogRead(diagnoseReadPin)*ONE_BIT_VOLTAGE)/4;
	}

	EEPROM.put(EAV_CALIBRATION_ADDRESS, inputVoltage);
	MAX_EAV_INPUT_THRESHOLD_VOLTAGE = inputVoltage;
	beep(1000);

	inputVoltage =0;

	//Veagtest, switch automaticlly
	digitalWrite(modeRelayPin, HIGH);

	//Vegatest, wait for manual switch change
	delay(5000);


	for (int i=0;i<100;i++){
		inputVoltage = (3*inputVoltage + analogRead(diagnoseReadPin)*ONE_BIT_VOLTAGE)/4;
	}

	EEPROM.put( VEG_CALIBRATION_ADDRESS, inputVoltage);
	MAX_VEG_INPUT_THRESHOLD_VOLTAGE = inputVoltage;

	beep(1000);
	delay(500);
	beep(1000);

	inputVoltage =0;


	digitalWrite(modeRelayPin,beginingRelayState);

}




void freq(unsigned long freq, float pwm){
/* Function generating signal on pin PD10
 * freq put *100, e.g. 10Hz = 1000
 * curr_pwm is duty cycle e.g. 1% = 1.0
 */
	unsigned long _freq = constrain(freq, 1, 10000);
	cli();

	uint16_t prescaler = 1;

	ICR1  = 0xFFFF;
    //TCNT1 = 0x01FF;
	TCNT1 = 0x0000;

	//Set mode 15 (16bit) (Fast PWM - TOP OCR1A, non-inverting mode)
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (1 << WGM10);


    // Choose the best prescaler for the frequency
    if (freq < 100) {

    	prescaler = 1024;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS12)   | (1 << CS11)   | (1 << CS10);

    } else if (freq < 400){

    	prescaler = 256;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS12)   | (0 << CS11)   | (0 << CS10);

    } else if (freq < 3100) {

    	prescaler = 64;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (0 << CS12)   | (1 << CS11)   | (1 << CS10);

    } else if (freq < 25000) {

    	prescaler = 8;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (0 << CS12)   | (1 << CS11)   | (0 << CS10);

    } else  {

    	prescaler = 1;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (0 << CS12)   | (0 << CS11)   | (1 << CS10);

    }



    // Set the nearest applicable frequency
    OCR1A = F_CPU / ( prescaler * (_freq / 100.0) ) - 1;

    // Set PWM duty cycle
    OCR1B = pwm * OCR1A;

    // Enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);

    sei();
}


// Measure of current in EAP & IPH during impulse (not RMS value).
ISR (TIMER1_COMPA_vect){

	 ui  = analogRead(analogCurrentPin);
	 ui = ui << 2;
	 ui+= ui;
	 ui = ui >>2;
	 //ui += analogRead(analogCurrentPin);
	 //ui += analogRead(analogCurrentPin);
	 //ui += analogRead(analogCurrentPin);
	 //ui += analogRead(analogCurrentPin); // 1.25 = 5/4 is correction of reverse current on Zener diode ~100uA on 330R
	 //ui = ui >> 2; //Filter: use 4 (5) samples instead of one

	 current = ui;


//TODO: May I should change 330R to 0R, and correction factor to 1.0 - to consider that!
//		1.25 is correction of reverse current on Zener diode ~100uA on 330R



}



void dc(){

	cli();

	TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0;
    OCR1B = 0;

    sei();

    //Current measurement 10 times per second only
    freq(1000, 1);

    //Write HIGH or LOW depends on active parameter
    //digitalWrite(electrodePin, act);

    //set freq=0 (dc)
    currentFreq= 0;
    currentPwm=1;
    pwm=1;
}

void activateOutput(boolean _act){
//Activate therapy circuit and set global act parameter

	if (_act) {

		freq(currentFreq, currentPwm);

	} else {

		//dc();
		digitalWrite(electrodePin, LOW);
		current = 0;
	}
	act = _act;


}

//Serial commands///////////////////////////////////////////////////////////////////////////

void serialEvent() {

  while (Serial.available()) {

	// Get the new byte:
    char inChar = (char)Serial.read();

    // Add it to the inputString:
    if (inChar!='\r'){
      inputString += inChar;
    }

    // Echo
    Serial.print(inChar);

    // If the incoming character is a newline, set a flag complete
    if (inChar == '\n') {
      stringComplete = true;
    }

  }

}

void getParams(String &inputString){

  for (int i=0; i<MAX_CMD_PARAMS; i++) param[i]="";

  int from = 0;
  int to = 0;

  for (int i=0; i<MAX_CMD_PARAMS; i++){
    to = inputString.indexOf(' ',from); //Find SPACE

    if (to==-1) {
      to = inputString.indexOf('\n',from); //Find NL #10
      if (to>0) param[i] = inputString.substring(from,to);
      param[i].trim();
      break;
    }

    if (to>0) param[i] = inputString.substring(from,to);
    param[i].trim();
    from = to+1;
  }
}

int executeCmd(String cmdLine){
// Main interpreter function

	getParams(cmdLine);

	buttonPressed=false;

	if (param[0].charAt(0)=='#') {
// Comment

    	;


	 } else if (param[0]=="calib"){
// EAV & Vegatest calibration circuit, maximum voltage = 100%

    	calibrate();
    	Serial.println(MAX_VEG_INPUT_THRESHOLD_VOLTAGE);
    	Serial.println("OK");


    } else if (param[0]=="veg"){
// Mode veagatest

    	dc();
    	mode = MODE_VEG;

    	digitalWrite(modeRelayPin, HIGH);
    	digitalWrite(modeTherapyDiagnoseRealyPin, HIGH);

    	EEPROM.get(VEG_CALIBRATION_ADDRESS, MAX_VEG_INPUT_THRESHOLD_VOLTAGE);
    	if ((boardType==BOARD_TYPE_PROTOTYPES) && (digitalRead(switchModePin)==LOW)) {
    		Serial.println("Switch device to VEGATEST mode!");
    		beepWrong();
    	}

    	Serial.println("OK");

    } else if (param[0]=="eav"){
// Mode EAV

    	dc();
    	activateOutput(0);

		mode = MODE_EAV;
		digitalWrite(modeRelayPin, LOW);
		digitalWrite(modeTherapyDiagnoseRealyPin, HIGH);

		EEPROM.get(EAV_CALIBRATION_ADDRESS, MAX_EAV_INPUT_THRESHOLD_VOLTAGE);
		if ((boardType==BOARD_TYPE_PROTOTYPES) && (digitalRead(switchModePin)==HIGH)) {
			Serial.println("Switch device to EAV mode!");
			beepWrong();
		}

		Serial.println("OK");

    } else if (param[0]=="eap"){
// Mode electroacupuncture

		mode = MODE_EAP;
		digitalWrite(modeTherapyDiagnoseRealyPin, LOW);
		activateOutput(1);

		Serial.println("OK");

    } else if (param[0]=="ion"){
// Mode iontophoresis

		mode = MODE_ION;
		digitalWrite(modeTherapyDiagnoseRealyPin, LOW);
		activateOutput(1);

		Serial.println("OK");

    } else if (param[0]==""){
// Emptyline

    	;

    } else if (param[0]=="mode"){
// Show current mode
    	switch (mode) {
			case MODE_EAP :	Serial.println("eap"); break;
			case MODE_EAV : Serial.println("eav"); break;
			case MODE_VEG : Serial.println("veg"); break;
			case MODE_ION : Serial.println("ion"); break;
    	}
		//Serial.println("OK");

    } else if (param[0]=="sfreq"){
//DEPRECATED Stop freq, change to DC signal

    	dc();
    	Serial.println("OK");

    } else if (param[0]=="dc"){
// Change to DC signal

    	dc();
    	Serial.println("OK");


    } else if (param[0]=="beep"){
// Beep [time_ms]

        beep(param[1].toInt());
        Serial.println("OK");


    } else if (param[0]=="chp"){
// Change output signal polarity

    	if (param[1]=="~") {
    		digitalWrite(polarizationPin, !digitalRead(polarizationPin));
    	} else {
    		digitalWrite(polarizationPin, param[1].toInt());

    	}
    	Serial.println("OK");

    } else if (param[0]=="act"){
// Change output signal polarity

    	activateOutput( param[1].toInt() );

    	Serial.println("OK");


    } else if (param[0]=="pwm"){
// Change or show curr_pwm duty cycle

    	if (param[1] != "") {

    		float param1 = param[1].toFloat();


    		if (param1==100) {

				dc();
				Serial.println("Direct current");
				Serial.println("OK");

    		} else {
    			// In ionotophoresis use 50, 90
				if (mode==MODE_ION) {
					if (param1<=75.0) {
						param1=50;
					} else {
						param1=90;
					}

				}

				pwm = constrain(param1, 0, 100) / 100; // 0.0 - 1.0 = 0 - 100%
				currentPwm = pwm;

				freq(currentFreq,currentPwm);

				Serial.println("OK");
    		}

    	} else {

    		Serial.println(pwm,1); //Show pwm
    	}


    } else if (param[0]=="freq"){
// Generate square signal - freq [freq] [curr_pwm]

    	if (param[2] != "") {
    		pwm = constrain( param[2].toFloat(), 0, 100) / 100;
    		currentPwm = pwm;
    	}

    	currentFreq = param[1].toInt();

    	freq(currentFreq, currentPwm);
    	Serial.println("OK");


    }  else {
//Unknown command
    	Serial.println("Unknown command: "+param[0]);
    }

return 0;
}

