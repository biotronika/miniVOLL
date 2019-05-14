/* miniVOLL is firmware software for Arduino miniVOLL device
 * Author: Chris Czoba  krzysiek@biotronika.pl
 *
 * For more see: biotronics.eu
 *
 * Calibrate by pressing button in the pen electrode or short
 * the electrodes when connect USB cable (powering on).
 */

#include <Arduino.h>      // For eclipse IDE only
#include <EEPROM.h>

//#define SERIAL_DEBUG

#define SOFT_VER "2019-05-14"
#define HRDW_VER "NANO 1.1"

#define analogInPin	A3						// Analog input from optocoupler from EAV circuit
#define buzzerPin	6						// Signal buzzer
#define analogCurrentPin A5 				// Analog input current in passive electrode: R330 with 3.3V ref. (II-2)
#define electrodePin 10  					// Active electrode EAP signal (II-3)
#define polarizationPin 2					// Polarization relay pin

#define ONE_BIT_VOLTAGE 3.2258 				// 1023bits = 3.3V => 1bit = 3.2258mV
#define ONE_BIT_CURRENT 9.775 				// 1023bits = 3.3V on 330R => 1bit = 9.775uA
#define MAX_INPUT_THRESHOLD_VOLTAGE 2500 	// Is used for calibration purpose. e.g.: 3000mV => 3.3V - 0.30V
#define MIN_INPUT_THRESHOLD_VOLTAGE 900 	// Threshold of vstart: communicate, default 900 mV
//#define MAX_INPUT_THRESHOLD_CURRENT 10000 	// default 10mA
#define MIN_INPUT_THRESHOLD_CURRENT 15 		// Threshold of cstart: communicate, default 15uA

#define DEF_FREQ 1000						// Start pulse frequency, default 10Hz (1.00Hz - 1kHz)
#define DEF_PWM 5.0							// Start duty cycle, default 5% (0.0% - 100.0%)

#define eeAddress 0							//  Address of calibration value

#define WELCOME_SCR "bioZAP interpreter welcome! See http://biotronics.eu"
#define MAX_CMD_PARAMS 3    				// Count of command parameters

#define MODE_EAP 0							// Mode electro(acu)punture (EAP)
#define MODE_EAV 1							// Mode Voll's electroacupuncture (EAV)
#define MODE_VEG 2							// Mode vegatest

volatile int mode = MODE_EAP;						// Default mode
long inputVoltage = 0;
boolean started = false;
int outputEAVPrecentage = 0;

int maximumInputVoltage = MAX_INPUT_THRESHOLD_VOLTAGE;


String inputString = "";                // Buffer string to hold incoming serial data
boolean stringComplete = false;         // Whether the string is complete
String param[MAX_CMD_PARAMS];           // param[0] = cmd name
float pwm = DEF_PWM; 					// Duty cycle
volatile unsigned int current = 0;		// Current measurement in therapy circuit
boolean lastPromptSign = true;			// What kind of char was send recently

void beep( int millis );
void calibrate();
void freq( unsigned long freq, float pwm );
void getParams( String &inputString );
int executeCmd( String cmdLine );
void checkPrompt ();


void setup() {

	// Pin configuration
	pinMode(buzzerPin, OUTPUT);
	pinMode(electrodePin, OUTPUT);
	pinMode(polarizationPin, OUTPUT);

	// Change to 3.3V External Arduino NANO reference
	analogReference( INTERNAL );

	Serial.begin( 115000 );

	beep(100);

	if (analogRead(analogInPin)*ONE_BIT_VOLTAGE > maximumInputVoltage) {

		// New calibration
		calibrate();

	} else {

		// Set 100% voltage value from last calibration value
		EEPROM.get(eeAddress, maximumInputVoltage);
	}

    Serial.println(WELCOME_SCR);
    Serial.print("Device miniVOLL ");
    Serial.print(HRDW_VER);
    Serial.print(" ");
    Serial.println(SOFT_VER);
    Serial.println(">");
    lastPromptSign=true;

    // Start therapy circuit with default parameters
    freq( DEF_FREQ, pwm );
}

void loop() {

  //Serial command
  if ( stringComplete ) {

	executeCmd(inputString);
	checkPrompt();
	Serial.print('>'); //Cursor for new command
	lastPromptSign=true;


	//Clear command string
	inputString = "";
	stringComplete = false;

  }

  if ( mode != MODE_EAP ) {

	  // Measurement and filter for therapy circuit
	  // Use 8 samples instead of one
	  inputVoltage = 0;
	  for( int i=0; i<8; i++ ){
		  inputVoltage +=analogRead(analogInPin);
	  }
	  inputVoltage = (inputVoltage >> 3) * ONE_BIT_VOLTAGE;

  }


  // Start measure of diagnose circuit
  if ( mode != MODE_EAP &&
		inputVoltage > MIN_INPUT_THRESHOLD_VOLTAGE &&
		inputVoltage < maximumInputVoltage  &&
		!started ) {

	  	  started = true;
	  	  checkPrompt();
	  	  Serial.println(":vstart");
  }

  // Start measure of therapy circuit
  if ( mode == MODE_EAP && current >= MIN_INPUT_THRESHOLD_CURRENT  && !started  ) {

	      started = true;
	      checkPrompt();
	      Serial.println(":cstart");

  }

  // Button press detection in therapy circuit
  if ( mode != MODE_EAP && !started && inputVoltage > maximumInputVoltage ){
	  	  checkPrompt();
	  	  Serial.println(":btn");
	  	  delay(100);
  }

  // End of measure detection in therapy circuit
  if ( mode != MODE_EAP && inputVoltage <= MIN_INPUT_THRESHOLD_VOLTAGE ){
	  	  started=false;
	  	  delay(100);
  }

  // Sent value to serial in diagnose circuit
  if ( mode != MODE_EAP  && started ) {

	if ( inputVoltage > maximumInputVoltage ) inputVoltage = maximumInputVoltage;
    outputEAVPrecentage = map(inputVoltage, MIN_INPUT_THRESHOLD_VOLTAGE , maximumInputVoltage, 0 , 1000);

    //Check pressed button (more then 95%)
    if ( outputEAVPrecentage > 950 ) {

    		checkPrompt();
    		Serial.println(":save");
    		delay(100);

    } else {
    		checkPrompt();
    		Serial.print(":v");
    		Serial.println( outputEAVPrecentage );
    		delay(20);

    }

  }



  // Sent value to serial in therapy circuit
  if ( (mode == MODE_EAP) && started){

	  //Check pressed button (more then 90%)
	  if (current < MIN_INPUT_THRESHOLD_CURRENT) {

		  started=false;
		  delay(100);

	  } else {

		  Serial.print(":c");
		  Serial.println(current);
		  delay(20);

	  }

  }



} //loop

void checkPrompt (){

	if (lastPromptSign) Serial.println();
	lastPromptSign=false;

}


void beep(int millis){

	digitalWrite(buzzerPin,HIGH);
	delay(millis);
	digitalWrite(buzzerPin,LOW);

}

void calibrate(){
// Calibrate device to 100% in EAV mode with shorted electrodes

	inputVoltage = analogRead(analogInPin)*ONE_BIT_VOLTAGE;

	for (int i=0;i<100;i++){
		inputVoltage = (3*inputVoltage + analogRead(analogInPin)*ONE_BIT_VOLTAGE)/4;
	}

	EEPROM.put(eeAddress, inputVoltage);
	maximumInputVoltage = inputVoltage;
	beep(400);

	inputVoltage =0;

}

void freq(unsigned long freq, float pwm){
/* Function generating signal on pin PD10
 * freq put *100, e.g. 10Hz = 1000
 * pwm is duty cycle e.g. 1% = 1.0
 */
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
    OCR1A = F_CPU / ( prescaler * (freq / 100.0) ) - 1;

    // Set PWM duty cycle
    OCR1B = (pwm/100) * OCR1A;

    // Enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);

    sei();

}

ISR (TIMER1_COMPA_vect){
// Measure of current in EAP during impulse.
	if (mode == MODE_EAP) {
		unsigned int ui  = analogRead(analogCurrentPin);
					 ui += analogRead(analogCurrentPin);
					 ui += analogRead(analogCurrentPin);
					 ui += analogRead(analogCurrentPin);
					 ui = ui >> 2; //Filter: use 4 samples instead of one

		current = ONE_BIT_CURRENT * ui * 1.25;
		//1.25 is correction of reverse current on Zener diode ~100uA on 330R
//TODO: May I should change 330R to 0R, and correction factor to 1.0 - to consider that!
	}

}


void freqStop(){

	cli();

	TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0;
    OCR1B = 0;

    sei();
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

    Serial.print(inChar); //echo

    // If the incoming character is a newline, set a flag complete
    if (inChar == '\n') {
      stringComplete = true;
    }

  }

}

void getParams(String &inputString){
  for (int i=0; i<MAX_CMD_PARAMS; i++) param[i]="";

  int from =0;
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


	if (param[0].charAt(0)=='#') {
// Comment

    	;

    } else if (param[0]=="vegatest"){
// Mode veagatest

    	mode = MODE_VEG;
    	Serial.println("OK");

    } else if (param[0]=="eav"){
// Mode EAV

		mode = MODE_EAV;
		Serial.println("OK");

    } else if (param[0]=="eap"){
// Mode electroacupuncture

		mode = MODE_EAP;
		Serial.println("OK");

    } else if (param[0]==""){
// Emptyline

    	;

    } else if (param[0]=="mode"){
// Show current mode
    	switch (mode) {
			case MODE_EAP :	Serial.println("eap"); break;
			case MODE_EAV : Serial.println("eav"); break;
			case MODE_VEG : Serial.println("vegatest"); break;
    	}
		//Serial.println("OK");

    } else if (param[0]=="sfreq"){
// Stop freq function

    	freqStop();
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


    } else if (param[0]=="pwm"){
// Change pwm duty cycle
    	pwm =constrain(param[1].toFloat(), 0, 100);
    	Serial.println("OK");


    } else if (param[0]=="freq"){
// Generate square signal - freq [freq] [pwm]

    	if (param[2] != "") {
    		pwm = constrain( param[2].toFloat(), 0, 100) ;
    	}

    	freq(param[1].toInt(), pwm);
    	Serial.println("OK");


    }  else {
//Unknown command
    	Serial.println("Unknown command: "+param[0]);
    }

return 0;
}

