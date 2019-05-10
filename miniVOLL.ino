/* miniVOLL is firmware software for Arduino miniVOLL device
 * Author: Chris Czoba  krzysiek@biotronika.pl
 *
 * For more see: biotronics.eu
 *
 * Calibrate by pressing button in the pen electrode or short the electrodes when connect USB cable (power on)
 *
 */

#include <Arduino.h>      // For eclipse IDE only
#include <EEPROM.h>

//#define SERIAL_DEBUG


#define SOFT_VER "2019-05-10"
#define HRDW_VER "NANO 1.0"

#define analogInPin	A3		// Analog input from optocoupler from EAV circuit
#define buzzerPin	6		// Signal buzzer
#define analogCurrentPin A5 // Analog input current in passive electrode: R330 with 3.3V ref. (II-2)
#define electrodePin 10  	// Active electrode signal (II-3)
#define polarizationPin 2	// Polarization relay pin

#define ONE_BIT_VOLTAGE 3.2258 				// 1023bits = 3.3V => 1bit = 3.2258mV
#define ONE_BIT_CURRENT 9.775 				// 1023bits 330R with REF=3.3V => 1bit = 9.775uA
#define MAX_INPUT_THRESHOLD_VOLTAGE 2500 	// Is used for calibration purpose. e.g.: 3000mV => 3.3V - 0.30V
#define MIN_INPUT_THRESHOLD_VOLTAGE 900 	// Threshold of 0%

#define eeAddress 0			//  Address of calibration value

#define WELCOME_SCR "bioZAP interpreter welcome! See http://biotronics.eu"
#define MAX_CMD_PARAMS 3    // Count of command parameters

long inputVoltage = 0;
boolean started = false;
int outputValue = 0;

int maximumInputVoltage = MAX_INPUT_THRESHOLD_VOLTAGE;


String inputString = "";                // A string to hold incoming serial data
boolean stringComplete = false;         // whether the string is complete
String param[MAX_CMD_PARAMS];           // param[0] = cmd name
float pwm = 1.0; 					    // Duty cycle, default 1% (0.0% - 100.0%)
boolean currentMesurement = false;
volatile unsigned int current = 0;

void startCmd();
void saveCmd();
void btnCmd();
void beep(int millis);
void calibrate();
void freq(unsigned long aFreq, float aPWM);
void getParams(String &inputString);
int executeCmd(String cmdLine);

void setup() {
	pinMode(buzzerPin, OUTPUT);
	pinMode(electrodePin, OUTPUT);
	pinMode(polarizationPin, OUTPUT);


	analogReference(INTERNAL); //Change to 3.3V External Arduino NANO reference

	Serial.begin(115000);

	beep(100);

	if (analogRead(analogInPin)*ONE_BIT_VOLTAGE > maximumInputVoltage) {
		//New calibration
		calibrate();
	} else {

		//Set 100% voltage value
		EEPROM.get(eeAddress, maximumInputVoltage);
	}

    Serial.println(WELCOME_SCR);
    Serial.print("Device miniVOLL ");
    Serial.print(HRDW_VER);
    Serial.print(" ");
    Serial.println(SOFT_VER);
    Serial.println(">");

	//Serial.println(100);
	//Serial.println(0);

    //cli();  // disable global interrupts

    //freq(78,10);

    //sei();  // enable global interrupts:

    //while(1);
}

void loop() {

  //inputVoltage = analogRead(analogInPin)*oneBitEqivalentVoltage;

  if (stringComplete) {

	executeCmd(inputString);
	Serial.print('>'); //Cursor for new command

	//Clear command string
	inputString = "";
	stringComplete = false;

  }

  // Use 8 samples instead of one
  inputVoltage = 0;
  for(int i=0; i<8; i++){
	  inputVoltage +=analogRead(analogInPin);
  }
  inputVoltage = (inputVoltage >> 3) * ONE_BIT_VOLTAGE;


  if (inputVoltage>MIN_INPUT_THRESHOLD_VOLTAGE && !started && inputVoltage < maximumInputVoltage){
    started = true;
    startCmd();
    delay(20);
  }

  if(!started && inputVoltage > maximumInputVoltage){
	  btnCmd();
	  delay(20);
  }

  if (inputVoltage<=MIN_INPUT_THRESHOLD_VOLTAGE){
    started=false;
    delay(100);
  }
  if (started) {
	if (inputVoltage > maximumInputVoltage) inputVoltage = maximumInputVoltage;
    outputValue = map(inputVoltage, MIN_INPUT_THRESHOLD_VOLTAGE , maximumInputVoltage, 0 , 1000);
    delay(20);

    //Check pressed button (more then 90%)
    if (outputValue>900) {
    	saveCmd();
    } else {
    	Serial.println(outputValue);
    }

  }
  //maxCurrent = analogRead(analogCurrentPin);
  delay(20);
}

void startCmd(){
    Serial.println(":start");
    //Serial.println(100);
    //for (int i=0; i<100; i++) Serial.println(0);
}

void saveCmd(){
	Serial.println(":save");
}

void btnCmd(){
	Serial.println(":btn");
}

void beep(int millis){

	digitalWrite(buzzerPin,HIGH);
	delay(millis);
	digitalWrite(buzzerPin,LOW);

}

void calibrate(){
// Calibrate device to 100% with shorted electrodes

	inputVoltage = analogRead(analogInPin)*ONE_BIT_VOLTAGE;

	for (int i=0;i<100;i++){
		inputVoltage = (3*inputVoltage + analogRead(analogInPin)*ONE_BIT_VOLTAGE)/4;
	}

	EEPROM.put(eeAddress, inputVoltage);
	maximumInputVoltage = inputVoltage;
	beep(400);

	inputVoltage =0;

}

void freq(unsigned long aFreq, float aPWM){
/* Function generating signal on pin PD10
 * aFreq put *100, e.g. 10Hz = 1000
 * aPWM is duty cycle in percentage *10, e.g. 1% = 10
 */
	cli();

	uint16_t prescaler = 1;

	ICR1  = 0xFFFF;
    //TCNT1 = 0x01FF;
	TCNT1 = 0x0000;

	//Set mode 15 (16bit) (Fast PWM - TOP OCR1A, non-inverting mode)
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (1 << WGM10);


    // Choose the best prescaler for the frequency
    if (aFreq < 100) {

    	prescaler = 1024;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS12)   | (1 << CS11)   | (1 << CS10);

    } else if (aFreq < 400){

    	prescaler = 256;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS12)   | (0 << CS11)   | (0 << CS10);

    } else if (aFreq < 3100) {

    	prescaler = 64;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (0 << CS12)   | (1 << CS11)   | (1 << CS10);

    } else if (aFreq < 25000) {

    	prescaler = 8;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (0 << CS12)   | (1 << CS11)   | (0 << CS10);

    } else  {

    	prescaler = 1;
    	TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (0 << CS12)   | (0 << CS11)   | (1 << CS10);

    }



    //Set the nearest applicable frequency
    OCR1A = F_CPU /( prescaler * (aFreq / 100.0))-1; //31250 //2Hz

    //Set PWM duty cycle
    OCR1B = (aPWM/100) * OCR1A;

    // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);
    //TIMSK1 != (1 << ICIE1);
    sei();

}

ISR (TIMER1_COMPA_vect){
//TODO
  current = ONE_BIT_CURRENT * analogRead(analogCurrentPin);
}


void freqStop(){
	//cli();
	TCCR1A = 0;
    TCCR1B = 0;
    OCR1A=0;
    OCR1B=0;
    //sei();
}

//Serial commands///////////////////////////////////////////////////////////////////////////

void serialEvent() {

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    if (inChar!='\r'){
      inputString += inChar;
    }

    Serial.print(inChar); //echo

    // if the incoming character is a newline, set a flag
    if (inChar == '\n') {
      stringComplete = true;
    }

  }

}

void getParams(String &inputString){
  for (int i=0; i<MAX_CMD_PARAMS; i++) param[i]="";

  int from =0;
  int to =0;
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


    } else if (param[0]==""){
// Emptyline

    	;
    } else if (param[0]=="curr"){
// Start current measurement

    	Serial.println(current );
    	Serial.println("OK");


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

