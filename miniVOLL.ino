/* miniVOLL is firmware software for Arduino miniVOLL device
 * Author: Chris Czoba  krzysiek@biotronika.pl
 *
 * For more see: biotronics.eu
 *
 */

#include <Arduino.h>      // For eclipse IDE only

#define SOFT_VER "2019-02-16"
#define HRDW_VER "NANO 1.0"

#define analogInPin	A3	// Analog input form transoptor
#define buzerPin	D3	// For signal buzer
#define oneBitEqivalentVoltage 3.2258 // 1023bits = 3.3V => 1bit = 3.2258mV
#define maximumInputThresholdVoltage 3000 // Is used for calibration purpose. e.g.: 3000mV => 3.3V - 0.30V
#define minimumSignalThresholdVolatge 1171 // Threshold of 0%

long inputVoltage = 0;
boolean started = false;
long outputValue = 0;

void startGraph();

void setup() {
  analogReference(INTERNAL); //Change to 3.3V External Arduino NANO reference
  Serial.begin(115000);
  Serial.println(100);
  Serial.println(0);
}

void loop() {

  inputVoltage = analogRead(analogInPin)*oneBitEqivalentVoltage;

  if (inputVoltage>minimumSignalThresholdVolatge && !started){
    started = true;
    startGraph();
  }

  if (inputVoltage<=minimumSignalThresholdVolatge){
    started=false;
    delay(100);
  }
  if (started) {
    outputValue = map(inputVoltage, 0, 3300, minimumSignalThresholdVolatge, 100);
    Serial.println(outputValue);
  }

  delay(10);
}

void startGraph(){

    Serial.println(100);
    for (int i=0; i<100; i++) Serial.println(0);

}

