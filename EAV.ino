#include <Arduino.h>      // For eclipse IDE only

#define analogInPin  A3               // Analog input form transoptor
#define oneBitEqivalentVoltage 3.2258 // 1023bits = 3.3V => 1bit = 3.2258mV
#define maximumInputTresholdVoltage 3000 // e.g.: 3000mV => 3.3V - 0.30V
#define minimumSignalTresholdVolatge 1000 //

long inputVoltage = 0;        // value read from the pot
boolean started = false;
long outputValue = 0;        // value output to the PWM (analog out)

void startGraph();

void setup() {
  analogReference(INTERNAL); //Change to 3.3V External arduino nano reference
  Serial.begin(115000);
  Serial.println(100);
  Serial.println(0);
}

void loop() {

  inputVoltage = analogRead(analogInPin)*oneBitEqivalentVoltage;

  if (inputVoltage>minimumSignalTresholdVolatge && !started){
    started = true;
    startGraph();
  }

  if (inputVoltage<=minimumSignalTresholdVolatge){
    started=false;
    delay(100);
  }
  if (started) {
    outputValue = map(inputVoltage, 0, 3300, 0, 100);
    Serial.println(outputValue);
  }

  delay(10);
}

void startGraph(){

    Serial.println(100);
    for (int i=0; i<100; i++) Serial.println(0);

}

