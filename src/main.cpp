#include <Arduino.h>
#include <SPI.h>
#include <SPIFlash.h>
#include "DS18B20.h"


#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire); 
SPIFlash flash(8 , 0xEF40);

#ifdef DEBUG
#define DEBUGPRINT(x) Serial.print(x)
#define DEBUGPRINTARG(x,y) Serial.print(x,y)
#define DEBUGPRINTLN(x) Serial.println(x)
#define DEBUGPRINTLNARG(x,y) Serial.println(x, y)
#else
#define DEBUGPRINT(x)
#define DEBUGPRINTARG(x,y)
#define DEBUGPRINTLN(x)
#define DEBUGPRINTLNARG(x,y)
#endif

void printFlashContent(){
int counter = 0;
while(counter<=256){
    DEBUGPRINTARG(flash.readByte(counter++), HEX);
    DEBUGPRINT('.');}
}

void printFlashUID(){
uint8_t* MAC = flash.readUniqueId(); 
for (uint8_t i=0;i<8;i++) { 
    DEBUGPRINTARG(MAC[i], HEX); 
    DEBUGPRINT(' '); }
    DEBUGPRINTLN();

}

//String printFlashUID(){
//String hexstring;
//uint8_t* MAC = flash.readUniqueId(); 
//for (uint8_t i=0;i<8;i++) { 
//    hexstring += String(MAC[i], HEX);
//    hexstring += String(' ');
//}
//    return String(hexstring);
//}

uint16_t analogOversample(const byte analogPin) {
  uint16_t reading = 0;
  int i = 0;
  while (i < 8)
  {
    i++;
    reading = reading + analogRead(analogPin);
  }
  return reading >> 3;
}

// Reads Analog input and calculates result as Volts according to Voltage divider Resistors
// R1 is the high side of the divider Resistor values in kOhms
float readBattVoltageFloat(float r1, float r2, int analogPin, float vcc) {
  float batt_v = 0.0;
  for (int i = 0; i < 16; i++) analogRead(analogPin); // first readings are not accurate
  uint16_t ADCreading = analogOversample(analogPin);

    batt_v = ((ADCreading * vcc) / 1024.0) / (r2/(r1+r2));
return batt_v;
}

int readBattVoltageInt(float r1, float r2, int analogPin, float vcc) {
  for (int i = 0; i < 16; i++) analogRead(analogPin); // first readings are not accurate
  uint16_t ADCreading = analogOversample(analogPin);

  //Convert analog ADC reading to millivolts
  int _voltage = (((ADCreading * vcc / 1024.0) * (r1 + r2) / r2) * 100);
  return _voltage;
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void setup() {
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(115200);
    delay(100);
    // Check FLASH
    Serial.println(F("Starting"));
    DEBUGPRINT(F("Start flash...")); 
    if (flash.initialize())
      DEBUGPRINTLN(F("Init Flash OK!"));
    else
      DEBUGPRINTLN(F("Init Flash FAIL!"));
    DEBUGPRINT(F("DeviceID: "));
    DEBUGPRINTLNARG(flash.readDeviceId(), HEX);
    DEBUGPRINTLN();
    DEBUGPRINTLN(F("Flash content:"));
    printFlashContent();   
    DEBUGPRINTLN();
    DEBUGPRINT(F("DeviceUID: "));
    printFlashUID();
    //DEBUGPRINT(printFlashUID()); 
    // Check 1-Wire Temp sensor
    sensor.begin();
    sensor.requestTemperatures();
    while (!sensor.isConversionComplete());  // wait until sensor is ready
    DEBUGPRINT(F("Temp: "));
    DEBUGPRINTLN(sensor.getTempC());
    // Read Battery voltage
    DEBUGPRINT(F("Batt: "));
    DEBUGPRINTLN(readBattVoltageInt(100.0, 100.0, A3, 3.3));
    DEBUGPRINTLN(readBattVoltageFloat(100.0, 100.0, A3, 3.3));
}

void loop() {
  // put your main code here, to run repeatedly:

}