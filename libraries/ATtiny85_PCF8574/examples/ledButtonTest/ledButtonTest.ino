#include <ATtiny85_PCF8574.h>

#define SDA_PIN 0 // pin on ATtiny85; pullup (10kOm to VCC)
#define SCL_PIN 2 // pin on ATtiny85; pullup (10kOm to VCC)
#define PCF8574_ADDRESS (0x20) // A0 - A1 - A2 (pins on PCF8574) are connected to GND

#define LED_PIN 1 // pin on PCF8574; cathode; resistor >= 330 Om
#define BUTTON_PIN 2 // pin on PCF8574; pullup (10kOm to VCC)

ATtiny85_PCF8574 expander;

unsigned int buttonTimer = 0;

void setup() {
  expander.begin(PCF8574_ADDRESS);
  
  expander.setBitMode(BUTTON_PIN, INPUT_BIT);
  expander.setBitMode(LED_PIN, OUTPUT_BIT);
}

void loop() {
  if(millis() - buttonTimer >= 50){
    buttonTimer = millis(); 
    
    byte bitValue = expander.getBit(BUTTON_PIN);
    //byte byteValue = expander.getByte();
  
    if(bitValue == 0){
      expander.setBit(LED_PIN, LOW);
    }
    else{
      expander.setBit(LED_PIN, HIGH);
    }
        
  } 
}
