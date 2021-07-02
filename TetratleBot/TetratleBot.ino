#define F_CPU 8000000UL

#include <avr/eeprom.h>
#include <tiny_IRremote.h>
#include <ATtiny85_PCF8574.h>
#include <GyverButton.h>

#define VCC_CONST 1.08

#define INIT_KEY 10

#define INIT_KEY_EEPROM_ADDRESS 15
#define FLAG_EEPROM_ADDRESS 17
#define LINE_EEPROM_ADDRESS 18
#define MODE_EEPROM_ADDRESS 19
#define ARRAY_EEPROM_ADDRESS 20

// ---------- Pins on ATtiny85 ----------
#define LEFT_PHOTO_SENSOR_PIN A2 // pull up (10kOm to VCC)
#define RIGHT_PHOTO_SENSOR_PIN A3 // pull up (10kOm to VCC)
#define IR_RECV_PIN 1 // pull up (10kOm to VCC)
#define SDA_PIN 0 // pull up (10kOm to VCC)
#define SCL_PIN 2 // pull up (10kOm to VCC)

// ---------- Pins on PCF8574 (1) ----------
#define PCF8574_1_ADDRESS (0x27) // A0 - A1 - A2 - VCC
#define LINE_SENSOR_PIN 0
#define BTN_BACK_PIN 1
#define LED_SENSORS_PIN 2 //cathode; resistor >= 330 Om
#define LED_STATE_PIN 4 //cathode; resistor >= 330 Om
#define BTN_CONFIG_PIN 5
#define BTN_RIGHT_PIN 6
#define BTN_LEFT_PIN 7

// ---------- Pins on PCF8574 (2) ----------
#define PCF8574_2_ADDRESS (0x21) // A0 - VCC; A1 - A2 - GND
#define LEFT_MOTOR_IN1_PIN 0
#define LEFT_MOTOR_IN2_PIN 2
#define RIGHT_MOTOR_IN3_PIN 3
#define RIGHT_MOTOR_IN4_PIN 1
#define LEFT_MOTOR_EN1_PIN 7
#define RIGHT_MOTOR_EN2_PIN 6

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8

#define IR_REMOTE_MODE 1
#define TENDRILS_MODE 2
#define PHOTO_SENSORS_MODE 3
#define LINE_SENSOR_MODE 4

#define LONG_BLINK_TIME 600
#define MIDDLE_BLINK_TIME 300
#define SHORT_BLINK_TIME 100
#define PAUSE_BLINK_TIME 200

#define IR_SIGNAL_PERIOD 150

#define LED_ON 0
#define LED_OFF 1

#define NO_OBSTACLES 0
#define LEFT_OBSTACLES 1
#define RIGHT_OBSTACLES 2
#define BACK_OBSTACLES 3

// 0, 1, 2, 3, 4, ↑, ↓, ←, →, OK
int arrayOfCodes[10];
// = {0xFFFFA857, 0xFFFF807F, 0x40BF, 0xFFFFC03F, 0x20DF, 0xFFFFD02F, 0x708F, 0x8F7, 0xFFFF8877, 0xFFFFB04F};
bool state = true;
byte currentMode = 0;
byte speed = 150;
bool loading = true;
unsigned int glowTimePeriod = 0; 
unsigned int pauseTimePeriod = 0;
byte blinkCounter = 0;
unsigned long blinkTimer = 0;

int codeValue = 0;
unsigned long irTimer = 0;
bool isStopped = false;

bool useTendrils = false;
byte obstacles = NO_OBSTACLES;
unsigned long obstaclesTimer = 0;

byte basicLightingLevel = 0;

bool blackLine = true;
bool turnDirectionChanged = true;
byte turnDirection = FORWARD_RIGHT;

IRrecv irRecv(IR_RECV_PIN);
decode_results code;
ATtiny85_PCF8574 expander1;
ATtiny85_PCF8574 expander2;
GButton buttonConfig;
GButton leftTendril;
GButton rightTendril;
GButton backTendril;

void setMode(byte _mode, bool _state = false);


void setup() {
  irRecv.enableIRIn();
  
  expander1.begin(PCF8574_1_ADDRESS);
  expander2.begin(PCF8574_2_ADDRESS);

  expander1.setBitMode(LED_STATE_PIN, OUTPUT_BIT);
  expander1.setBitMode(LED_SENSORS_PIN, OUTPUT_BIT);
  expander1.setBitMode(BTN_CONFIG_PIN, INPUT_BIT);
  expander1.setBitMode(BTN_LEFT_PIN, INPUT_BIT);
  expander1.setBitMode(BTN_RIGHT_PIN, INPUT_BIT);
  expander1.setBitMode(BTN_BACK_PIN, INPUT_BIT);
  expander1.setBitMode(LINE_SENSOR_PIN, INPUT_BIT);

  expander2.setBitMode(LEFT_MOTOR_IN1_PIN, OUTPUT_BIT);
  expander2.setBitMode(LEFT_MOTOR_IN2_PIN, OUTPUT_BIT);
  expander2.setBitMode(RIGHT_MOTOR_IN3_PIN, OUTPUT_BIT);
  expander2.setBitMode(RIGHT_MOTOR_IN4_PIN, OUTPUT_BIT);
  expander2.setBitMode(LEFT_MOTOR_EN1_PIN, OUTPUT_BIT);
  expander2.setBitMode(RIGHT_MOTOR_EN2_PIN, OUTPUT_BIT);

  setMove(STOP);
  expander1.setBit(LED_SENSORS_PIN, LED_OFF);
  expander1.setBit(LED_STATE_PIN, LED_OFF);

  delay(100);

  if(eeprom_read_byte(INIT_KEY_EEPROM_ADDRESS) != INIT_KEY || !expander1.getBit(BTN_CONFIG_PIN)){
    byte i = 0;
    bool isCodeValid = false;
    
    for(byte j = 0; j < 5; j++){
      expander1.setBit(LED_STATE_PIN, LED_ON);
      delay(100);
      expander1.setBit(LED_STATE_PIN, LED_OFF);
      delay(100);
    }

    while(true){
      buttonConfig.tick(!expander1.getBit(BTN_CONFIG_PIN));
      
      if(irRecv.decode(&code)){
        if(code.value != REPEAT){
          codeValue = code.value;
          isCodeValid = true;
          expander1.setBit(LED_STATE_PIN, LED_OFF);
          delay(100);
          expander1.setBit(LED_STATE_PIN, LED_ON);
        }
        
        irRecv.resume();
      }

      if(buttonConfig.isClick() && isCodeValid){
        arrayOfCodes[i] = codeValue;
        isCodeValid = false;
        expander1.setBit(LED_STATE_PIN, LED_OFF);
        delay(100);
        expander1.setBit(LED_STATE_PIN, LED_ON);
        delay(100);
        expander1.setBit(LED_STATE_PIN, LED_OFF);
        
        if(i == (sizeof(arrayOfCodes) / sizeof(*arrayOfCodes)) - 1) break;
        
        i++;
      }
    }

    for(byte j = 0; j < 5; j++){
      expander1.setBit(LED_STATE_PIN, LED_ON);
      delay(100);
      expander1.setBit(LED_STATE_PIN, LED_OFF);
      delay(100);
    }

    eeprom_update_byte(FLAG_EEPROM_ADDRESS, useTendrils);
    eeprom_update_byte(LINE_EEPROM_ADDRESS, blackLine);
    eeprom_update_byte(MODE_EEPROM_ADDRESS, currentMode);
    eeprom_write_block((void*)&arrayOfCodes, ARRAY_EEPROM_ADDRESS, sizeof(arrayOfCodes));
    eeprom_write_byte(INIT_KEY_EEPROM_ADDRESS, INIT_KEY);
  }
 
  useTendrils = eeprom_read_byte(FLAG_EEPROM_ADDRESS);
  blackLine = eeprom_read_byte(LINE_EEPROM_ADDRESS);
  eeprom_read_block((void*)&arrayOfCodes, ARRAY_EEPROM_ADDRESS, sizeof(arrayOfCodes));

  setMode(eeprom_read_byte(MODE_EEPROM_ADDRESS));
}


void loop() {
  buttonConfig.tick(!expander1.getBit(BTN_CONFIG_PIN));

  if(buttonConfig.hasClicks()){
    switch(buttonConfig.getClicks()){
      case 1:
        state = !state;
        state ? setBlink(LONG_BLINK_TIME, PAUSE_BLINK_TIME, 1) : setBlink(SHORT_BLINK_TIME, PAUSE_BLINK_TIME, 1);
        setMove(STOP);
        loading = true;
        break;
      case 2:
        currentMode < 4 ? setMode(++currentMode) : setMode(1);
        break;
      case 3:
        currentMode > 1 ? setMode(--currentMode) : setMode(4);
        break;
      case 4:
        useTendrils = !useTendrils;
        eeprom_update_byte(FLAG_EEPROM_ADDRESS, useTendrils);
        useTendrils ? setBlink(LONG_BLINK_TIME, PAUSE_BLINK_TIME, 1) : setBlink(SHORT_BLINK_TIME, PAUSE_BLINK_TIME, 1);
        break;
      case 5:
        blackLine = !blackLine;
        eeprom_update_byte(LINE_EEPROM_ADDRESS, blackLine);
        blackLine ? setBlink(SHORT_BLINK_TIME, PAUSE_BLINK_TIME, 2) : setBlink(LONG_BLINK_TIME, PAUSE_BLINK_TIME, 2);
        break;
    }
  }

  if(buttonConfig.isHolded()){
    setBlink(LONG_BLINK_TIME, PAUSE_BLINK_TIME, getBatteryLevel());
  }

  if(irRecv.decode(&code)){
    irTimer = millis();
    
    codeValue = code.value;

    if(codeValue == arrayOfCodes[0]){
      useTendrils = !useTendrils;
      eeprom_update_byte(FLAG_EEPROM_ADDRESS, useTendrils);
      useTendrils ? setBlink(LONG_BLINK_TIME, PAUSE_BLINK_TIME, 1) : setBlink(SHORT_BLINK_TIME, PAUSE_BLINK_TIME, 1);     
    }
    else if(codeValue == arrayOfCodes[1]){
      setMode(IR_REMOTE_MODE);
    }
    else if(codeValue == arrayOfCodes[2]){
      currentMode = TENDRILS_MODE;
      setMode(TENDRILS_MODE);
    }
    else if(codeValue == arrayOfCodes[3]){
      setMode(PHOTO_SENSORS_MODE);
    }
    else if(codeValue == arrayOfCodes[4]){
      setMode(LINE_SENSOR_MODE);
    }
    else if(codeValue == arrayOfCodes[9] && currentMode != IR_REMOTE_MODE){
      state = !state;
      state ? setBlink(LONG_BLINK_TIME, PAUSE_BLINK_TIME, 1) : setBlink(SHORT_BLINK_TIME, PAUSE_BLINK_TIME, 1);
      setMove(STOP);
      loading = true;
    }

    irRecv.resume();
  }

  blinkListener();

  leftTendril.tick(!expander1.getBit(BTN_LEFT_PIN));
  rightTendril.tick(!expander1.getBit(BTN_RIGHT_PIN));
  backTendril.tick(!expander1.getBit(BTN_BACK_PIN));

  if(leftTendril.state() || rightTendril.state() || backTendril.state())
    expander1.setBit(LED_SENSORS_PIN, LED_ON);
  else
    expander1.setBit(LED_SENSORS_PIN, LED_OFF);

  setSpeed(speed);
    
  switch(currentMode){
    case IR_REMOTE_MODE:
      irRemoteMode();
      break;
    case TENDRILS_MODE:
      if(state) tendrilsMode();
      break;
    case PHOTO_SENSORS_MODE:
      if(state) photoSensorsMode();
      break;
    case LINE_SENSOR_MODE:
      if(state) lineSensorMode();
      break;
  }

  codeValue = 0;
}


void setMode(byte _mode, bool _state = false){
  currentMode = _mode;
  eeprom_update_byte(MODE_EEPROM_ADDRESS, currentMode);
  setBlink(MIDDLE_BLINK_TIME, PAUSE_BLINK_TIME, currentMode);
  state = _state;
  setMove(STOP);
  loading = true;
}


void irRemoteMode(){
  speed = 150;
  
  if(codeValue == arrayOfCodes[5])
    setMove(FORWARD);
  else if(codeValue == arrayOfCodes[6])
    setMove(BACKWARD);
  else if(codeValue == arrayOfCodes[7])
    setMove(LEFT);
  else if(codeValue == arrayOfCodes[8])
    setMove(RIGHT);
  else if(codeValue == arrayOfCodes[9])
    setMove(STOP);
  
  if(useTendrils){
    if(!isStopped && (leftTendril.state() || rightTendril.state() || backTendril.state())){
      setMove(STOP);
      isStopped = true;
    }
    else if(!leftTendril.state() && !rightTendril.state() && !backTendril.state()){
      isStopped = false;
    }
  }

  if(millis() - irTimer >= IR_SIGNAL_PERIOD)
    setMove(STOP);
}


void tendrilsMode(){
  speed = 150;
  
  if(!obstacleAnalysis())
    setMove(FORWARD);
}


void photoSensorsMode(){
  if(loading){
    basicLightingLevel = ((analogRead(LEFT_PHOTO_SENSOR_PIN) / 8 * 0.95) 
                         + (analogRead(RIGHT_PHOTO_SENSOR_PIN) / 8)) / 2 * 1.2;
    loading = false;
  }
  
  speed = 220;
  
  if(!useTendrils || (useTendrils && !obstacleAnalysis())){
    byte leftPhotoSensor = analogRead(LEFT_PHOTO_SENSOR_PIN) / 8 * 0.95;
    byte rightPhotoSensor = analogRead(RIGHT_PHOTO_SENSOR_PIN) / 8;

    if(leftPhotoSensor <= basicLightingLevel && rightPhotoSensor <= basicLightingLevel)
      setMove(STOP);
    else if(leftPhotoSensor > rightPhotoSensor)
      setMove(FORWARD_LEFT);
    else if(leftPhotoSensor < rightPhotoSensor)
      setMove(FORWARD_RIGHT);
    else 
      setMove(FORWARD);
  }
}


void lineSensorMode(){
  if(loading){
    turnDirection = FORWARD_RIGHT;
    loading = false;
    turnDirectionChanged = true;
  }
  
  speed = 230;
  
  if(!useTendrils || (useTendrils && !obstacleAnalysis())){
    if(expander1.getBit(LINE_SENSOR_PIN) != blackLine && !turnDirectionChanged){
      if(turnDirection == FORWARD_LEFT) turnDirection = FORWARD_RIGHT;
      else turnDirection = FORWARD_LEFT;
      
      turnDirectionChanged = true;
    }
    else if(expander1.getBit(LINE_SENSOR_PIN) == blackLine && turnDirectionChanged){
      turnDirectionChanged = false;
    }
      
    setMove(turnDirection);
  }
}


bool obstacleAnalysis(){
  if(leftTendril.state()){
    obstaclesTimer = millis();
    obstacles = LEFT_OBSTACLES;
  }
  else if(rightTendril.state()){
    obstaclesTimer = millis();
    obstacles = RIGHT_OBSTACLES;
  }
  else if(backTendril.state()){
    obstaclesTimer = millis();
    obstacles = BACK_OBSTACLES;
  }

  if(obstacles != NO_OBSTACLES){
    if(obstacles == BACK_OBSTACLES){
      setMove(STOP);
      obstacles = NO_OBSTACLES;

      return true;
    }
  
    unsigned int _obstaclesPeriod = millis() - obstaclesTimer;
    speed = 150;

    if(_obstaclesPeriod < 200){
      setMove(BACKWARD);
    }
    else if(_obstaclesPeriod < 700){
      if(obstacles == LEFT_OBSTACLES) 
        setMove(LEFT);
      else if(obstacles == RIGHT_OBSTACLES) 
        setMove(RIGHT);
    }
    else {
      setMove(STOP);
      obstacles = NO_OBSTACLES;

      return false;
    }

    return true;
  }
  else {
    return false;
  }
}


void setMove(byte command){
  switch(command){
    case STOP:
      leftMotorStop();
      rightMotorStop();
      break;
    case FORWARD:
      leftMotorAnticlockwise();
      rightMotorClockwise();
      break;
    case BACKWARD:
      leftMotorClockwise();
      rightMotorAnticlockwise();
      break;
    case LEFT:
      leftMotorClockwise();
      rightMotorClockwise();
      break;
    case RIGHT:
      leftMotorAnticlockwise();
      rightMotorAnticlockwise();
      break;
    case FORWARD_LEFT:
      leftMotorStop();
      rightMotorClockwise();
      break;
    case FORWARD_RIGHT:
      leftMotorAnticlockwise();
      rightMotorStop();
      break;
    case BACKWARD_LEFT:
      leftMotorStop();
      rightMotorAnticlockwise();
      break;
    case BACKWARD_RIGHT:
      leftMotorClockwise();
      rightMotorStop();
      break;
  }
}


void leftMotorStop(){
  expander2.setBit(LEFT_MOTOR_IN1_PIN, LOW);
  expander2.setBit(LEFT_MOTOR_IN2_PIN, LOW);
}


void rightMotorStop(){
  expander2.setBit(RIGHT_MOTOR_IN3_PIN, LOW);
  expander2.setBit(RIGHT_MOTOR_IN4_PIN, LOW);
}


void leftMotorClockwise(){
  expander2.setBit(LEFT_MOTOR_IN1_PIN, HIGH);
  expander2.setBit(LEFT_MOTOR_IN2_PIN, LOW);
}


void leftMotorAnticlockwise(){
  expander2.setBit(LEFT_MOTOR_IN1_PIN, LOW);
  expander2.setBit(LEFT_MOTOR_IN2_PIN, HIGH);
}


void rightMotorClockwise(){
  expander2.setBit(RIGHT_MOTOR_IN3_PIN, LOW);
  expander2.setBit(RIGHT_MOTOR_IN4_PIN, HIGH);
}


void rightMotorAnticlockwise(){
  expander2.setBit(RIGHT_MOTOR_IN3_PIN, HIGH);
  expander2.setBit(RIGHT_MOTOR_IN4_PIN, LOW);
}


void setSpeed(byte pwm){
  expander2.setBit(LEFT_MOTOR_EN1_PIN, HIGH);
  expander2.setBit(RIGHT_MOTOR_EN2_PIN, HIGH);
  delayMicroseconds(pwm * 10);

  expander2.setBit(LEFT_MOTOR_EN1_PIN, LOW);
  expander2.setBit(RIGHT_MOTOR_EN2_PIN, LOW);
  delayMicroseconds((255 - pwm) / 10);
}


void setBlink(int glowTime, int pauseTime, byte amount){
  expander1.setBit(LED_STATE_PIN, LED_OFF);
  
  glowTimePeriod = glowTime; 
  pauseTimePeriod = pauseTime;
  blinkCounter = amount * 2;
}


void blinkListener(){
  if(blinkCounter > 0){
    if(expander1.getBit(LED_STATE_PIN) == LED_OFF && millis() - blinkTimer >= pauseTimePeriod){
      blinkTimer = millis();
      expander1.setBit(LED_STATE_PIN, LED_ON);
      blinkCounter--;
    }
    else if(expander1.getBit(LED_STATE_PIN) == LED_ON && millis() - blinkTimer >= glowTimePeriod){
      blinkTimer = millis();
      expander1.setBit(LED_STATE_PIN, LED_OFF);
      blinkCounter--;
    }
  }
}


void eeprom_update_byte(uint8_t _address, uint8_t _value){
  if(_value != eeprom_read_byte(_address)){
    eeprom_write_byte(_address, _value);
  }
}


byte getBatteryLevel(){
  int realVcc = (int)readVcc();

  if(realVcc > 3900) return 5;
  else if(realVcc > 3700) return 4;
  else if(realVcc > 3500) return 3;
  else if(realVcc > 3300) return 2;
  else return 1;
}


long readVcc(){
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  long result = (high << 8) | low;
  result = VCC_CONST * 1023 * 1000 / result;
  
  return result;
}
