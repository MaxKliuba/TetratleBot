/*
 * ATTiny85 library to interface with PCF8574
 * install TinyWireM library https://github.com/adafruit/TinyWireM
 * I2C interface: SDA - pin P0 (0) and SCL - pin 2 (2); pull up to VCC (resistor 10kOm)
 *
 * by Maxim Kliuba
 */

#include <TinyWireM.h>
#include "ATtiny85_PCF8574.h"


ATtiny85_PCF8574::ATtiny85_PCF8574() {
	_address = 0;
	_bitModeMask = B11111111;
} 


void ATtiny85_PCF8574::begin(uint8_t address) {
	_address = address;
	TinyWireM.begin();
}


void ATtiny85_PCF8574::setBitMode(uint8_t bit, uint8_t mode) {
	if (bit < 0 || bit > 7) {
		return;
	}

	switch (mode) {
		case INPUT_BIT:
			_bitModeMask |= (1 << bit);
			break;
		case OUTPUT_BIT:
			_bitModeMask &= ~(1 << bit);
			break;
	}
}


void ATtiny85_PCF8574::setBit(uint8_t bit, bool value) {
	if (bit < 0 || bit > 7) {
		return;
	}

	if (value) {
		setByte(_currentByte | (1 << bit));
	} 
	else {
		setByte(_currentByte & ~(1 << bit));
	}
}


void ATtiny85_PCF8574::setByte(uint8_t value) {
	if (_address == 0) {
		return;
	}

	_currentByte = value | _bitModeMask;
	TinyWireM.beginTransmission(_address);
	TinyWireM.write(_currentByte);
	TinyWireM.endTransmission();
}


uint8_t ATtiny85_PCF8574::getByte() {
	TinyWireM.requestFrom(_address, 1);

	return TinyWireM.read();
}


bool ATtiny85_PCF8574::getBit(uint8_t bit) {
	if (bit < 0 || bit > 7) {
		return 0;
	}

	uint8_t byteValue = getByte();

	return (byteValue & (1<<bit)) != 0;
}