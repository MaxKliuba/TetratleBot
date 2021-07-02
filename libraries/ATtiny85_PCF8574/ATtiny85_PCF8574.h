/*
 * ATTiny85 library to interface with PCF8574
 * install TinyWireM library https://github.com/adafruit/TinyWireM
 * I2C interface: SDA - pin P0 (0) and SCL - pin 2 (2); pull up to VCC (resistor 10kOm)
 *
 * by Maxim Kliuba
 */

#ifndef ATtiny85_PCF8574_h
#define ATtiny85_PCF8574_h

#define INPUT_BIT 1
#define OUTPUT_BIT 0

class ATtiny85_PCF8574 {
	public:
		ATtiny85_PCF8574();

		void begin(uint8_t address);

		void setBitMode(uint8_t bit, uint8_t mode);

		void setBit(uint8_t bit, bool value);

		void setByte(uint8_t value);

		uint8_t getByte();

		bool getBit(uint8_t bit);

	private:
		int _address;
		uint8_t _currentByte;
		uint8_t _bitModeMask;
	};
#endif