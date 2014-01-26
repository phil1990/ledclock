#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define HW_SPI

// default SPI
#define DDR_SPI		DDRB
#define PORT_SPI	PORTB

#define SS		(1 << 2) //
#define DS		(1 << 5) // MOSI
#define STCP		(1 << 0) // RCLK (Latch)
#define SHCP		(1 << 7) // SRCLK, SCK

uint8_t time[] = {
	0b00111111	//0
	0b00000110	//1
	0b01011011	//2	
	0b01001111	//3
	0b00010000	//4
	0b00100000	//5
	0b01000000	//6
	0b10000000	//7
	0b00000010	//8
	0b00000001	//9
};

/**
 * SPI
 */
#ifdef HW_SPI
void spi_init() {
	DDR_SPI = SHCP + STCP + DS + SS;		// SHCP, STCP, DS and SS as output

	SPCR = (1 << SPE) | (1 << DORD) | (1 << MSTR);	// enable SPI, LSB first, act as master
	SPSR = (1 << SPI2X);				// double speed
}

uint8_t spi_byte(uint8_t byte) {
	SPDR = byte;
	while (!(SPSR & (1 << SPIF))); // wait until tx complete
	return SPDR;
}
#else
void spi_init() {
	DDR_SPI = SHCP + STCP + DS;	// SHCP, STCP, DS as output
}

uint8_t spi_byte(uint8_t byte) {
	uint8_t cnt;
	for (cnt = 0; cnt < 8; cnt++) {
		if (byte & (1 << cnt)) {
			PORT_SPI |= DS;
		}
		else {
			PORT_SPI &= ~DS;
		}

		// rising edge shiftreg clock
		PORT_SPI |= SHCP;
		PORT_SPI &= ~SHCP;
	}

	return 0;
}
#endif

void spi_string(char *bytes, uint8_t len) {
	uint8_t cnt;
	for (cnt = 0; cnt < len; cnt++) {
		spi_byte((uint8_t) bytes[cnt]);
	}

	// rising edge storage clock
	_delay_ms(10);
	PORT_SPI |= STCP;
	_delay_ms(10);
	PORT_SPI &= ~STCP;
}

int main( void ) {
	spi_init();
	uint8_t i;
	while (1) {
		for (i = 0; i < 10; i++) {		
			spi_byte(time[i]);

			// rising edge storage clock
			PORT_SPI |= STCP;
			PORT_SPI &= ~STCP;

			_delay_ms(1);
		}
	}

	return 0;
}
