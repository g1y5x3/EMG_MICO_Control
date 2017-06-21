#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835.h>
#include "spi.h"  

// Delay in microseconds.
void delayus(uint64_t microseconds)
{
	bcm2835_delayMicroseconds(microseconds);
}

// Send 8 bit value over serial interface (SPI).
void send8bit(uint8_t data)
{
	bcm2835_spi_transfer(data);
}

// Recieve 8 bit value over serial interface (SPI).
uint8_t recieve8bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

// Wait until DRDY is low.
void waitDRDY(void)
{
	while(!DRDY_LOW()){
		continue;
	}
}

// Initialize SPI, call every time at the start of the program.
// Returns 1 if succesfull!
uint8_t initializeSPI()
{
	if (!bcm2835_init())
	    return -1;
	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
	// Spi clock divider: 250Mhz / 256 = 0.97 Mhz ~ between 4 to 10 * 1/freq.clkin.
	// Divider 128 is already more than 4 * 1/freq.clckin so it is not apropriate for usage.
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
	bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP); // Set SPICS pin to output
	bcm2835_gpio_write(SPICS, HIGH);
	bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);  // Set DRDY pin to input
	bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP); 
	return 1;
}

// End SPI, call every time at the end of the program.
void endSPI()
{
	bcm2835_spi_end();
	bcm2835_close();
}
