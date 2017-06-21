#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835.h>  


#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED


// DRDY (ads1256 data ready output) - used as status signal to indicate when 
// conversion data is ready to be read.  
// Low  - new data avaliable, high - 24 bits are read or new data is being updated.
#define  DRDY		RPI_GPIO_P1_11
// RST (ADS1256 reset output)
#define  RST 		RPI_GPIO_P1_12
// SPICS (ADS1256 chip select) - allows individual selection of a ADS1256 device 
// when multiple devices share the serial bus. 
// Low - for the duration of the serial communication, high - serial interface is reset 
// and DOUT enters high impedance state.
#define	 SPICS		RPI_GPIO_P1_15
// DIN (data input) - send data to ADS1256. When SCLK goes from low to high.
#define  DIN 		RPI_GPIO_P1_19
// DOUT (data output) - read data from ADS1256. When SCLK goes from high to low.
#define  DOUT 		RPI_GPIO_P1_21
// SCLK (serial clock) - used to clock data on DIN and DOUT pins into and out of ADS1256.
// If not using external clock, ignore it.
#define  SCLK 		RPI_GPIO_P1_23
// Set SPICS to high (DOUT goes high).
#define  CS_1()  	bcm2835_gpio_write(SPICS, HIGH)
// Set SPICS to low (for serial communication).
#define  CS_0()  	bcm2835_gpio_write(SPICS, LOW)
// Set RST to high.
#define  RST_1() 	bcm2835_gpio_write(RST, HIGH)
// Set RST to low.
#define  RST_0() 	bcm2835_gpio_write(RST, LOW)
// Returns True if DRDY is low.
#define  DRDY_LOW()	bcm2835_gpio_lev(DRDY)==0


/*	
    *******************************
	** PART 1 - serial interface **
	*******************************
	Functions:
		- CS_1()
		- CS_0()
		- RST_1()
		- RST_0()
		- DRDY_LOW()
		- delayus()
		- send8bit()
		- recieve8bit()
		- waitDRDY()
		- initializeSPI()
		- endSPI()
*/

void    delayus(uint64_t microseconds);
void    send8bit(uint8_t data);
uint8_t recieve8bit(void);
void    waitDRDY(void);
uint8_t initializeSPI();
void    endSPI();


#endif // SPI
