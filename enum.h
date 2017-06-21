/*
	***************************
	** PART 0 - enumerations **
	***************************
	Enumerations:
		- PGA   - programmable gain amplifier (PGA) settings
		- DRATE - data rate of programmable filter settings
		- REG   - register control adresses
		- CMD   - commands for controlling operation of ADS1256
		- AIN   - input analog channels
		- bool  - boolean True, False
*/

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <bcm2835.h>  


#ifndef ENUM_H_INCLUDED
#define ENUM_H_INCLUDED


// Set custom data types that are 8, 16 and 32 bits long.
#define uint8_t  unsigned char  	// 1 byte
#define uint16_t unsigned short 	// 2 bytes
#define uint32_t unsigned long  	// 4 bytes
#define uint64_t unsigned long long // 8 bytes

//  Set the Programmable gain amplifier (PGA).
//	PGA Provides more resolution when measuring smaller input signals.
//	Set the PGA to the highest possible setting.
enum
{
	PGA_GAIN1	= 0, // Input voltage range: +- 5 V
	PGA_GAIN2	= 1, // Input voltage range: +- 2.5 V
	PGA_GAIN4	= 2, // Input voltage range: +- 1.25 V
	PGA_GAIN8	= 3, // Input voltage range: +- 0.625 V
	PGA_GAIN16	= 4, // Input voltage range: +- 0.3125 V
	PGA_GAIN32	= 5, // Input voltage range: +- 0.15625 V
	PGA_GAIN64	= 6  // Input voltage range: +- 0.078125 V
};

//  Set a data rate of a programmable filter (programmable averager).
//	Programmable from 30,000 to 2.5 samples per second (SPS).
//	Setting the data rate to high value results in smaller resolution of the data.
enum
{
	DRATE_30000 = 0xF0, 
	DRATE_15000 = 0xE0,
	DRATE_7500  = 0xD0,
	DRATE_3750  = 0xC0,
	DRATE_2000  = 0xB0,
	DRATE_1000  = 0xA1,
	DRATE_500   = 0x92,
	DRATE_100   = 0x82,
	DRATE_60    = 0x72,
	DRATE_50    = 0x63,
	DRATE_30    = 0x53,
	DRATE_25    = 0x43,
	DRATE_15    = 0x33,
	DRATE_10    = 0x20,
	DRATE_5     = 0x13,
	DRATE_2d5   = 0x03
};

//  Set of registers.
//	The operation of the ADS1256 is controlled through a set of registers. 
//	Collectively, the registers contain all the information needed to configure 
//	data rate, multiplexer settings, PGA setting, calibration, etc.
enum
{
	REG_STATUS = 0,	 // Register adress: 00h, Reset value: x1H
	REG_MUX    = 1,  // Register adress: 01h, Reset value: 01H
	REG_ADCON  = 2,  // Register adress: 02h, Reset value: 20H
	REG_DRATE  = 3,  // Register adress: 03h, Reset value: F0H
	REG_IO     = 4,  // Register adress: 04h, Reset value: E0H
	REG_OFC0   = 5,  // Register adress: 05h, Reset value: xxH
	REG_OFC1   = 6,  // Register adress: 06h, Reset value: xxH
	REG_OFC2   = 7,  // Register adress: 07h, Reset value: xxH
	REG_FSC0   = 8,  // Register adress: 08h, Reset value: xxH
	REG_FSC1   = 9,  // Register adress: 09h, Reset value: xxH
	REG_FSC2   = 10, // Register adress: 0Ah, Reset value: xxH
};

//  This commands control the operation of the ADS1256. 
//	All of the commands are stand-alone except for the register reads and writes 
//	(RREG, WREG) which require a second command byte plus data.
//	CS must stay low (CS_0()) during the entire command sequence.
enum
{
	CMD_WAKEUP   = 0x00, // Completes SYNC and Exits Standby Mode
	CMD_RDATA    = 0x01, // Read Data
	CMD_RDATAC   = 0x03, // Read Data Continuously
	CMD_SDATAC   = 0x0F, // Stop Read Data Continuously
	CMD_RREG     = 0x10, // Read from REG - 1st command byte: 0001rrrr 
						 //					2nd command byte: 0000nnnn
	CMD_WREG     = 0x50, // Write to REG  - 1st command byte: 0001rrrr
						 //					2nd command byte: 0000nnnn
						 // r = starting reg address, n = number of reg addresses
	CMD_SELFCAL  = 0xF0, // Offset and Gain Self-Calibration
	CMD_SELFOCAL = 0xF1, // Offset Self-Calibration
	CMD_SELFGCAL = 0xF2, // Gain Self-Calibration
	CMD_SYSOCAL  = 0xF3, // System Offset Calibration
	CMD_SYSGCAL  = 0xF4, // System Gain Calibration
	CMD_SYNC     = 0xFC, // Synchronize the A/D Conversion
	CMD_STANDBY  = 0xFD, // Begin Standby Mode
	CMD_RESET    = 0xFE, // Reset to Power-Up Values
};

// Input analog channels.
enum
{
	AIN0   = 0, //Binary value: 0000 0000
	AIN1   = 1, //Binary value: 0000 0001
	AIN2   = 2, //Binary value: 0000 0010
	AIN3   = 3, //Binary value: 0000 0011
	AIN4   = 4, //Binary value: 0000 0100
	AIN5   = 5, //Binary value: 0000 0101
	AIN6   = 6, //Binary value: 0000 0110
	AIN7   = 7, //Binary value: 0000 0111
	AINCOM = 8  //Binary value: 0000 1000
};

// Boolean values.
//typedef enum
//{
//	False = 0,
//	True  = 1,
//} bool;


#endif // ENUM
