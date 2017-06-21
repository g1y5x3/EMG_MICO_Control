/*
	*****************************
	** PART 2 - ads1256 driver **
	*****************************
	Functions:
		- readByteFromReg()
		- writeByteToReg()
		- writeCMD()
		- readChipID()
		- setSEChannel()
		- setDIFFChannel()
		- setPGA()
		- setDataRate()
		- readData()
		- getValSEChannel()
		- getValDIFFChannel()
		- scanSEChannels()
		- scanDIFFChannels()
		- scanSEChannelContinuous()
		- scanDIFFChannelContinuous()
*/

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835.h> 
#include "enum.h"
#include "spi.h"
#include "ads1256.h"


// Read 1 byte from register address registerID. 
// This could be modified to read any number of bytes from register!	
uint8_t readByteFromReg(uint8_t registerID)
{
	CS_0();
	send8bit(CMD_RREG | registerID); // 1st byte: address of the first register to read
	send8bit(0x00); 				 // 2nd byte: number of bytes to read = 1.
	
	delayus(7); 	// min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec
	uint8_t read = recieve8bit();
	CS_1();
	return read;
}

// Write value (1 byte) to register address registerID.
// This could be modified to write any number of bytes to register!
void writeByteToReg(uint8_t registerID, uint8_t value)
{
	CS_0();
	send8bit(CMD_WREG | registerID); // 1st byte: address of the first register to write
	send8bit(0x00); 				 // 2nd byte: number of bytes to write = 1.
	send8bit(value);				 // 3rd byte: value to write to register
	CS_1();
}

// Send standalone commands to register.
uint8_t writeCMD(uint8_t command)
{
	CS_0();
	send8bit(command);
	CS_1();
}

// Set the internal buffer (True - enable, False - disable).
uint8_t setBuffer(bool val)
{
	CS_0();
	send8bit(CMD_WREG | REG_STATUS);
	send8bit((0 << 3) | (1 << 2) | (val << 1));
	CS_1();
}

// Get data from STATUS register - chip ID information.
uint8_t readChipID(void)
{
	waitDRDY();
	uint8_t id = readByteFromReg(REG_STATUS);
	return (id >> 4); // Only bits 7,6,5,4 are the ones to read (only in REG_STATUS) - return shifted value!
}

// Write to MUX register - set channel to read from in single-ended mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setSEChannel(uint8_t channel)
{
	writeByteToReg(REG_MUX, channel << 4 | 1 << 3); // xxxx1000 - AINp = channel, AINn = AINCOM
}

// Write to MUX register - set channel to read from in differential mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setDIFFChannel(uint8_t positiveCh, uint8_t negativeCh)
{
	writeByteToReg(REG_MUX, positiveCh << 4 | negativeCh); // xxxx1000 - AINp = positiveCh, AINn = negativeCh
}

// Write to A/D control register - set programmable gain amplifier (PGA).
// CLKOUT and sensor detect options are turned off in this case.
void setPGA(uint8_t pga)
{
	writeByteToReg(REG_ADCON, pga); // 00000xxx -> xxx = pga 
}

// Write to A/D data rate register - set data rate.
void setDataRate(uint8_t drate)
{
	writeByteToReg(REG_DRATE, drate);
}

// Read 24 bit value from ADS1256. Issue this command after DRDY goes low to read s single
// conversion result. Allows reading data from multiple different channels and in 
// single-ended and differential analog input.
int32_t readData(void)
{
	uint32_t read = 0;
	uint8_t buffer[3];

	CS_0();
	send8bit(CMD_RDATA);
	delayus(7); // min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec

	buffer[0] = recieve8bit();
	buffer[1] = recieve8bit();
	buffer[2] = recieve8bit();
	// DRDY goes high here

	// construct 24 bit value
	read =  ((uint32_t)buffer[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buffer[1] << 8);
	read |= buffer[2];
	if (read & 0x800000){
		read |= 0xFF000000;
	}

	CS_1();

	return (int32_t)read;
}

// Get one single-ended analog input value by issuing command to input multiplexer.
// It reads a value from previous conversion!
// DRDY needs to be low!
int32_t getValSEChannel(uint8_t channel)
{
	int32_t read = 0;
	setSEChannel(channel); // MUX command
	delayus(3); // min delay: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);    // SYNC command
	delayus(3);
	writeCMD(CMD_WAKEUP);  // WAKEUP command
	delayus(1); // min delay: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	read = readData();
	return read;
}

// Get one differential analog input value by issuing command to input multiplexer.
// It reads a value from previous conversion!
// DRDY needs to be low!
int32_t getValDIFFChannel(uint8_t positiveCh, uint8_t negativeCh)
{
	int32_t read = 0;
	setDIFFChannel(positiveCh, negativeCh);
	delayus(3); // min delayus: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);
	delayus(3);
	writeCMD(CMD_WAKEUP);
	delayus(1); // min delayus: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	read = readData();
	return read;
}

// Get one single-ended analog input value from input channels you set (min 1, max 8).
void scanSEChannels(uint8_t channels[], uint8_t numOfChannels, uint32_t *values)
{
	for (int i = 0; i < numOfChannels; ++i){
		waitDRDY();
		values[i] = getValSEChannel(channels[i]);
	}
}

// Get one differential analog input value from input channels you set (min 1, max 4).
void scanDIFFChannels(uint8_t positiveChs[], uint8_t negativeChs[], uint8_t numOfChannels, uint32_t *values)
{
	for (int i = 0; i < numOfChannels; ++i){
		waitDRDY();
		values[i] = getValDIFFChannel(positiveChs[i], negativeChs[i]);
	}
}

// Continuously acquire analog data from one single-ended analog input.
// Allows sampling of one single-ended input channel up to 30,000 SPS.
void scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime)
{
	uint8_t buffer[3];
	uint32_t read = 0;
	uint8_t del = 8;
	
	// Set single-ended analog input channel.
	setSEChannel(channel);
	delayus(del);
	
	// Set continuous mode.
	CS_0();
	waitDRDY();
	send8bit(CMD_RDATAC);
	delayus(del); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds
	
	// Start reading data
	currentTime [numOfMeasure];
	clock_t startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY();
		buffer[0] = recieve8bit();
		buffer[1] = recieve8bit();
		buffer[2] = recieve8bit();

		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8);
		read |= buffer[2];
		if (read & 0x800000){
			read |= 0xFF000000;
		}
		values[i] = read;
		currentTime[i] = clock() - startTime;
		//printf("%f %i\n", (float)read/1670000, clock() - startTime); // TESTING
		delayus(del);
	}
	
	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();
}

// Continuously acquire analog data from one differential analog input.
// Allows sampling of one differential input channel up to 30,000 SPS.
void scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime)
{
	uint8_t buffer[3];
	uint32_t read = 0;
	uint8_t del = 8;

	// Set differential analog input channel.
	setDIFFChannel(positiveCh, negativeCh);
	delayus(del);

	// Set continuous mode.
	CS_0();
	waitDRDY();
	send8bit(CMD_RDATAC);
	delayus(del); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds

	// Start reading data.
	currentTime [numOfMeasure];	
	clock_t startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY();
		buffer[0] = recieve8bit();
		buffer[1] = recieve8bit();
		buffer[2] = recieve8bit();

		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8);
		read |= buffer[2];
		if (read & 0x800000){
			read |= 0xFF000000;
		}
		values[i] = read;
		currentTime[i] = clock() - startTime;
		//printf("%f %i\n", (float)read/1670000, clock() - startTime); // TESTING
		delayus(del);
	}

	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();
}

/*
	********************************************
	** PART 3 - "high-level" data acquisition **
	********************************************
	Functions:
		- writeToFile()
		- getValsMultiChSE()
		- getValsMultiChDIFF()
		- getValsSingleChSE()
		- getValsSingleChDIFF()

*/
/*
// Write array of values to file. To avoid openning the file in the middle of a data acquisition,
// we need to open it before calling this function and assign it to null pointer. It also needs to be closed manually.
void writeValsToFile(FILE *file, uint32_t *values[], uint8_t numOfValues, uint8_t numOfChannels, char *pathWithFileName[])
{
	if (NULL)
	{
		file = fopen(pathWithFileName, "a");
	}
	for (int i = 0; i < numOfValues/numOfChannels; ++i)
	{
		fprintf(file, "%i  ", i);
		for (int ch = 0; i < numOfChannels; ++ch)
		{
			fprintf(file, "%f ", (double)values[i*numOfChannels + ch]/1670000);
		}
		fprintf(file, "\n");
	}
}		

// Get data from multiple channels in single-ended input mode, either with or without flushing data 
// to file (depends on how much values you want to store).
void getValsMultiChSE(uint32_t numOfMeasure, uint32_t *values[], uint8_t *channels[], uint8_t numOfChannels, bool flushToFile, char *path[])
{
	clock_t startTime, endTime;
	uint32_t tempValues [numOfChannels];
	startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		//scanSEChannels(channels, numOfChannels, tempValues); //try changing tempValues with values[numOfChannels*i]
		scanSEChannels(channels, numOfChannels, values[numOfChannels*i]);
		printf("%i ||", i+1);
		for (int ch = 0; ch < numOfChannels; ++ch)
		{
			//values[i*numOfChannels + ch] = tempValues[ch];  // GET RID OF THIS COPYING! 
			printf(" %fV ||", (double)values[ch]/10000/167);
		}
		printf("\n");
		if (flushToFile && (i == sizeof(values)/32))
		{
			FILE *file;
			file = writeValToFile(values, numOfMeasure, numOfChannels, path);
			i = 0;
		}
	}
	endTime = clock();
	printf("Time for %i single-ended measurements on %i channels is %d microseconds (%5.1f SPS/channel).\n", numOfMeasure, numOfChannels, endTime - startTime, (double)(numOfMeasure)/(endTime - startTime)*1e6);
}

// Get data from multiple channels in differential input mode, either with or without flushing data 
// to file (depends on how much values you want to store).
void getValsMultiChDIFF(uint32_t numOfMeasure, uint32_t *values[], uint8_t *posChs[], uint8_t *negChs[], uint8_t numOfChannels, bool flushToFile, char *path[])
{
	clock_t startTime, endTime;
	startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		scanSEChannels(channels, numOfChannels, values[numOfChannels*i]);
		printf("%i ||", i+1);
		for (int ch = 0; ch < numOfChannels; ++ch)
		{
			printf(" %fV ||", (double)values[ch]/10000/167);
		}
		printf("\n");
		if (flushToFile && (i == sizeof(values)/32))
		{
			FILE *file;
			file = writeValToFile(values, numOfMeasure, numOfChannels, path);
			i = 0;
		}
	}
	endTime = clock();
	printf("Time for %i single-ended measurements on %i channels is %d microseconds (%5.1f SPS/channel).\n", numOfMeasure, numOfChannels, endTime - startTime, (double)(numOfMeasure)/(endTime - startTime)*1e6);
}

// Get data from a single channel in single-ended input mode, either with or without flushing data 
// to file (depends on how much values you want to store).
void getValsSingleChSE(uint32_t numOfMeasure, uint32_t *values[], uint8_t channel, bool flushToFile, char *path[])
{
	scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, uint32_t *values)


}

// Get data from a single channel in differential input mode, either with or without flushing data 
// to file (depends on how much values you want to store).
void getValsSingleChDIFF(uint32_t numOfMeasure, uint32_t **values, uint8_t posCh, uint8_t negCh, uint8_t numOfChannels, bool flushToFile)
{

}
*/
