/*	*****************************
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
		- scanSEChannelsContinuous()
		- scanDIFFChannelsContinuous()
*/

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835.h> 
#include "enum.h"
#include "spi.h"


#ifndef ADS1256_H_INCLUDED
#define ADS1256_H_INCLUDED


uint8_t readByteFromReg(uint8_t registerID);
void    writeByteToReg(uint8_t registerID, uint8_t value);
uint8_t writeCMD(uint8_t command);
uint8_t setBuffer(bool val);
uint8_t readChipID(void);
void    setSEChannel(uint8_t channel);
void    setDIFFChannel(uint8_t positiveCh, uint8_t negativeCh);
void    setPGA(uint8_t pga);
void    setDataRate(uint8_t drate);
int32_t readData(void);
int32_t getValSEChannel(uint8_t channel);
int32_t getValDIFFChannel(uint8_t positiveCh, uint8_t negativeCh);
void    scanSEChannels(uint8_t channels[], uint8_t numOfChannels, uint32_t *values);
void    scanDIFFChannels(uint8_t positiveChs[], uint8_t negativeChs[], uint8_t numOfChannels, uint32_t *values);
void 	scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime);
void 	scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime);


/*	********************************************
	** PART 3 - "high-level" data acquisition **
	********************************************
	Functions:
		- writeToFile()
		- getValsMultiChSE()
		- getValsMultiChDIFF()
		- getValsSingleChSE()
		- getValsSingleChDIFF()

*/

//void writeValsToFile(FILE *file, uint32_t *values[], uint8_t numOfValues, uint8_t numOfChannels, char *pathWithFileName[]);
//void getValsMultiChSE(uint32_t numOfMeasure, uint32_t *values[], uint8_t *channels[], uint8_t numOfChannels, bool flushToFile, char *path[]);
//void getValsSingleChSE(uint32_t numOfMeasure, uint32_t *values[], uint8_t channel, bool flushToFile, char *path[]);
//void getValsSingleChDIFF(uint32_t numOfMeasure, uint32_t **values, uint8_t posCh, uint8_t negCh, uint8_t numOfChannels, bool flushToFile);


#endif



