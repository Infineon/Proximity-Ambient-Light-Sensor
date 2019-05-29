/**
 * @file Pals2.cpp
 * @brief Arduino library to control the proximity and ambient light sensor PALS2 from Infineon
 * @author Angel Corona
 */

#include "Pals2.h"
#include <math.h>

Pals2::Pals2() {
}

void Pals2::begin(void) {
	//begin i2c
	Wire.begin();
	//reset sensor
	resetSensor();
	//wake up sensor (stand-by)
	wakeUpSensor();
}

/***********************************************************************/
/**********************        GENERAL          ************************/
/***********************************************************************/
 void Pals2::resetSensor(void) {
	wireWriteDataByte(CONFIG_REGISTER,0x00);
	wireWriteDataByte(PROXIMITY_CONFIG, 0x00);
	wireWriteDataByte(PROXIMITY_ADC_GAIN,0x00);
	wireWriteDataByte(IRED_CONFIG, 0x00);
	wireWriteDataByte(ALS_CONFIG, 0x00);
	wireWriteDataByte(INTERRUPT_THR_CONFIG, 0x00);
} 

void Pals2::wakeUpSensor(void) {
	wireWriteDataByte(CONFIG_REGISTER, 0x01);
	wireWriteDataByte(PROXIMITY_ADC_GAIN, 0xC2);
}

/***********************************************************************/
/**********************       PROXIMITY         ************************/
/***********************************************************************/
/*
*  ENABLE = 1
*  DISABLE = 0
*/
void Pals2::setProximityPeriodicMeasurements(uint8_t enable_value) {
	
	uint8_t temp = 0;
	
	temp = wireReadDataByte(CONFIG_REGISTER);
	temp |= enable_value << 1;
	wireWriteDataByte(CONFIG_REGISTER, temp);
}

void Pals2::setProximityMeasurementRate(uint16_t rate) {
	
	if (rate > 256){
		// Rate is 256 max
		rate = 256;
	}
		
	//clear the bits for measurement rate
	proximityConfig &= ~0x07;
	
	//prx_rate = ld(rate) - 1
	uint8_t prx_rate = -1;
	
	while (rate >>= 1){
		prx_rate++;
	}
	proximityConfig |= prx_rate;
	proximityConfig |= 0x60; // Doubles input current of proximity channel and adds dark light
	wireWriteDataByte(PROXIMITY_CONFIG, proximityConfig);
}

void Pals2::setProximityOutput(uint8_t ired, uint8_t current) {
	
	if (current > 20){
		// Rate is 256 max
		current = 20;
	}
		
	outputonfig |= ired << 5;
	outputonfig |= current; 
	wireWriteDataByte(IRED_CONFIG, outputonfig);
}


uint16_t Pals2::getRawProximityOnDemand(void) {
	// prox_od and standby_en
	
	uint8_t upperByte;
	uint8_t lowerByte;
	uint8_t prox_data[2];
	bool ready = false;
	uint8_t tempValue = 0;
	
	wireWriteDataByte(CONFIG_REGISTER, 0x09);
	
	unsigned long start = millis();
	//busy waiting
	
	while (!ready) {
		ready = (wireReadDataByte(CONFIG_REGISTER) & 0x20) >> 5;
		//set time out in case sensor gets stuck
		if (millis() - start > 1000)
			break;
		
		Wire.endTransmission();
	}
	
	
	wireReadDataBlock(PROXIMITY_MSB,(uint8_t*)prox_data,2);
	
	return concatResults(prox_data[0], prox_data[1]);
}


void Pals2::setProximityInterrupt(uint16_t topThreshold, uint8_t valuetop, uint16_t bottomThreshold, uint8_t valuebottom) {
	//interrupt on exceeding both top & bottom thresholds
	interruptConfig = wireReadDataByte(INTERRUPT_THR_CONFIG);
	
	if(valuetop){
		interruptConfig |= (valuetop << 4);
		/* If enabled, update top threshold value*/
		wireWriteDataByte(PROX_INT_TOP_MSB, (topThreshold>> 8) & 0xFF );
		wireWriteDataByte(PROX_INT_TOP_LSB, topThreshold & 0xFF);
	
	}
	else{
		interruptConfig &= ~(valuetop << 4);
	}
	
	if(valuebottom){
		interruptConfig |= (valuebottom << 3);
		/* If enabled, update bottom threshold value*/
		wireWriteDataByte(PROX_INT_BOT_MSB, (bottomThreshold>> 8) & 0xFF );
		wireWriteDataByte(PROX_INT_BOT_LSB, bottomThreshold & 0xFF);

	}
	else{
		interruptConfig &= ~(valuebottom << 3);
	}
	
	/* Set Interrupt config register*/
	wireWriteDataByte(INTERRUPT_THR_CONFIG, interruptConfig);


}

void Pals2::clearProximityInterrupts(void)
{
 	/* Clear Interrupt */
	uint8_t cleanInterruptsMask = interruptConfig & 0x38; //mask top and bottom enable bits
		
	wireWriteDataByte(INTERRPUT_STATUS, cleanInterruptsMask);
}


uint16_t Pals2::readProximity(void)
{
    uint8_t prox_data[2];
    
	wireReadDataBlock(PROXIMITY_MSB,(uint8_t*)prox_data,2);
	
	return concatResults(prox_data[0], prox_data[1]);
}

/* @ToDo: Proximity Offset compensation */
/* @ToDo: Proximity Interrupt Persistance */
/* @ToDo: ADC gain */


/***********************************************************************/
/********************       AMBIENT LIGHT         **********************/
/***********************************************************************/

void Pals2::setAmbientLightPeriodicMeasurements(uint8_t enable_value) {
	
	uint8_t temp = 0;
	
	temp = wireReadDataByte(CONFIG_REGISTER);
	temp |= enable_value << 2;
	wireWriteDataByte(CONFIG_REGISTER, temp);
}


void Pals2::setAmbientLightInterrupt(uint16_t topThreshold, uint8_t valuetop, uint16_t bottomThreshold, uint8_t valuebottom) {
	//interrupt on exceeding both top & bottom thresholds
	interruptConfig = wireReadDataByte(INTERRUPT_THR_CONFIG);
	
	if(valuetop){
		interruptConfig |= (valuetop << 2);
		/* If enabled, update top threshold value*/
		wireWriteDataByte(ALS_INT_TOP_MSB, (topThreshold>> 8) & 0xFF );
		wireWriteDataByte(ALS_INT_TOP_LSB, topThreshold & 0xFF);
	
	}
	else{
		interruptConfig &= ~(valuetop << 2);
	}
	
	if(valuebottom){
		interruptConfig |= (valuebottom << 1);
		/* If enabled, update bottom threshold value*/
		wireWriteDataByte(ALS_INT_BOT_MSB, (bottomThreshold>> 8) & 0xFF );
		wireWriteDataByte(ALS_INT_BOT_LSB, bottomThreshold & 0xFF);

	}
	else{
		interruptConfig &= ~(valuebottom << 1);
	}
	
	/* Set Interrupt config register*/
	wireWriteDataByte(INTERRUPT_THR_CONFIG, interruptConfig);


}

void Pals2::clearAmbientLightInterrupts(void)
{
 	/* Clear Interrupt */
	uint8_t cleanInterruptsMask = interruptConfig & 0x07; //mask top and bottom enable bits
		
	wireWriteDataByte(INTERRPUT_STATUS, cleanInterruptsMask);
}

void Pals2::setAmbientLightMeasurementRate(uint8_t alsRate) {
	
	ambientLightConfig = wireReadDataByte(ALS_CONFIG);
	//max. rate is 8 measurements/s
	if (alsRate > 8)
		alsRate = 8;
	ambientLightConfig &= ~0x07;
	ambientLightConfig |= (alsRate - 1);
	wireWriteDataByte(ALS_CONFIG, ambientLightConfig);
} 

void Pals2::setAmbientLightADCGain(uint16_t adcGain, bool isHalved) {
	
	ambientLightConfig = wireReadDataByte(ALS_CONFIG);
	//clears bits 3,4
	ambientLightConfig &= ~(0x03 << 3);
	//default ADC gain is 200 fA (bits 3, 4 = 0)
	switch (adcGain) {
		case 800:
			gainFactor = 22.17;
			ambientLightConfig |= 0x01 << 3;
			break;
		case 3200:
			gainFactor = 6.02;
			ambientLightConfig |= 0x10 << 3;
			break;
		case 25600:
			gainFactor = 0.75;
			ambientLightConfig |= 0x11 << 3;
			break;
		default:
			gainFactor = 81.79;
			break;
	}
		
	if(isHalved) ambientLightConfig |=  0x80;
	
	wireWriteDataByte(ALS_CONFIG, ambientLightConfig);
	

} 


 uint16_t Pals2::getRawAmbientLightOnDemand(void) {

	
	uint8_t upperByte;
	uint8_t lowerByte;
	uint8_t als_data[2];
	bool ready = false;
	uint8_t tempValue = 0;
	
	// als_od and standby_en
	wireWriteDataByte(CONFIG_REGISTER, 0x11);
	
	unsigned long start = millis();
	//busy waiting
	
	while (!ready) {
		ready = (wireReadDataByte(CONFIG_REGISTER) & 0x40) >> 6;
		//set time out in case sensor gets stuck
		if (millis() - start > 1000)
			break;
		
		Wire.endTransmission();
	}
	
	wireReadDataBlock(ALS_MSB,(uint8_t*)als_data,2);
	
	return concatResults(als_data[0], als_data[1]);
}

 
 uint16_t Pals2::readAmbientLight(void)
{
    uint8_t als_data[2];
    
	wireReadDataBlock(ALS_MSB,(uint8_t*)als_data,2);
	
	return concatResults(als_data[0], als_data[1]);
}
 
/*-----------------------------------------------------------------------------------------------------*/
/* Utilities */
/*-----------------------------------------------------------------------------------------------------*/

uint16_t Pals2::concatResults(uint8_t upperByte, uint8_t lowerByte) {
	uint16_t value = 0x0000;
	value = (uint16_t) upperByte << 8;
	value |= (uint16_t) lowerByte;
	return value;
}

void Pals2::writeOut(uint16_t regNum, uint16_t val) {
	Wire.beginTransmission(SLAVE_ADDRESS);
	Wire.write(regNum);
	Wire.write(val);
	Wire.endTransmission();
}

uint8_t Pals2::dumpRegister(uint8_t regNum) {
	
	uint8_t value;
	Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.requestFrom(SLAVE_ADDRESS, 1, regNum, PALS2_REG_SIZE_BYTES, 0);
	value = Wire.read();
	Wire.endTransmission();
	return value;
}


/*-----------------------------------------------------------------------------------------------------*/
/* Low level Register Read/Write Functions */
/*-----------------------------------------------------------------------------------------------------*/

/**
 * @brief Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
uint8_t Pals2::wireReadDataByte(uint8_t reg)
{
    uint8_t val=0;
    Wire.beginTransmission(SLAVE_ADDRESS);
    /* Read from register */
    Wire.requestFrom(SLAVE_ADDRESS, 1,reg,PALS2_REG_SIZE_BYTES,0);
    while (Wire.available()) {
        val = Wire.read();
    }
	
	Wire.endTransmission();
    return val;
}

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
int Pals2::wireReadDataBlock(   uint8_t reg, uint8_t *val, unsigned int len)
{
    unsigned char i = 0;
    
    Wire.beginTransmission(SLAVE_ADDRESS);
    /* Read block data */
    Wire.requestFrom(SLAVE_ADDRESS, len,reg,1,0);
    while (Wire.available()) {
        if (i >= len) {
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }
	Wire.endTransmission();
    return i;
}


/**
 * @brief Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool Pals2::wireWriteDataByte(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();

    return true;
}

/**
 * @brief Writes a block (array) of bytes to the I2C device and register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val pointer to the beginning of the data byte array
 * @param[in] len the length (in bytes) of the data to write
 * @return True if successful write operation. False otherwise.
 */
bool Pals2::wireWriteDataBlock(  uint8_t reg, uint8_t *val, unsigned int len)
{
    unsigned int i;

    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(reg);
    for(i = 0; i < len; i++) {
        Wire.beginTransmission(val[i]);
    }
    if( Wire.endTransmission() != 0 ) {
        return false;
    }

    return true;
}


