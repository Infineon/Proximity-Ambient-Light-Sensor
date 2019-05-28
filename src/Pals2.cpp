/**
 * @file Pals2.cpp
 * @brief Arduino library to control the proximity and ambient light sensor PALS2 from Infineon (packaged by Vishay as VCNL4135X01)
 * @author Angel Corona
 * @bug no Blue-PD value updates -> getIlluminance() not working;
 * @bug update very slow (1 measurement/s) when periodic measurement is enabled, changing measurement rates in config register has no effect
 * @bug	in register 83h sensor measurement freezes if IRED output is not default(0): due to missing IREDs?
 */

#include "Pals2.h"
#include <math.h>

Pals2::Pals2() {
}

void Pals2::begin(void) {
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

/* void Pals2::setInterruptPersistence(uint8_t persistence) {
	uint8_t counts = persistence;
	if (persistence > 128)
		counts = 128;
	//clear the highest 3 bits
	interruptConfig &= ~(0x07 << 5);
	//find the highest set bit
	uint8_t i = 0;
	while (counts >>= 1)
		i++;
	//set the highest 3 bits
	interruptConfig |= (i << 5);
	writeOut(INTERRUPT_CONFIG, interruptConfig);
} */


/***********************************************************************/
/********************       AMBIENT LIGHT         **********************/
/***********************************************************************/

void Pals2::setAmbientLightPeriodicMeasurements(uint8_t enable_value) {
	
	uint8_t temp = 0;
	
	temp = wireReadDataByte(CONFIG_REGISTER);
	temp |= enable_value << 2;
	wireWriteDataByte(CONFIG_REGISTER, temp);
}



/* void Pals2::enableAmbientLightInterrupt(uint16_t topThreshold,
		uint16_t bottomThreshold) {
	//interrupt on exceeding both top & bottom thresholds
	interruptConfig |= (0x03 << 1);
	writeOut(INTERRUPT_CONFIG, interruptConfig);

	writeOut(ALS_INT_BOT_HB, bottomThreshold & (0xFF << 8));
	writeOut(ALS_INT_BOT_LB, bottomThreshold & 0xFF);
	writeOut(ALS_INT_TOP_HB, topThreshold & (0xFF << 8));
	writeOut(ALS_INT_TOP_LB, topThreshold & 0xFF);
} */

/* void Pals2::disableAmbientLightInterrupt(void) {
	interruptConfig |= (0x03 << 1);
	writeOut(INTERRUPT_CONFIG, interruptConfig);
}

void Pals2::enableColorCompensation(bool colorCompPeriod) {
	writeOut(ALS_COMPENSATION, 0x01 + 0x02 * colorCompPeriod);
	colorCompensationEnabled = true;
}
 */
/* float Pals2::getBlueRatio(void) {
	//TODO: check why always zero
	return (blue1PD - blue2PD) / blue1PD;
} */

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
 
 
 
/***********************************************************************/
/***********************       GESTURE         *************************/
/***********************************************************************/

gesture_t Pals2::GestureAlgorithm(proximity_Samples_t *samples)
{
  uint16_t ps[3];

  static uint32_t ps_entry_time[3] = { 0, 0, 0 };
  static uint32_t ps_exit_time[3]  = { 0, 0, 0 };

  static uint8_t  ps_state[3] = { 0, 0, 0 };

  uint8_t array_counter;
  uint32_t diff_x ;
  uint32_t diff_y1 ;
  uint32_t diff_y2 ;
  uint32_t ps_time[3] ;
  uint32_t ps_avg;
  gesture_t  ret = NONE;  /*gesture result return value */
  /*save new samples into ps array */
  ps[0] = samples->ps1;
  ps[1] = samples->ps2;
  ps[2] = samples->ps3;

  /* Check state of all three measurements */
  for (array_counter = 0; array_counter < 3; array_counter++)
  {
    /* If measurement higher than the ps_threshold value, */
    /*   record the time of entry and change the state to look for the exit time */
    if (ps[array_counter] >= PS_THRESHOLD)
    {
      ret = PROX; 
      if (ps_state[array_counter] == 0)
      {
        ps_state[array_counter]      = 1;
        ps_entry_time[array_counter] = samples->timestamp;
      }
    }
    else
    {
      if (ps_state[array_counter] == 1)
      {
        ps_state[array_counter]     = 0;
        ps_exit_time[array_counter] = samples->timestamp;
      }
    }
  }

  /* If there is no object in front of the board, look at history to see if a gesture occured */
  if ((ps[0] < PS_THRESHOLD) && (ps[1] < PS_THRESHOLD) && (ps[2] < PS_THRESHOLD))
  {
    /* If the ps_max values are high enough and there exit entry and exit times, */
    /*   then begin processing gestures */
    if ((ps_entry_time[0] != 0) && (ps_entry_time[1] != 0) && (ps_entry_time[2] != 0)
        && (ps_exit_time[0] != 0) && (ps_exit_time[1] != 0) && (ps_exit_time[2] != 0))
    {
      /* Make sure no timestamps overflowed, indicated possibility if any of them are close to overflowing */
      if ((ps_exit_time[0] > 0xFC000000L) || (ps_exit_time[1] > 0xFC000000L) || (ps_exit_time[2] > 0xFC000000L)
          || (ps_entry_time[0] > 0xFC000000L) || (ps_entry_time[1] > 0xFC000000L) || (ps_entry_time[2] > 0xFC000000L))
      {         /* If any of them are close to overflowing, overflow them all so they all have the same reference */
        ps_exit_time[0] += 0x1FFFFFFFL;
        ps_exit_time[1] += 0x1FFFFFFFL;
        ps_exit_time[2] += 0x1FFFFFFFL;

        ps_entry_time[0] += 0x1FFFFFFFL;
        ps_entry_time[1] += 0x1FFFFFFFL;
        ps_entry_time[2] += 0x1FFFFFFFL;
      }

      /* Calculate the midpoint (between entry and exit times) of each waveform */
      /*  the order of these midpoints helps determine the gesture */
      ps_time[0] = (ps_exit_time[0] - ps_entry_time[0]) / 2;
      ps_time[0] = ps_time[0] + ps_entry_time[0];

      ps_time[1] = (ps_exit_time[1] - ps_entry_time[1]) / 2;
      ps_time[1] = ps_time[1] + ps_entry_time[1];

      ps_time[2] = (ps_exit_time[2] - ps_entry_time[2]) / 2;
      ps_time[2] = ps_time[2] + ps_entry_time[2];

      /* The diff_x and diff_y values help determine a gesture by comparing the */
      /*  LED measurements that are on a single axis */
      if (ps_time[1] > ps_time[2])
      {
        diff_x = ps_time[1] - ps_time[2];
      }
      else
      {
        diff_x = ps_time[2] - ps_time[1];
      }
      if( ps_time[0] > ps_time[1] )
      {
        diff_y1 = ps_time[0] - ps_time[1];
      }
	  else
      {
        diff_y1 = ps_time[1] - ps_time[0];
      }

      if( ps_time[0] > ps_time[2] )
      {
        diff_y2 = ps_time[0] - ps_time[2];
      }
	  else
      {
        diff_y2 = ps_time[2] - ps_time[0];
      }


      /* Take the average of all three midpoints to make a comparison point for each midpoint */
      ps_avg = (u32) ps_time[0] + (u32) ps_time[1] + (u32) ps_time[2];
      ps_avg = ps_avg / 3;

      if ((ps_exit_time[0] - ps_entry_time[0]) > 10 || (ps_exit_time[1] - ps_entry_time[1]) > 10 || (ps_exit_time[2] - ps_entry_time[2]) > 10)
      {
        if( ( (ps_time[0] < ps_time[1]) &&  (diff_y1 > diff_x) ) || ( (ps_time[0] <= ps_time[2]) && (diff_y2 > diff_x) ) )
        {           /* An up gesture occured if the bottom LED had its midpoint first */
          ret = UP;
        }
        else if  ( ( (ps_time[0] < ps_time[1]) &&  (diff_y1 > diff_x) ) || ( (ps_time[0] > ps_time[2]) && (diff_y2 > diff_x) ) )
        {           /* A down gesture occured if the bottom LED had its midpoint last */
          ret = DOWN;
        }
        else if((ps_time[0] < ps_time[1]) && (ps_time[2] < ps_time[1]) && (diff_x > ((diff_y1+diff_y2)/2)))
        {           /* A left gesture occured if the left LED had its midpoint last */
          ret = LEFT;
        }
        else if( (ps_time[0] < ps_time[2]) && (ps_time[1] < ps_time[2])  && (diff_x > ((diff_y1+diff_y2)/2)))
        {           /* A right gesture occured if the right LED had midpoint later than the right LED */
          ret = RIGHT;
        }
      }
    }
    for (array_counter = 0; array_counter < 3; array_counter++)
    {
      ps_exit_time[array_counter]  = 0;
      ps_entry_time[array_counter] = 0;
    }
  }

  return ret;
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


