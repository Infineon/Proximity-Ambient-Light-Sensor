/**
 * @page The Arduino Library for the proximity and ambient light sensor PALS-2
 * @section introduction Introduction
 
 * The Infineon PALS-2 is a proximity and ambient light
 * sensor. It offers proximity and ambient light readings with 16-bit resolution. I2C protocol
 * is used to communicate with the host microcontroller. It can be used for gesture recognition,
 * touch screen locking and dimming of displays.
 *
 * For the proximity function there are a built-in IRED driver and photo-pin-diode. LED driver
 * current can be programmed and up to 3 external IREDs can be connected. Offset compensation can be
 * enabled for the proximity measurement; with this feature the sensor writes the difference between
 * the normal proximity value and the estimated offset into the corresponding register.
 *
 * For the ambient light function there is one photo-pin-diode. Two additional photodiodes can receive
 * light in the blue area.
 *
 * Other features include: readouts either periodically or on-demand; interrupts for both functions, with
 * adjustable lower/upper thresholds and persistence.
 */

#ifndef PALS2_H_INCLUDED
#define PALS2_H_INCLUDED

#define PALS2_NUM_REG 			44
#define PALS2_REG_SIZE_BYTES	1
#define SLAVE_ADDRESS 			0x13  //0x26 left-shifted 

/* Configuration */
#define CONFIG_REGISTER		    0x80
#define PROD_REVISION			0x81	
#define PROXIMITY_CONFIG		0x82
#define PROXIMITY_ADC_GAIN      0x83
#define IRED_CONFIG				0x84
#define ALS_CONFIG				0x85
#define MISC_CONFIG				0x86
/* Ambient light */
#define ALS_MSB			   		0X87
#define ALS_LSB					0x88
#define ALS_BLUE_MSB			0x89
#define ALS_BLUE_LSB			0x8A
#define ALS_DARK_MSB			0x8B
#define ALS_DARK_LSB		  	0x8C
#define ALS_BLUE_DARK_MSB       0x8D
#define ALS_BLUE_DARK_LSB	    0x8E
/* Proximity */
#define PROXIMITY_MSB			0x8F
#define PROXIMITY_LSB			0x90
/* Interrupt */
#define INTERRUPT_THR_CONFIG    0x91
#define INTERRUPT_MISC_CONFIG	0x92
#define PROX_INT_BOT_MSB		0x93
#define PROX_INT_BOT_LSB		0x94
#define PROX_INT_TOP_MSB		0x95
#define PROX_INT_TOP_LSB		0x96
#define ALS_INT_BOT_MSB			0x97
#define ALS_INT_BOT_LSB			0x98
#define ALS_INT_TOP_MSB			0x99
#define ALS_INT_TOP_LSB			0x9A
#define INTERRPUT_STATUS 		0x9B
/*LED control */
#define LED_CTRL 				0x9C

/* Config params*/
#define ENABLE 1
#define DISABLE 0

#define IRED1 0
#define IRED2 1
#define IRED3 2

#define PS_THRESHOLD 50000

#include <Arduino.h>
#include <Wire.h>


typedef enum
{
  NONE,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  PROX
} gesture_t;


typedef struct
{
  uint32_t timestamp;         /* Timestamp to record */
  uint16_t ps1;               /* PS1 */
  uint16_t ps2;               /* PS2 */
  uint16_t ps3;               /* PS3 */
} proximity_Samples_t;


class Pals2 {
public:
	Pals2();

/*-----------------------------------------------------------------------------------------------------------------------*/
/* General 																										         */
/*-----------------------------------------------------------------------------------------------------------------------*/
	/**
	 * @brief Starts the sensor.
	 */
	void begin(void);
	/**
	 * @brief Resets the Proximity,ALS and Interrupt configuration.
	 */	
	void resetSensor(void); 
	/**
	 * @brief Sets sensor in standby mode.
	 */	
	void wakeUpSensor(void); 

/*-----------------------------------------------------------------------------------------------------------------------*/
/* Proximity 																										     */
/*-----------------------------------------------------------------------------------------------------------------------*/
	/**
	 * @brief Enables selt-timed measurement of raw proximity values. 
	 * @param ENABLE/DISABLE
	 */
	void setProximityPeriodicMeasurements(uint8_t enable_value);
	
	/**
	 * @brief Sets the measurement rate of proximity measurement.
	 * @param rate Number of measurements per second. Can be one of the numbers from [2, 4, 8, 16, 32, 64, 128, 256].
	 */
	void setProximityMeasurementRate(uint16_t rate);

	/**
	 * @brief Selects the external IRED for proximity measurement and its current.
	 * @param ired External IRED( IRED1, IRED2, IRED3)
	 * @param current 0-20 max. Any value exceeding 20 is cut to 20. 
	 */
	void setProximityOutput(uint8_t ired, uint8_t current);
	
	/**
	 * @brief Sets the number of consecutive measurements needed above/below the threshold for an interrupt to be generated.
	 * @param persistence Number of valid measurements needed, which is one of the numbers from [1, 2, 4, 8, 16, 32, 64, 128].
	 */
	void setInterruptPersistence(uint8_t persistence);
	
	/**
	 * @brief Enables on-demand measurement of raw proximity values. Busy waiting is used for the values to be ready.
	 * @return a single raw proximity value measured on demand.
	 */
	uint16_t getRawProximityOnDemand(void);

	/**
	 * @brief Enables/disables interrupts for proximity measurement and sets the lower/upper thresholds.
	 * @param topThreshold Upper threshold. By default 65536
	 * @param ENABLE/DISABLE
	 * @param bottomThreshold Lower threshold. By default 0
	 * @param ENABLE/DISABLE
	 */
	void setProximityInterrupt(uint16_t topThreshold, uint8_t valuetop, uint16_t bottomThreshold, uint8_t valuebottom);

	/**
	 * @brief Clears proximity-related inputs
	 *
	 * @return True if operation successful. False otherwise.
	 */	 	
	void clearProximityInterrupts(void);
	


	/**
	 * @brief Reads the proximity level as a 16-bit value
	 *
	 * @return True if operation successful. False otherwise.
	 */	 
	uint16_t readProximity(void);
	

/*-----------------------------------------------------------------------------------------------------------------------*/
/* Ambient Light Sensor 																						         */
/*-----------------------------------------------------------------------------------------------------------------------*/

void setAmbientLightPeriodicMeasurements(uint8_t enable_value);
	/**
	 * @brief Enables on-demand measurement of raw als values. Busy waiting is used for the values to be ready.
	 * @return a single als proximity value measured on demand.
	 */
	uint16_t getRawAmbientLightOnDemand(void);
	/**
	 * @brief Enables interrupts for ambient light measurement and sets the lower/upper thresholds.
	 * @param topThreshold Upper threshold. By default 65536
	 * @param bottomThreshold Lower threshold. By default 0
	 */
	void setAmbientLightInterrupt(uint16_t topThreshold, uint8_t valuetop, uint16_t bottomThreshold, uint8_t valuebottom);	 

	/**
	 * @brief Sets the ADC gain, which affects the calculation of illuminance. A higher ADC gain leads to a higher illuminance value.
	 * @param adcGain ADC gain in fA, can be 200/800/3200/25600; for any other value the default (200 fA) is taken
	 * @param isHalved is true/false to indicate whether the gain should be halved and gain some Lx
	 */
	void setAmbientLightADCGain(uint16_t adcGain, bool isHalved);
	/**
	 * @brief Sets the rate of ambient light measurement.
	 * @alsRate Number of measurements per second, which is an integer from 1 to 8
	 */
	void setAmbientLightMeasurementRate(uint8_t alsRate);
	

	void clearAmbientLightInterrupts(void);
    uint16_t readAmbientLight(void);

 
 
	void writeOut(uint16_t regNum, uint16_t val);
	uint8_t dumpRegister(uint8_t regNum);
	
private:
	bool colorCompensationEnabled = false;
	uint8_t proximityConfig = 0;
	uint8_t outputonfig =0;
	uint8_t ambientLightConfig = 0;
	uint8_t interruptConfig = 0;
	float gainFactor = 81.79;
	uint16_t rawProximity = 0;
	uint16_t rawAmbientLight = 0;
	float getBlueRatio(void);
	uint16_t blue1PD = 0;
	uint16_t blue2PD = 0;
	
	uint16_t concatResults(uint8_t upperByte, uint8_t lowerByte);	
	uint8_t wireReadDataByte(uint8_t reg);
	int  wireReadDataBlock(   uint8_t reg, uint8_t *val, unsigned int len);
	bool wireWriteDataByte(uint8_t reg, uint8_t val);
	bool wireWriteDataBlock(  uint8_t reg, uint8_t *val, unsigned int len);
};

#endif		/* PALS2_H_INCLUDED */
