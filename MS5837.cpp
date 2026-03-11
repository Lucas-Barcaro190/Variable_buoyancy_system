#include "MS5837.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "FreeRTOS.h"
#include "task.h"

const uint8_t MS5837_ADDR = 0x76;
const uint8_t MS5837_RESET = 0x1E;
const uint8_t MS5837_ADC_READ = 0x00;
const uint8_t MS5837_PROM_READ = 0xA0;
const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

volatile uint32_t air_pressure = 101300;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;
const uint8_t MS5837::MS5837_UNRECOGNISED = 255;

// context: https://github.com/ArduPilot/ardupilot/pull/29122#issuecomment-2877269114
const uint16_t MS5837_02BA_MAX_SENSITIVITY = 49000;
const uint16_t MS5837_02BA_30BA_SEPARATION = 37000;
const uint16_t MS5837_30BA_MIN_SENSITIVITY = 26000;

MS5837::MS5837() {
	fluidDensity = 1029;
}

bool MS5837::init(i2c_inst_t *i2c_instance) {
	_i2c = i2c_instance; // Store the I2C instance

	// Reset the MS5837, per datasheet
	uint8_t reset_cmd = MS5837_RESET;
	i2c_write_blocking(_i2c, MS5837_ADDR, &reset_cmd, 1, false);

	// Wait for reset to complete
	sleep_ms(10);

	// Read calibration values and CRC
	for ( uint8_t i = 0 ; i < 7 ; i++ ) {
		uint8_t prom_addr = MS5837_PROM_READ + i*2;
		i2c_write_blocking(_i2c, MS5837_ADDR, &prom_addr, 1, true);

		uint8_t calibration_data[2];
		i2c_read_blocking(_i2c, MS5837_ADDR, calibration_data, 2, false);
		C[i] = (calibration_data[0] << 8) | calibration_data[1];
	}

	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	if ( crcCalculated != crcRead ) {
		return false; // CRC fail
	}

	// PROM Word 1 represents the sensor's pressure sensitivity calibration
	// Set _model according to the experimental pressure sensitivity thresholds
	if (C[1] < MS5837_30BA_MIN_SENSITIVITY || C[1] > MS5837_02BA_MAX_SENSITIVITY)
	{
		_model = MS5837_UNRECOGNISED;
	}
	else if (C[1] > MS5837_02BA_30BA_SEPARATION)
	{
		_model = MS5837_02BA;
	}
	else
	{
		_model = MS5837_30BA;
	}

	// TODO: extract and store/report sensor package type from bits 11-5 of PROM Word 0,
	// per https://github.com/ArduPilot/ardupilot/pull/29122#pullrequestreview-2837597764

	// The sensor has passed the CRC check, so we should return true even if
	// the sensor version is unrecognised.
	// (The MS5637 has the same address as the MS5837 and will also pass the CRC check)
	// (but will hopefully be unrecognised.)
	return true;
}

void MS5837::setModel(uint8_t model) {
	_model = model;
}

uint8_t MS5837::getModel() {
	return (_model);
}

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
}

void MS5837::read() {
	// Check that _i2c is not NULL (i.e. has the user forgotten to call .init?)
	if (_i2c == NULL)
	{
		return;
	}

	// Request D1 conversion
	uint8_t d1_cmd = MS5837_CONVERT_D1_8192;
	i2c_write_blocking(_i2c, MS5837_ADDR, &d1_cmd, 1, false);

	vTaskDelay(20); // Max conversion time per datasheet

	uint8_t adc_read_cmd = MS5837_ADC_READ;
	i2c_write_blocking(_i2c, MS5837_ADDR, &adc_read_cmd, 1, true);

	uint8_t d1_data[3];
	i2c_read_blocking(_i2c, MS5837_ADDR, d1_data, 3, false);
	D1 = 0;
	D1 = d1_data[0];
	D1 = (D1 << 8) | d1_data[1];
	D1 = (D1 << 8) | d1_data[2];

	// Request D2 conversion
	uint8_t d2_cmd = MS5837_CONVERT_D2_8192;
	i2c_write_blocking(_i2c, MS5837_ADDR, &d2_cmd, 1, false);

	vTaskDelay(20); // Max conversion time per datasheet

	adc_read_cmd = MS5837_ADC_READ;
	i2c_write_blocking(_i2c, MS5837_ADDR, &adc_read_cmd, 1, true);

	uint8_t d2_data[3];
	i2c_read_blocking(_i2c, MS5837_ADDR, d2_data, 3, false);
	D2 = 0;
	D2 = d2_data[0];
	D2 = (D2 << 8) | d2_data[1];
	D2 = (D2 << 8) | d2_data[2];

	calculate();
}

void MS5837::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	// Use of l or LL to make sure the 32-bit RP2040 architecture does not overflow with the 64-bit calculations
	// required for the compensation algorithm

	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	// Terms called
	dT = D2-uint32_t(C[5])*256l;
	if ( _model == MS5837_02BA ) {
		SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
		OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
		P = (D1*SENS/(2097152l)-OFF)/(32768l);
	} else {
		SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
		OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
		P = (D1*SENS/(2097152l)-OFF)/(8192l);
	}

	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;

	//Second order compensation
	if ( _model == MS5837_02BA ) {
		if((TEMP/100)<20){         //Low temp
			Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	} else {
		if((TEMP/100)<20){         //Low temp
			Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15){    //Very low temp
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20){    //High temp
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}

	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;

	TEMP = (TEMP-Ti);

	if ( _model == MS5837_02BA ) {
		P = (((D1*SENS2)/2097152l-OFF2)/32768l);
	} else {
		P = (((D1*SENS2)/2097152l-OFF2)/8192l);
	}
}

float MS5837::pressure(float conversion) {
	if ( _model == MS5837_02BA ) {
		return P*conversion/100.0f;
	}
	else {
		return P*conversion/10.0f;
	}
}

float MS5837::temperature() {
	return TEMP/100.0f;
}

// The pressure sensor measures absolute pressure, so it will measure the atmospheric pressure + water pressure
// We subtract the atmospheric pressure to calculate the depth with only the water pressure
// The average atmospheric pressure of 101300 pascal is used for the calcuation, but atmospheric pressure varies
// If the atmospheric pressure is not 101300 at the time of reading, the depth reported will be offset
// In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and
// that value should subtracted for subsequent depth calculations.
float MS5837::depth() {
	return (pressure(MS5837::Pa)-air_pressure)/(fluidDensity*9.80665);
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}