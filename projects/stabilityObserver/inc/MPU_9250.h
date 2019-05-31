/** @addtogroup MPU9250
 *  @{
 *  @file mpu9250.h
 *  @brief Driver to use with MPU9250
 *
 *  @date 19/11/2018
 */
#include <stdint.h>

#ifndef MODULES_LPC4337_M4_DRIVERS_BM_INC_MPU9250_H_
#define MODULES_LPC4337_M4_DRIVERS_BM_INC_MPU9250_H_

#define MPU9250_ADDRESS_AD0H	0x69  	/**< Device address when ADO = 1 */
#define MPU9250_ADDRESS_AD0L 	0x68  	/**< Device address when ADO = 0 */

#define PORT_PIN_SPI_CLK		15		/**<SSP1 Port PIN SCLK*/
#define PIN_SPI_CLK				4		/**<PIN SCLK*/
#define PORT_PIN_SPI_MISO		1		/**<Port PIN MISO*/
#define PIN_SPI_MISO			3		/**<PIN MISO*/
#define PORT_PIN_SPI_MOSI		1		/**<Port PIN MOSI*/
#define PIN_SPI_MOSI			4		/**<PIN MOSI*/
#define PORT_PIN_SPI_CS			6		/**<GPIO0 Port PIN CS*/
#define PIN_SPI_CS				1		/**<PIN CS*/
#define GPIO_PORT_SPI_CS		3		/**<GPIO Port CS*/
#define GPIO_PIN_SPI_CS			0		/**<GPIO Pin Port CS*/


/**brief Ranges available for the gyroscope*/
enum GyroRange{
	GYRO_RANGE_250DPS,					/**<Range 250DPS*/
	GYRO_RANGE_500DPS,					/**<Range 500DPS*/
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
};

/**brief Ranges available for the accelerometer*/
enum AccelRange{
	ACCEL_RANGE_2G,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
};

/**brief Ranges available for the DLPF*/
enum DlpfBandwidth{
	 DLPF_BANDWIDTH_184HZ,
	 DLPF_BANDWIDTH_92HZ,
	 DLPF_BANDWIDTH_41HZ,
	 DLPF_BANDWIDTH_20HZ,
	 DLPF_BANDWIDTH_10HZ,
	 DLPF_BANDWIDTH_5HZ
 };

/**brief Ranges available for the Low power ODR in accelerometer*/
enum LpAccelOdr{
	 LP_ACCEL_ODR_0_24HZ = 0,
	 LP_ACCEL_ODR_0_49HZ = 1,
	 LP_ACCEL_ODR_0_98HZ = 2,
	 LP_ACCEL_ODR_1_95HZ = 3,
	 LP_ACCEL_ODR_3_91HZ = 4,
	 LP_ACCEL_ODR_7_81HZ = 5,
	 LP_ACCEL_ODR_15_63HZ = 6,
	 LP_ACCEL_ODR_31_25HZ = 7,
	 LP_ACCEL_ODR_62_50HZ = 8,
	 LP_ACCEL_ODR_125HZ = 9,
	 LP_ACCEL_ODR_250HZ = 10,
	 LP_ACCEL_ODR_500HZ = 11
};

uint8_t  I2CWrite( uint8_t, uint8_t* , uint16_t );

uint8_t I2CRead( uint8_t,uint8_t*, uint16_t ,uint8_t*, uint16_t);

/**brief Initialize bus I2C to use width MPU9250
 *
 *Use the I2C0 module
 *
 *@param clkHz: Frequency for bus I2C.
 *@param mpu9250address: Address used for MPU9250 on I2C bus.
 *
 */
void MPU9250InitI2C(uint32_t clkHz, uint8_t mpu9250address);

/**brief Initialize SPI to use width MPU9250
 *
 *Use the SSP1 module, and GPIO0 for CS
 *
 *@param clkHz: Frequency for bus SPI.
 */
void MPU9250InitSPI(uint32_t clkHz);

/**brief Initialize MPU9250
 *
 *This function must to be call first.
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250Begin();

/**brief Verify communication MPU9250
 *
 *This function read the Who_Am_I register and test with the corresponding values
 *it is 0x71 for MPU9250 and 0x48 for AK8963
 *
 * @return Return 1 if no error was found. A negative number in case of error, -3 if neither were found,
 * -1 when only MPU9250 was found or -2 if only AK8963 was found.
 */
int MPU9250Test();


/**brief Range of Accelerometer
 *
 *@param range: Value width the new range.
 *
 *@return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250SetAccelRange(enum AccelRange range);

/**brief Range of Gyroscope
 *
 *@param range: Value width the new range.
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250SetGyroRange(enum GyroRange range);


/**brief Bandwidth for DLPF filter
 *
 *@param bandwidth: Value width the new bandwidth.
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250SetDlpfBandwidth(enum DlpfBandwidth bandwidth);

/**brief Sample Rate Divisor
 *
 *@param sampleRateDiv: Value width the new sample rate divisor.
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250SetSrd(uint8_t sampleRateDiv);

/**brief Data Ready Interrupt Enable
 *
 *Configure the interruption width a 50us pulse.
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250EnableDataReadyInterrupt();

/**brief Data Ready Interrupt Disable
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250DisableDataReadyInterrupt();

/**brief Wake on Motion
 *
 *Enable wake on motion.
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250EnableWakeOnMotion(float womThresh_mg, enum LpAccelOdr odr);


/**brief Get gyroscope data
 *
 *Read the data of gyroscope.
 *
 *@param gyroX: Pointer where to save data X of gyroscope
 *@param gyroY: Pointer where to save data Y of gyroscope
 *@param gyroZ: Pointer where to save data Z of gyroscope
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250ReadGyroscope(float *gyroX, float *gyroY, float *gyroZ);

/**brief Get accelerometer data
 *
 *Read the data of accelerometer.
 *
 *@param acceX: Pointer where to save data X of accelerometer
 *@param acceY: Pointer where to save data Y of accelerometer
 *@param acceZ: Pointer where to save data Z of accelerometer
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250ReadAccelerometer(float *acceX, float *acceY, float *acceZ);

/**brief Get magnetometer data
 *
 *Read the data of magnetometer.
 *
 *@param magX: Pointer where to save data X of magnetometer
 *@param magY: Pointer where to save data Y of magnetometer
 *@param magZ: Pointer where to save data Z of magnetometer
 *
 *@return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250ReadMagnetometer(float *magX, float *magY, float *magZ);

/**brief Get temperature data
 *
 *Read the data of temperature.
 *
 *@param temp: Pointer where to save data of temperature
 *
 *@return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250ReadTemperature(float *temp);

/**brief Get sensor data
 *
 *Read the data of accelerometer, temperature, gyroscope, and magnetometer.
 *Always call this function before use the getters.
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250ReadSensor();

/**brief Get accelerometer X value
 *
 *It is the getter for the X value of the accelerometer, read using MPU9250ReadSensor() function.
 *
 * @return Return the accelerometer X value.
 */
float MPU9250GetAccelX_mss();

/**brief Get accelerometer Y value
 *
 *It is the getter for the Y value of the accelerometer, read using MPU9250ReadSensor() function.
 *
 * @return Return the accelerometer Y value.
 */
float MPU9250GetAccelY_mss();

/**brief Get accelerometer Z value
 *
 *It is the getter for the Z value of the accelerometer, read using MPU9250ReadSensor() function.
 *
 * @return Return the accelerometer Z value.
 */
float MPU9250GetAccelZ_mss();

/**brief Get gyroscope X value
 *
 *It is the getter for the X value of the gyroscope, read using MPU9250ReadSensor() function.
 *
 * @return Return the gyroscope X value.
 */
float MPU9250GetGyroX_rads();

/**brief Get gyroscope Y value
 *
 *It is the getter for the Y value of the gyroscope, read using MPU9250ReadSensor() function.
 *
 * @return Return the gyroscope Y value.
 */
float MPU9250GetGyroY_rads();

/**brief Get gyroscope Z value
 *
 *It is the getter for the Z value of the gyroscope, read using MPU9250ReadSensor() function.
 *
 * @return Return the gyroscope Z value.
 */
float MPU9250GetGyroZ_rads();

/**brief Get magnetometer X value
 *
 *It is the getter for the X value of the magnetometer, read using MPU9250ReadSensor() function.
 *
 * @return Return the magnetometer X value.
 */
float MPU9250GetMagX_uT();

/**brief Get magnetometer Y value
 *
 *It is the getter for the Y value of the magnetometer, read using MPU9250ReadSensor() function.
 *
 * @return Return the magnetometer Y value.
 */
float MPU9250GetMagY_uT();

/**brief Get magnetometer Z value
 *
 *It is the getter for the Z value of the magnetometer, read using MPU9250ReadSensor() function.
 *
 * @return Return the magnetometer Z value.
 */
float MPU9250GetMagZ_uT();

/**brief Get temperature value
 *
 *It is the getter for the temperature, read using MPU9250ReadSensor() function.
 *
 * @return Return the temperature.
 */
float MPU9250GetTemperature_C();

/**brief Calibrate gyroscope
 *
 *Read sensor data about 4 seconds and get new offset.
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250CalibrateGyro();

/**brief Get offset GyroX
 *
 *It is the getter for the GryoX offset.
 *
 * @return Return GryoX offset.
 */
float MPU9250GetGyroBiasX_rads();

/**brief Get offset GyroY
 *
 *It is the getter for the GryoY offset.
 *
 * @return Return GryoY offset.
 */
float MPU9250GetGyroBiasY_rads();

/**brief Get offset GyroZ
 *
 *It is the getter for the GryoZ offset.
 *
 * @return Return GryoZ offset.
 */
float MPU9250GetGyroBiasZ_rads();

/**brief Set new GyroX offset
 *
 *Configure the GryoX offset.
 *
 *@param bias: Is the new offset
 */
void MPU9250SetGyroBiasX_rads(float bias);

/**brief Set new GyroY offset
 *
 *Configure the GryoY offset.
 *
 *@param bias: Is the new offset
 */
void MPU9250SetGyroBiasY_rads(float bias);

/**brief Set new GyroZ offset
 *
 *Configure the GryoZ offset.
 *
 *@param bias: Is the new offset
 */
void MPU9250SetGyroBiasZ_rads(float bias);

/**brief Calibrate accelerometer
 *
 *Read sensor data about 4 seconds, and get new offset for accelerometer.
 *
 *@return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250CalibrateAccel();

/**brief Get X offset for Accelerometer
 *
 *It is the getter for the X offset.
 *
 * @return Return accelerometer X offset.
 */
float MPU9250GetAccelBiasX_mss();

/**brief Get X Scale Factor for Accelerometer
 *
 *It is the getter for the X scale factor.
 *
 * @return Return accelerometer X scale factor.
 */
float MPU9250GetAccelScaleFactorX();

/**brief Get Y offset for Accelerometer
 *
 *It is the getter for the Y offset.
 *
 * @return Return accelerometer Y offset.
 */
float MPU9250GetAccelBiasY_mss();

/**brief Get Y Scale Factor for Accelerometer
 *
 *It is the getter for the Y scale factor.
 *
 * @return Return accelerometer Y scale factor.
 */
float MPU9250GetAccelScaleFactorY();

/**brief Get Z offset for Accelerometer
 *
 *It is the getter for the Z offset.
 *
 * @return Return accelerometer Z offset.
 */
float MPU9250GetAccelBiasZ_mss();

/**brief Get Z Scale Factor for Accelerometer
 *
 *It is the getter for the Z scale factor.
 *
 * @return Return accelerometer Z scale factor.
 */
float MPU9250GetAccelScaleFactorZ();

/**brief Set X Accelerometer offset and scale factor
 *
 *Configure the offset and scale factor.
 *
 *@param bias: Is the new offset
 *@param scaleFactor: Is the new scale factor
 */
void MPU9250SetAccelCalX(float bias, float scaleFactor);

/**brief Set Y Accelerometer offset and scale factor
 *
 *Configure the offset and scale factor.
 *
 *@param bias: Is the new offset
 *@param scaleFactor: Is the new scale factor
 */
void MPU9250SetAccelCalY(float bias, float scaleFactor);

/**brief Set Z Accelerometer offset and scale factor
 *
 *Configure the offset and scale factor.
 *
 *@param bias: Is the new offset
 *@param scaleFactor: Is the new scale factor
 */
void MPU9250SetAccelCalZ(float bias, float scaleFactor);

/**brief Calibrate magnetometer
 *
 *Read sensor data about 4 seconds and get new offset for magnetometer.
 *
 * @return Return 1 if no error was found. A negative number in case of error.
 */
int MPU9250CalibrateMag();

/**brief Get X offset for Magnetometer
 *
 *It is the getter for the X offset.
 *
 * @return Return magnetometer X offset.
 */
float MPU9250GetMagBiasX_uT();

/**brief Get X Scale Factor for Magnetometer
 *
 *It is the getter for the X scale factor.
 *
 * @return Return magnetometer X scale factor.
 */
float MPU9250GetMagScaleFactorX();

/**brief Get Y offset for Magnetometer
 *
 *It is the getter for the Y offset.
 *
 * @return Return magnetometer Y offset.
 */
float MPU9250GetMagBiasY_uT();

/**brief Get Y Scale Factor for Magnetometer
 *
 *It is the getter for the Y scale factor.
 *
 * @return Return magnetometer Y scale factor.
 */
float MPU9250GetMagScaleFactorY();

/**brief Get Z offset for Magnetometer
 *
 *It is the getter for the Z offset.
 *
 * @return Return magnetometer Z offset.
 */
float MPU9250GetMagBiasZ_uT();

/**brief Get Z Scale Factor for Magnetometer
 *
 *It is the getter for the Z scale factor.
 *
 * @return Return magnetometer Z scale factor.
 */
float MPU9250GetMagScaleFactorZ();

/**brief Set X Magnetometer offset and scale factor
 *
 *Configure the offset and scale factor.
 *
 *@param bias: Is the new offset
 *@param scaleFactor: Is the new scale factor
 */
void MPU9250SetMagCalX(float bias, float scaleFactor);

/**brief Set Y Magnetometer offset and scale factor
 *
 *Configure the offset and scale factor.
 *
 *@param bias: Is the new offset
 *@param scaleFactor: Is the new scale factor
 */
void MPU9250SetMagCalY(float bias, float scaleFactor);

/**brief Set Z Magnetometer offset and scale factor
 *
 *Configure the offset and scale factor.
 *
 *@param bias: Is the new offset
 *@param scaleFactor: Is the new scale factor
 */
void MPU9250SetMagCalZ(float bias, float scaleFactor);

int MPU9250EnableFifo(uint8_t accel, uint8_t gyro, uint8_t mag, uint8_t temp);
int MPU9250ReadFifo();
void MPU9250GetFifoAccelX_mss(uint8_t *size, float *data);
void MPU9250GetFifoAccelY_mss(uint8_t *size,float *data);
void MPU9250GetFifoAccelZ_mss(uint8_t *size,float *data);
void MPU9250GetFifoGyroX_rads(uint8_t *size,float *data);
void MPU9250GetFifoGyroY_rads(uint8_t *size,float *data);
void MPU9250GetFifoGyroZ_rads(uint8_t *size,float *data);
void MPU9250GetFifoMagX_uT(uint8_t *size,float *data);
void MPU9250GetFifoMagY_uT(uint8_t *size,float *data);
void MPU9250GetFifoMagZ_uT(uint8_t *size,float *data);
void MPU9250GetFifoTemperature_C(uint8_t *size,float *data);

#endif /* MODULES_LPC4337_M4_DRIVERS_BM_INC_MPU9250_H_ */

/** @}*/
