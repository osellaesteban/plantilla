/*
 * MPU9250.c
 *
 *  Created on: Nov 19, 2018
 *      Author: German E. Hachmann, Juan Carlos Neville
 */

#include "MPU_9250.h"

#include <chip.h>
#include "stopwatch.h"
#include <stdlib.h>

int WriteRegister(uint8_t regAddress, uint8_t data);
int ReadRegisters(uint8_t regAddress, uint8_t count, uint8_t *dest);
int WriteAK8963Register(uint8_t regAddress, uint8_t data, uint8_t testBack);
int ReadAK8963Registers(uint8_t regAddress, uint8_t count, uint8_t* dest);
int WhoAmI(void);
int WhoAmIAK8963(void);

#define WHO_AM_I_AK8963  	0x00 	/**<Device ID should return 0x48*/
#define INFO             	0x01	/**<Information*/
#define AK8963_ST1       	0x02  	/**<Status 1*/
#define AK8963_XOUT_L    	0x03  	/**<Measurement Data*/
#define AK8963_XOUT_H    	0x04
#define AK8963_YOUT_L    	0x05
#define AK8963_YOUT_H    	0x06
#define AK8963_ZOUT_L    	0x07
#define AK8963_ZOUT_H    	0x08
#define AK8963_ST2       	0x09  	/**<Status 2*/
#define AK8963_CNTL1     	0x0A  	/**<Control 1*/
#define AK8963_CNTL2     	0x0B	/**<Control 2*/
#define AK8963_ASTC      	0x0C  	/**<Self-Test Control*/
#define AK8963_I2CDIS    	0x0F  	/**<I2C Disable*/
#define AK8963_ASAX      	0x10  	/**<Sensitivity Adjustment values*/
#define AK8963_ASAY      	0x11
#define AK8963_ASAZ      	0x12

#define SELF_TEST_X_GYRO 	0x00	/**<Gyroscope Self-Test Registers*/
#define SELF_TEST_Y_GYRO 	0x01
#define SELF_TEST_Z_GYRO 	0x02
#define SELF_TEST_X_ACCEL 	0x0D	/**<Accelerometer Self-Test Registers*/
#define SELF_TEST_Y_ACCEL 	0x0E
#define SELF_TEST_Z_ACCEL 	0x0F
#define XG_OFFSET_H       	0x13  	/**<Gyro Offset Registers*/
#define XG_OFFSET_L       	0x14
#define YG_OFFSET_H       	0x15
#define YG_OFFSET_L       	0x16
#define ZG_OFFSET_H       	0x17
#define ZG_OFFSET_L       	0x18
#define SMPLRT_DIV        	0x19	/**<Sample Rate Divider*/
#define CONFIG            	0x1A	/**<Configuration*/
#define GYRO_CONFIG       	0x1B	/**Gyroscope Configuration*/
#define ACCEL_CONFIG      	0x1C	/**Accelerometer Configuration 1*/
#define ACCEL_CONFIG2     	0x1D	/**Accelerometer Configuration 2*/
#define LP_ACCEL_ODR      	0x1E	/**<Low Power Accelerometer ODR Control*/
#define WOM_THR           	0x1F	/**<Wake-on Motion Threshold*/
#define FIFO_EN            	0x23	/**<FIFO Enable*/
#define I2C_MST_CTRL       	0x24	/**<I2C Master Control*/
#define I2C_SLV0_ADDR      	0x25	/**<I2C Slave 0 Control*/
#define I2C_SLV0_REG       	0x26	/**<I2C Slave 0 Control*/
#define I2C_SLV0_CTRL      	0x27	/**<I2C Slave 0 Control*/
#define I2C_SLV1_ADDR      	0x28	/**<I2C Slave 1 Control*/
#define I2C_SLV1_REG       	0x29	/**<I2C Slave 1 Control*/
#define I2C_SLV1_CTRL      	0x2A	/**<I2C Slave 1 Control*/
#define I2C_SLV2_ADDR      	0x2B	/**<I2C Slave 2 Control*/
#define I2C_SLV2_REG       	0x2C	/**<I2C Slave 2 Control*/
#define I2C_SLV2_CTRL      	0x2D	/**<I2C Slave 2 Control*/
#define I2C_SLV3_ADDR      	0x2E	/**<I2C Slave 3 Control*/
#define I2C_SLV3_REG       	0x2F	/**<I2C Slave 3 Control*/
#define I2C_SLV3_CTRL      	0x30	/**<I2C Slave 3 Control*/
#define I2C_SLV4_ADDR      	0x31	/**<I2C Slave 4 Control*/
#define I2C_SLV4_REG       	0x32	/**<I2C Slave 4 Control*/
#define I2C_SLV4_DO        	0x33	/**<I2C Slave 4 Control*/
#define I2C_SLV4_CTRL      	0x34	/**<I2C Slave 4 Control*/
#define I2C_SLV4_DI        	0x35	/**<I2C Slave 4 Control*/
#define I2C_MST_STATUS     	0x36	/**<I2C Master Status*/
#define INT_PIN_CFG        	0x37	/**<INT Pin / Bypass Enable Configuration*/
#define INT_ENABLE         	0x38	/**<Interrupt Enable*/
#define INT_STATUS         	0x3A	/**<Interrupt Status*/
#define ACCEL_XOUT_H       	0x3B	/**<Accelerometer Measurements*/
#define ACCEL_XOUT_L       	0x3C
#define ACCEL_YOUT_H       	0x3D
#define ACCEL_YOUT_L       	0x3E
#define ACCEL_ZOUT_H       	0x3F
#define ACCEL_ZOUT_L       	0x40
#define TEMP_OUT_H         	0x41	/**<Temperature Measurement*/
#define TEMP_OUT_L         	0x42
#define GYRO_XOUT_H        	0x43	/**<Gyroscope Measurements*/
#define GYRO_XOUT_L        	0x44
#define GYRO_YOUT_H        	0x45
#define GYRO_YOUT_L        	0x46
#define GYRO_ZOUT_H        	0x47
#define GYRO_ZOUT_L        	0x48
#define EXT_SENS_DATA_00   	0x49	/**<External Sensor Data*/
#define EXT_SENS_DATA_01   	0x4A
#define EXT_SENS_DATA_02   	0x4B
#define EXT_SENS_DATA_03   	0x4C
#define EXT_SENS_DATA_04   	0x4D
#define EXT_SENS_DATA_05   	0x4E
#define EXT_SENS_DATA_06   	0x4F
#define EXT_SENS_DATA_07   	0x50
#define EXT_SENS_DATA_08   	0x51
#define EXT_SENS_DATA_09   	0x52
#define EXT_SENS_DATA_10   	0x53
#define EXT_SENS_DATA_11   	0x54
#define EXT_SENS_DATA_12   	0x55
#define EXT_SENS_DATA_13   	0x56
#define EXT_SENS_DATA_14   	0x57
#define EXT_SENS_DATA_15   	0x58
#define EXT_SENS_DATA_16   	0x59
#define EXT_SENS_DATA_17   	0x5A
#define EXT_SENS_DATA_18   	0x5B
#define EXT_SENS_DATA_19   	0x5C
#define EXT_SENS_DATA_20   	0x5D
#define EXT_SENS_DATA_21   	0x5E
#define EXT_SENS_DATA_22   	0x5F
#define EXT_SENS_DATA_23   	0x60
#define I2C_SLV0_DO        	0x63	/**<I2C Slave 0 Data Out*/
#define I2C_SLV1_DO        	0x64	/**<I2C Slave 1 Data Out*/
#define I2C_SLV2_DO        	0x65	/**<I2C Slave 2 Data Out*/
#define I2C_SLV3_DO        	0x66	/**<I2C Slave 3 Data Out*/
#define I2C_MST_DELAY_CTRL 	0x67	/**<I2C Master Delay Control*/
#define SIGNAL_PATH_RESET  	0x68	/**<Signal Path Reset*/
#define ACCEL_INTEL_CTRL   	0x69	/**<Accelerometer Interrupt Control*/
#define USER_CTRL          	0x6A  	/**<USER_CTRL*/
#define PWR_MGMT_1         	0x6B 	/**<Power Management 1*/
#define PWR_MGMT_2         	0x6C	/**<Power Management 2*/
#define FIFO_COUNTH        	0x72	/**<FIFO Count Registers*/
#define FIFO_COUNTL        	0x73
#define FIFO_R_W           	0x74	/**<Read/Write command provides Read or Write operation for the FIFO.*/
#define WHO_AM_I_MPU9250   	0x75 	/**<Should return 0x71 */
#define XA_OFFSET_H        	0x77	/**<Accelerometer Offset Registers*/
#define XA_OFFSET_L        	0x78
#define YA_OFFSET_H        	0x7A
#define YA_OFFSET_L        	0x7B
#define ZA_OFFSET_H        	0x7D
#define ZA_OFFSET_L        	0x7E

#define AK8963_ADDRESS  	0x0C   	/**< Address of Magnetometer*/

enum {
	NONE,
	I2C,
	SPI
}_CommMode = NONE;

const int16_t tX[3] = {0,  1,  0};
const int16_t tY[3] = {1,  0,  0};
const int16_t tZ[3] = {0,  0, -1};
const float _tempScale = 333.87f;
const float _tempOffset = 21.0f;
const float G = 9.807f;
const float _d2r = 3.14159265359f/180.0f;


uint8_t MPU9250_ADDRESS;
uint8_t _buffer[21];
// data counts
int16_t _axcounts, _aycounts,_azcounts;
int16_t _gxcounts,_gycounts,_gzcounts;
int16_t _hxcounts,_hycounts,_hzcounts;
int16_t _tcounts;
// data buffer
float _ax, _ay, _az;
float _gx, _gy, _gz;
float _hx, _hy, _hz;
float _t;
// wake on motion
uint8_t _womThreshold;
// scale factors
float _accelScale;
float _gyroScale;
float _magScaleX, _magScaleY, _magScaleZ;
// configuration
enum AccelRange _accelRange;
enum GyroRange _gyroRange;
enum DlpfBandwidth _bandWidth;
uint8_t _srd;
// gyro bias estimation
uint8_t _numSamples = 100;
double _gxbD, _gybD, _gzbD;
float _gxb, _gyb, _gzb;
// accel bias and scale factor estimation
double _axbD, _aybD, _azbD;
float _axmax, _aymax, _azmax;
float _axmin, _aymin, _azmin;
float _axb, _ayb, _azb;
float _axs = 1.0f;
float _ays = 1.0f;
float _azs = 1.0f;
// magnetometer bias and scale factor estimation
uint16_t _maxCounts = 1000;
float _deltaThresh = 0.3f;
uint8_t _coeff = 8;
uint16_t _counter;
float _framedelta, _delta;
float _hxfilt, _hyfilt, _hzfilt;
float _hxmax, _hymax, _hzmax;
float _hxmin, _hymin, _hzmin;
float _hxb, _hyb, _hzb;
float _hxs = 1.0f;
float _hys = 1.0f;
float _hzs = 1.0f;
float _avgs;
float _accelScale, _gyroScale;
float _magScaleX, _magScaleY, _magScaleZ;
// fifo
uint8_t _enFifoAccel, _enFifoGyro, _enFifoMag, _enFifoTemp;
uint8_t _fifoSize,_fifoFrameSize;
float _axFifo[85], _ayFifo[85], _azFifo[85];
uint8_t _aSize;
float _gxFifo[85], _gyFifo[85], _gzFifo[85];
uint8_t _gSize;
float _hxFifo[73], _hyFifo[73], _hzFifo[73];
uint8_t _hSize;
float _tFifo[256];
uint8_t _tSize;

uint8_t  I2CWrite( uint8_t  i2cSlaveAddress, uint8_t* transmitDataBuffer, uint16_t transmitDataBufferSize )
{

      I2CM_XFER_T i2cData;

      // Prepare the i2cData register
      i2cData.slaveAddr = i2cSlaveAddress;
      i2cData.options   = 0;
      i2cData.status    = 0;
      i2cData.txBuff    = transmitDataBuffer;
      i2cData.txSz      = transmitDataBufferSize;
      i2cData.rxBuff    = 0;
      i2cData.rxSz      = 0;

      /* Send the i2c data */
      if( Chip_I2CM_XferBlocking(LPC_I2C0, &i2cData ) == 0 )
      {
         return FALSE;
      }

      return TRUE;
}

uint8_t I2CRead( uint8_t  i2cSlaveAddress,uint8_t* dataToReadBuffer, uint16_t dataToReadBufferSize,
                  uint8_t* receiveDataBuffer, uint16_t receiveDataBufferSize)
{
      I2CM_XFER_T i2cData;

      i2cData.slaveAddr = i2cSlaveAddress;
      i2cData.options   = 0;
      i2cData.status    = 0;
      i2cData.txBuff    = dataToReadBuffer;
      i2cData.txSz      = dataToReadBufferSize;
      i2cData.rxBuff    = receiveDataBuffer;
      i2cData.rxSz      = receiveDataBufferSize;

      if( Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData ) == 0 )
      {
         return FALSE;
      }

      return TRUE;
}

//---------------------------------------------------------------------------------------------------
int WriteRegister(uint8_t regAddress, uint8_t data){
	uint8_t buf[2];
//	I2CM_XFER_T xfer;

	if(_CommMode == SPI){
		_buffer[0] = regAddress;
		_buffer[1] = data;
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0x03, 0x00);
		if(Chip_SSP_WriteFrames_Blocking(LPC_SSP1, _buffer, 2) != 2)
			return -1;
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0x03, 0x00);
	}
	else{
//		buf[0] = regAddress;
//		buf[1] = data;
//
//		xfer.slaveAddr = MPU9250_ADDRESS;
//		xfer.txBuff = buf;
//		xfer.txSz = 2;
//		xfer.rxBuff = 0;
//		xfer.rxSz = 0;
//
//		if(!Chip_I2CM_XferBlocking(LPC_I2C0, &xfer))
//			return -1;

		_buffer[0] = (MPU9250_ADDRESS << 1);
		_buffer[1] = regAddress;
		_buffer[2] = data;
		if(Chip_I2CM_Write(LPC_I2C0, _buffer, 3) != 3)
			return -1;
		Chip_I2CM_SendStop(LPC_I2C0);
	}

	return 1;
}
//---------------------------------------------------------------------------------------------------
int ReadRegisters(uint8_t regAddress, uint8_t count, uint8_t *dest){
	uint8_t rxLen = 0, err = 0, aux;
	uint8_t buf[2];
	I2CM_XFER_T xfer;

	if(_CommMode == SPI){
		regAddress |= 0x80;

		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0x03, 0x00);
		if(Chip_SSP_WriteFrames_Blocking(LPC_SSP1, &regAddress, 1) != 1){
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0x03, 0x00);
			return -1;
		}
		if(Chip_SSP_ReadFrames_Blocking(LPC_SSP1, dest, count) != count){
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0x03, 0x00);
			return -1;
		}
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0x03, 0x00);
	}
	else{
		buf[0] = regAddress;
		I2CRead( MPU9250_ADDRESS,buf, 1, dest, count);
//		xfer.slaveAddr = MPU9250_ADDRESS;
//		xfer.txBuff = buf;
//		xfer.txSz = 1;
//		xfer.rxBuff = dest;
//		xfer.rxSz = count;
//
//		if(!Chip_I2CM_XferBlocking(LPC_I2C0, &xfer))
//			return -1;
	}

	return 1;
}
//---------------------------------------------------------------------------------------------------
int WriteAK8963Register(uint8_t regAddress, uint8_t data, uint8_t testBack){
	if(_CommMode == I2C)
		return 1;

	if(WriteRegister(I2C_SLV0_ADDR, AK8963_ADDRESS) < 0)
		return -1;
	if(WriteRegister(I2C_SLV0_REG, regAddress) < 0)
		return -2;
	if(WriteRegister(I2C_SLV0_DO, data) < 0)
		return -3;
	if(WriteRegister(I2C_SLV0_CTRL, 0x81) < 0)
		return -4;
	StopWatch_DelayMs(10);
	// read the register and confirm
	if(testBack){
		if(ReadAK8963Registers(regAddress , 1, _buffer) < 0)
			return -5;
 		if(_buffer[0] == data)
 			return 1;
 		else
 			return -6;
	}

	return 1;
}
//---------------------------------------------------------------------------------------------------
int ReadAK8963Registers(uint8_t regAddress, uint8_t count, uint8_t *dest){
	uint8_t value, buf[2];

	if(_CommMode == I2C)
		return 1;

	if(WriteRegister(I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80) < 0)
		return -1;

	if(WriteRegister(I2C_SLV0_REG, regAddress) < 0)
		return -2;

	if(WriteRegister(I2C_SLV0_CTRL, 0x80 | count) < 0)
		return -3;

	StopWatch_DelayMs(10);

	value = ReadRegisters(EXT_SENS_DATA_00, count, dest);

	return value;
}
//---------------------------------------------------------------------------------------------------
int WhoAmI(){
	if(ReadRegisters(WHO_AM_I_MPU9250, 1 , &_buffer[0]) < 0)
		return -1;

	return _buffer[0];
}
//---------------------------------------------------------------------------------------------------
int WhoAmIAK8963(){
	if(_CommMode == I2C)
		return 0x48;

	if(ReadAK8963Registers(WHO_AM_I_AK8963, 1 , &_buffer[0]) < 0)
		return -1;

	return _buffer[0];
}
//---------------------------------------------------------------------------------------------------
int MPU9250Test(){
	int value, buf;

	value = 0;

	//enable I2C Master mode
	if(WriteRegister(USER_CTRL, 0x20) < 0)
		return -10;

	// set the I2C bus speed to 400 kHz
	if(WriteRegister(I2C_MST_CTRL, 0x0D) < 0)
		return -11;

	buf = WhoAmI();
	if(buf < 0)
		value  = -2;
	else{
		if(buf!=0x71 && buf!=0x73)
			value = -2;
	}

	buf = WhoAmIAK8963();
	if(buf < 0)
		value  += -1;
	else{
		if(buf != 0x48)
			value += -1;
	}

	if(value == 0)
		return 1;
	return value;
}
//---------------------------------------------------------------------------------------------------
void MPU9250InitI2C(uint32_t clkHz, uint8_t mpu9250address){
	if(_CommMode == SPI)
		Chip_SSP_DeInit(LPC_SSP1);

	_CommMode = I2C;

	MPU9250_ADDRESS = mpu9250address;
    // Configuracion de las lineas de SDA y SCL de la placa
    Chip_SCU_I2C0PinConfig(I2C0_STANDARD_FAST_MODE);

    // Inicializacion del periferico
    Chip_I2C_Init(I2C0);
    // Seleccion de velocidad del bus
    Chip_I2C_SetClockRate(I2C0, clkHz);
    // Configuracion para que los eventos se resuelvan por polling
    // (la otra opcion es por interrupcion)
    Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);

    StopWatch_Init();

}

//---------------------------------------------------------------------------------------------------
void MPU9250InitSPI(uint32_t clkHz){
	if(_CommMode == I2C)
		Chip_I2CM_DeInit(I2C0);

	_CommMode = SPI;

	/* Set up clock and power for SSP1 module */
	// Configure SSP SSP1 pins
	Chip_SCU_PinMuxSet(0x0F, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CLK0
	Chip_SCU_PinMuxSet(0x01, 3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); // MISO1
	Chip_SCU_PinMuxSet(0x01, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC5)); // MOSI1

	Chip_SCU_PinMuxSet(0x06, 1, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CS1 configured as GPIO
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 0);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 3, 0, 1);

	// Initialize SSP Peripheral
	Chip_SSP_Init(LPC_SSP1);
	Chip_SSP_Set_Mode(LPC_SSP1, SSP_MODE_MASTER);
	Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE3);
	Chip_SSP_SetBitRate(LPC_SSP1, clkHz);
	Chip_SSP_Enable(LPC_SSP1);

	StopWatch_Init();
}

//---------------------------------------------------------------------------------------------------
int MPU9250Begin(){
	if(!MPU9250Test())
		return -100;

	//Reset device
	if(WriteRegister(PWR_MGMT_1, 0x80) < 0)
		return -1;
	StopWatch_DelayMs(1);

	//Enable clock source
	if(WriteRegister(PWR_MGMT_1, 0x01) < 0)
		return -2;
	StopWatch_DelayMs(1);

	//Enable Accelerometer ang Gyroscope
	if(WriteRegister(PWR_MGMT_2, 0x00) < 0)
		return -3;
	StopWatch_DelayMs(1);

	//Reset device
	if(WriteRegister(PWR_MGMT_1, 0x80) < 0)
		return -4;
	StopWatch_DelayMs(1);

	if(WriteRegister(CONFIG, 0x01) < 0)
		return -5;
	StopWatch_DelayMs(1);

	_bandWidth = DLPF_BANDWIDTH_184HZ;

	//Setting the sample rate divider to 0 as default
	if(WriteRegister(SMPLRT_DIV, 0x00) < 0)
		return -6;
	StopWatch_DelayMs(1);

	_srd = 0;

	// setting accel range to 16G as default
	if(WriteRegister(ACCEL_CONFIG, 0x18) < 0)
		return -7;
	StopWatch_DelayMs(1);

	_accelScale = G * 2.0f/32767.5f; // setting the accel scale to 16G
	_accelRange = ACCEL_RANGE_2G;

	// setting the gyro range to 2000DPS as default
	if(WriteRegister(GYRO_CONFIG, 0x18) < 0)
		return -8;
	StopWatch_DelayMs(1);

	_gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
	_gyroRange = GYRO_RANGE_2000DPS;

	//Setting bandwidth to 184Hz as default
	if(WriteRegister(ACCEL_CONFIG2, 0x01) < 0)
		return -9;
	StopWatch_DelayMs(1);

	if(WriteRegister(INT_PIN_CFG, 0x01) < 0)
		return -10;
	StopWatch_DelayMs(1);

	//I2C_Master_Mode
	if(WriteRegister(USER_CTRL, 0x20) < 0)
		return -11;
	StopWatch_DelayMs(1);

	//I2C Mutimaster 400KHz
	if(WriteRegister(I2C_MST_CTRL, 0x0D) < 0)
		return -12;
	StopWatch_DelayMs(1);

	if(WriteAK8963Register(AK8963_CNTL2, 0x01, 0) < 0)
		return -13;
	StopWatch_DelayMs(100);

	if(WriteAK8963Register(AK8963_CNTL1, 0x12, 1) < 0)
		return -14;
	StopWatch_DelayMs(1);

	ReadAK8963Registers(AK8963_ASAX, 3, _buffer);
	_magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	if((WhoAmI() != 113) && (WhoAmI() != 115)){
	    return -15;
	}
	StopWatch_DelayMs(1);

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if(WhoAmIAK8963() != 72)
	    return -16;

	// successful init, return 1
	 return 1;

}
//---------------------------------------------------------------------------------------------------
int MPU9250SetAccelRange(enum AccelRange range){
	switch(range) {
	case ACCEL_RANGE_2G:
		// setting the accel range to 2G
		if(WriteRegister(ACCEL_CONFIG, 0x00) < 0){
			return -1;
	     }
		_accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
		break;
	case ACCEL_RANGE_4G:
		// setting the accel range to 4G
		if(WriteRegister(ACCEL_CONFIG, 0x08) < 0){
			return -1;
		}
		_accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
		break;

	case ACCEL_RANGE_8G:
		// setting the accel range to 8G
		if(WriteRegister(ACCEL_CONFIG, 0x10) < 0){
			return -1;
	     }
		_accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
		break;

	case ACCEL_RANGE_16G:
		// setting the accel range to 16G
		if(WriteRegister(ACCEL_CONFIG, 0x18) < 0){
			return -1;
		}
		_accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
		break;

	}
	_accelRange = range;
	return 1;
}
//---------------------------------------------------------------------------------------------------
int MPU9250SetGyroRange(enum GyroRange range){
	switch(range){
	case GYRO_RANGE_250DPS:
		// setting the gyro range to 250DPS
		if(WriteRegister(GYRO_CONFIG, 0x00) < 0)
			return -1;
		_gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
		break;

	case GYRO_RANGE_500DPS:
		// setting the gyro range to 500DPS
		if(WriteRegister(GYRO_CONFIG, 0x08) < 0)
			return -1;
		_gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
		break;
	case GYRO_RANGE_1000DPS:
		// setting the gyro range to 1000DPS
		if(WriteRegister(GYRO_CONFIG, 0x10) < 0)
			return -1;
		_gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
		break;
	case GYRO_RANGE_2000DPS:
		// setting the gyro range to 2000DPS
		if(WriteRegister(GYRO_CONFIG, 0x18) < 0)
			return -1;
		_gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
		break;
	}
	_gyroRange = range;

	return 1;

}
//---------------------------------------------------------------------------------------------------
int MPU9250SetDlpfBandwidth(enum DlpfBandwidth bandwidth){
	switch(bandwidth){
	case DLPF_BANDWIDTH_184HZ:
		if(WriteRegister(ACCEL_CONFIG2, 0x01) < 0) // setting accel bandwidth to 184Hz
			return -1;
		if(WriteRegister(CONFIG, 0x01) < 0) // setting gyro bandwidth to 184Hz
			return -2;
		break;
	case DLPF_BANDWIDTH_92HZ:
		if(WriteRegister(ACCEL_CONFIG2, 0x02) < 0) // setting accel bandwidth to 92Hz
			return -1;
		if(WriteRegister(CONFIG, 0x02) < 0) // setting gyro bandwidth to 92Hz
			return -2;
		break;
	case DLPF_BANDWIDTH_41HZ:
		if(WriteRegister(ACCEL_CONFIG2, 0x03) < 0) // setting accel bandwidth to 41Hz
			return -1;
		if(WriteRegister(CONFIG, 0x03) < 0) // setting gyro bandwidth to 41Hz
			return -2;
		break;
	case DLPF_BANDWIDTH_20HZ:
		if(WriteRegister(ACCEL_CONFIG2, 0x04) < 0) // setting accel bandwidth to 20Hz
			return -1;
		if(WriteRegister(CONFIG, 0x04) < 0) // setting gyro bandwidth to 20Hz
			return -2;
		break;
	case DLPF_BANDWIDTH_10HZ:
		if(WriteRegister(ACCEL_CONFIG2, 0x05) < 0) // setting accel bandwidth to 10Hz
			return -1;
		if(WriteRegister(CONFIG, 0x05) < 0) // setting gyro bandwidth to 10Hz
			return -2;
		break;
	case DLPF_BANDWIDTH_5HZ:
		if(WriteRegister(ACCEL_CONFIG2, 0x06) < 0) // setting accel bandwidth to 5Hz
			return -1;
		if(WriteRegister(CONFIG, 0x06) < 0) // setting gyro bandwidth to 5Hz
			return -2;
		break;
	}

	_bandWidth = bandwidth;
	return 1;
}
//---------------------------------------------------------------------------------------------------
int MPU9250SetSrd(uint8_t sampleRateDiv){
	if(WriteRegister(SMPLRT_DIV, 19) < 0) // setting the sample rate divider
		return -1;
	if(sampleRateDiv > 9){
		// set AK8963 to Power Down
		if(WriteAK8963Register(AK8963_CNTL1, 0x00, 1) < 0)
			return -2;
		StopWatch_DelayMs(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 8 Hz update rate
		if(WriteAK8963Register(AK8963_CNTL1, 0x12, 1) < 0)
			return -3;
		StopWatch_DelayMs(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		if(ReadAK8963Registers(AK8963_XOUT_H, 7, _buffer) < 0)
			return -4;
	}
	else{
		// set AK8963 to Power Down
		if(WriteAK8963Register(AK8963_CNTL1, 0x00, 1) < 0)
			return -2;
		StopWatch_DelayMs(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		if(WriteAK8963Register(AK8963_CNTL1, 0x16, 1) < 0)
			return -3;
		StopWatch_DelayMs(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		if(ReadAK8963Registers(AK8963_XOUT_H, 7, _buffer) < 0)
			return -4;
	}
	/* setting the sample rate divider */
	if(WriteRegister(SMPLRT_DIV, sampleRateDiv) < 0) // setting the sample rate divider
		return -5;

	_srd = sampleRateDiv;
	return 1;

}
//---------------------------------------------------------------------------------------------------
int MPU9250EnableDataReadyInterrupt(){
	/* setting the interrupt */
	if(WriteRegister(INT_PIN_CFG, 0x00) < 0) // setup interrupt, 50 us pulse
		return -1;
	if(WriteRegister(INT_ENABLE, 0x01) < 0) // set to data ready
		return -2;
	return 1;
}
//---------------------------------------------------------------------------------------------------
int MPU9250DisableDataReadyInterrupt(){
	if(WriteRegister(INT_ENABLE, 0x00) < 0) // disable interrupt
		return -1;
	return 1;
}
//---------------------------------------------------------------------------------------------------
int MPU9250EnableWakeOnMotion(float womThresh_mg, enum LpAccelOdr odr){
	// set AK8963 to Power Down
	if(WriteAK8963Register(AK8963_CNTL1, 0x00, 1) < 0)
		return -1;
	// reset the MPU9250
	if(WriteRegister(PWR_MGMT_1, 0x80) < 0)
		return -2;
	// wait for MPU-9250 to come back up
	StopWatch_DelayMs(1);
	if(WriteRegister(PWR_MGMT_1, 0x00) < 0) // cycle 0, sleep 0, standby 0
		return -3;
	if(WriteRegister(PWR_MGMT_2, 0x07) < 0) // disable gyro measurements
		return -4;
	if(WriteRegister(ACCEL_CONFIG2, 0x01) < 0) // setting accel bandwidth to 184Hz
		return -5;
	if(WriteRegister(INT_ENABLE, 0x40) < 0) // enabling interrupt to wake on motion
		return -6;
	if(WriteRegister(ACCEL_INTEL_CTRL, (0x80 | 0x40)) < 0) // enabling accel hardware intelligence
		return -7;
	_womThreshold = (womThresh_mg*255) / 1020;
	if(WriteRegister(WOM_THR,_womThreshold) < 0) // setting wake on motion threshold
		return -8;
	if(WriteRegister(LP_ACCEL_ODR,(uint8_t)odr) < 0) // set frequency of wakeup
		return -9;
	if(WriteRegister(PWR_MGMT_1, 0x20) < 0) // switch to accel low power mode
		return -10;

	return 1;
}
//---------------------------------------------------------------------------------------------------
int MPU9250ReadGyroscope(float *gyroX, float *gyroY, float *gyroZ){
	if(ReadRegisters(GYRO_XOUT_H, 6, _buffer) < 0)
		return -1;
	_gxcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	_gycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	_gzcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];

	_gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
	_gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
	_gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;

	if(gyroX != 0)
		*gyroX = _gx;
	if(gyroY != 0)
		*gyroY = _gy;
	if(gyroZ != 0)
		*gyroZ = _gz;

	return 1;
}

//---------------------------------------------------------------------------------------------------
int MPU9250ReadAccelerometer(float *acceX, float *acceY, float *acceZ){
	// grab the data from the MPU9250
	if(ReadRegisters(ACCEL_XOUT_H, 6, _buffer) < 0)
		return -1;
	// combine into 16 bit values
	_axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	_aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	_azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];

	_ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
	_ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
	_az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;

	if(acceX != 0)
		*acceX = _ax;
	if(acceY != 0)
		*acceY = _ay;
	if(acceZ != 0)
		*acceZ = _az;

	return 1;
}

//---------------------------------------------------------------------------------------------------
int MPU9250ReadMagnetometer(float *magX, float *magY, float *magZ){
	//get magnetometro data
	ReadAK8963Registers(AK8963_XOUT_L, 7, _buffer);
	_hxcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	_hycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	_hzcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];

	_hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
	_hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
	_hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;

	if(magX != 0)
		*magX = _hx;
	if(magY != 0)
		*magY = _hy;
	if(magZ != 0)
		*magZ = _hz;

	return 1;
}

//---------------------------------------------------------------------------------------------------
int MPU9250ReadTemperature(float *temp){
	// grab the data from the MPU9250
	if(ReadRegisters(TEMP_OUT_H, 2, _buffer) < 0)
		return -1;

	_tcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	_t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;

	*temp = _t;

	return 1;
}

//---------------------------------------------------------------------------------------------------
int MPU9250ReadSensor(){
	//get magnetometro data
	ReadAK8963Registers(AK8963_XOUT_L, 7, _buffer);

	// grab the data from the MPU9250
	if(ReadRegisters(ACCEL_XOUT_H, 21, _buffer) < 0)
		return -1;
	// combine into 16 bit values
	_axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	_aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	_azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
	_tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
	_gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
	_gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	_gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
	_hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
	_hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
	_hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];
	// transform and convert to float values
	_ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
	_ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
	_az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
	_gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
	_gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
	_gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
	_hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
	_hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
	_hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
	_t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;

	return 1;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelX_mss(){
	return _ax;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelY_mss(){
	return _ay;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelZ_mss(){
	return _az;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetGyroX_rads(){
	return _gx;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetGyroY_rads(){
	return _gy;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetGyroZ_rads(){
	return _gz;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagX_uT(){
	return _hx;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagY_uT(){
	return _hy;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagZ_uT(){
	return _hz;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetTemperature_C(){
	return _t;
}
//---------------------------------------------------------------------------------------------------
int MPU9250CalibrateGyro(){
	// set the range, bandwidth, and srd
	if(MPU9250SetGyroRange(GYRO_RANGE_250DPS) < 0)
		return -1;
	if(MPU9250SetDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0)
		return -2;
	if(MPU9250SetSrd(19) < 0)
		return -3;

	// take samples and find bias
	_gxbD = 0;
	_gybD = 0;
	_gzbD = 0;
	for (uint8_t i=0; i < _numSamples; i++) {
		MPU9250ReadGyroscope(0, 0, 0);
		_gxbD += (MPU9250GetGyroX_rads() + _gxb)/((double)_numSamples);
		_gybD += (MPU9250GetGyroY_rads() + _gyb)/((double)_numSamples);
		_gzbD += (MPU9250GetGyroZ_rads() + _gzb)/((double)_numSamples);
		StopWatch_DelayMs(20);
	}
	_gxb = (float)_gxbD;
	_gyb = (float)_gybD;
	_gzb = (float)_gzbD;

	// set the range, bandwidth, and srd back to what they were
	if(MPU9250SetGyroRange(_gyroRange) < 0)
		return -4;
	if(MPU9250SetDlpfBandwidth(_bandWidth) < 0)
		return -5;
	if(MPU9250SetSrd(_srd) < 0)
		return -6;
	return 1;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetGyroBiasX_rads(){
	return _gxb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetGyroBiasY_rads(){
	return _gyb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetGyroBiasZ_rads(){
	return _gzb;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetGyroBiasX_rads(float bias){
	_gxb = bias;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetGyroBiasY_rads(float bias){
	_gyb = bias;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetGyroBiasZ_rads(float bias){
	_gzb = bias;
}
//---------------------------------------------------------------------------------------------------
int MPU9250CalibrateAccel(){
	// set the range, bandwidth, and srd
	if(MPU9250SetAccelRange(ACCEL_RANGE_2G) < 0)
		return -1;
	if(MPU9250SetDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0)
		return -2;
	if(MPU9250SetSrd(19) < 0)
		return -3;

	// take samples and find min / max
	_axbD = 0;
	_aybD = 0;
	_azbD = 0;
	for (uint8_t i=0; i < _numSamples; i++) {
		MPU9250ReadAccelerometer(0, 0, 0);
		_axbD += (MPU9250GetAccelX_mss()/_axs + _axb)/((double)_numSamples);
		_aybD += (MPU9250GetAccelY_mss()/_ays + _ayb)/((double)_numSamples);
		_azbD += (MPU9250GetAccelZ_mss()/_azs + _azb)/((double)_numSamples);
		StopWatch_DelayMs(20);
	}
	if(_axbD > 9.0)
		_axmax = (float)_axbD;
	if(_aybD > 9.0)
		_aymax = (float)_aybD;
	if(_azbD > 9.0)
		_azmax = (float)_azbD;
	if(_axbD < -9.0)
		_axmin = (float)_axbD;
	if(_aybD < -9.0)
		_aymin = (float)_aybD;
	if(_azbD < -9.0)
		_azmin = (float)_azbD;

	// find bias and scale factor
	if((abs(_axmin) > 9.0) && (abs(_axmax) > 9.0)) {
		_axb = (_axmin + _axmax) / 2.0;
		_axs = G/((abs(_axmin) + abs(_axmax)) / 2.0);
	}
	if((abs(_aymin) > 9.0) && (abs(_aymax) > 9.0)) {
		_ayb = (_axmin + _axmax) / 2.0;
		_ays = G/((abs(_aymin) + abs(_aymax)) / 2.0);
	}
	if((abs(_azmin) > 9.0) && (abs(_azmax) > 9.0)) {
		_azb = (_azmin + _azmax) / 2.0;
		_azs = G/((abs(_azmin) + abs(_azmax)) / 2.0);
	}

	// set the range, bandwidth, and srd back to what they were
	if(MPU9250SetAccelRange(_accelRange) < 0)
		return -4;
	if(MPU9250SetDlpfBandwidth(_bandWidth) < 0)
		return -5;
	if(MPU9250SetSrd(_srd) < 0)
		return -6;
	return 1;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelBiasX_mss(){
	return _axb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelScaleFactorX(){
	return _axs;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelBiasY_mss(){
	return _ayb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelScaleFactorY(){
	return _ays;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelBiasZ_mss(){
	return _azb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetAccelScaleFactorZ(){
	return _azs;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetAccelCalX(float bias, float scaleFactor){
	_axb = bias;
	_axs = scaleFactor;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetAccelCalY(float bias, float scaleFactor){
	_ayb = bias;
	_ays = scaleFactor;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetAccelCalZ(float bias, float scaleFactor){
	_azb = bias;
	_azs = scaleFactor;
}
//---------------------------------------------------------------------------------------------------
int MPU9250CalibrateMag(){
	// set the srd
	if(MPU9250SetSrd(19) < 0)
		return -1;

	// get a starting set of data
    ReadAK8963Registers(AK8963_XOUT_L, 7, _buffer);
	MPU9250ReadMagnetometer(0, 0, 0);
	_hxmax = MPU9250GetMagX_uT();
	_hxmin = MPU9250GetMagX_uT();
	_hymax = MPU9250GetMagY_uT();
	_hymin = MPU9250GetMagY_uT();
	_hzmax = MPU9250GetMagZ_uT();
	_hzmin = MPU9250GetMagZ_uT();

	// collect data to find max / min in each channel
	_counter = 0;
	while(_counter < _maxCounts) {
		_delta = 0.0f;
		_framedelta = 0.0f;

		ReadAK8963Registers(AK8963_XOUT_L, 7, _buffer);
		MPU9250ReadMagnetometer(0, 0, 0);

		_hxfilt = (_hxfilt*((float)_coeff-1)+(MPU9250GetMagX_uT()/_hxs+_hxb))/((float)_coeff);
		_hyfilt = (_hyfilt*((float)_coeff-1)+(MPU9250GetMagY_uT()/_hys+_hyb))/((float)_coeff);
		_hzfilt = (_hzfilt*((float)_coeff-1)+(MPU9250GetMagZ_uT()/_hzs+_hzb))/((float)_coeff);
		if(_hxfilt > _hxmax) {
			_delta = _hxfilt - _hxmax;
			_hxmax = _hxfilt;
		}
		if(_delta > _framedelta)
			_framedelta = _delta;
		if(_hyfilt > _hymax){
			_delta = _hyfilt - _hymax;
			_hymax = _hyfilt;
		}
		if(_delta > _framedelta)
			_framedelta = _delta;
		if(_hzfilt > _hzmax) {
			_delta = _hzfilt - _hzmax;
			_hzmax = _hzfilt;
		}
		if(_delta > _framedelta)
			_framedelta = _delta;
		if(_hxfilt < _hxmin) {
			_delta = abs(_hxfilt - _hxmin);
			_hxmin = _hxfilt;
		}
		if(_delta > _framedelta)
			_framedelta = _delta;
		if(_hyfilt < _hymin) {
			_delta = abs(_hyfilt - _hymin);
			_hymin = _hyfilt;
		}
		if(_delta > _framedelta)
			_framedelta = _delta;
		if(_hzfilt < _hzmin) {
			_delta = abs(_hzfilt - _hzmin);
			_hzmin = _hzfilt;
		}
		if(_delta > _framedelta)
			_framedelta = _delta;
		if(_framedelta > _deltaThresh)
			_counter = 0;
		else
			_counter++;
		StopWatch_DelayMs(20);
	}

	// find the magnetometer bias
	_hxb = (_hxmax + _hxmin) / 2.0;
	_hyb = (_hymax + _hymin) / 2.0;
	_hzb = (_hzmax + _hzmin) / 2.0;

	// find the magnetometer scale factor
	_hxs = (_hxmax - _hxmin) / 2.0;
	_hys = (_hymax - _hymin) / 2.0;
	_hzs = (_hzmax - _hzmin) / 2.0;
	_avgs = (_hxs + _hys + _hzs) / 3.0;
	_hxs = _avgs/_hxs;
	_hys = _avgs/_hys;
	_hzs = _avgs/_hzs;

	// set the srd back to what it was
	if(MPU9250SetSrd(_srd) < 0)
		return -2;
	return 1;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagBiasX_uT(){
	return _hxb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagScaleFactorX(){
	return _hxs;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagBiasY_uT(){
	return _hyb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagScaleFactorY(){
	return _hys;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagBiasZ_uT(){
	return _hzb;
}
//---------------------------------------------------------------------------------------------------
float MPU9250GetMagScaleFactorZ(){
	return _hzs;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetMagCalX(float bias, float scaleFactor){
	_hxb = bias;
	_hxs = scaleFactor;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetMagCalY(float bias, float scaleFactor){
	_hyb = bias;
	_hys = scaleFactor;
}
//---------------------------------------------------------------------------------------------------
void MPU9250SetMagCalZ(float bias, float scaleFactor){
	_hzb = bias;
	_hzs = scaleFactor;
}
//---------------------------------------------------------------------------------------------------
int MPU9250EnableFifo(uint8_t accel, uint8_t gyro, uint8_t mag, uint8_t temp){
	if(WriteRegister(USER_CTRL, (0x40 | 0x20)) < 0)
		return -1;
	if(WriteRegister(FIFO_EN, (accel*0x08)|(gyro*0x70)|(mag*0x01)|(temp*0x80)) < 0)
		return -2;
	_enFifoAccel = accel;
	_enFifoGyro = gyro;
	_enFifoMag = mag;
	_enFifoTemp = temp;
	_fifoFrameSize = accel*6 + gyro*6 + mag*7 + temp*2;
	return 1;
}
//---------------------------------------------------------------------------------------------------
int MPU9250ReadFifo(){
	ReadRegisters(0x72, 2, _buffer);
	_fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
	// read and parse the buffer
	for(uint8_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
		// grab the data from the MPU9250
		if(ReadRegisters(0x74, _fifoFrameSize, _buffer) < 0)
			return -1;
		if(_enFifoAccel) {
			// combine into 16 bit values
			_axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
			_aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
			_azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
			// transform and convert to float values
			_axFifo[i] = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale)-_axb)*_axs;
			_ayFifo[i] = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale)-_ayb)*_ays;
			_azFifo[i] = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale)-_azb)*_azs;
			_aSize = _fifoSize/_fifoFrameSize;
		}
		if(_enFifoTemp) {
			// combine into 16 bit values
			_tcounts = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
			// transform and convert to float values
			_tFifo[i] = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
			_tSize = _fifoSize/_fifoFrameSize;
		}
		if(_enFifoGyro) {
			// combine into 16 bit values
			_gxcounts = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
			_gycounts = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
			_gzcounts = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
			// transform and convert to float values
			_gxFifo[i] = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
			_gyFifo[i] = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
			_gzFifo[i] = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
			_gSize = _fifoSize/_fifoFrameSize;
		}
		if(_enFifoMag) {
			// combine into 16 bit values
			_hxcounts = (((int16_t)_buffer[1 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[0 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
			_hycounts = (((int16_t)_buffer[3 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[2 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
			_hzcounts = (((int16_t)_buffer[5 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[4 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
			// transform and convert to float values
			_hxFifo[i] = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
			_hyFifo[i] = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
			_hzFifo[i] = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
			_hSize = _fifoSize/_fifoFrameSize;
		}
	}
	return 1;
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoAccelX_mss(uint8_t *size, float *data){
	*size = _aSize;
	for(int i=0; i<_aSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_axFifo)[i];
	//memcpy(data, _axFifo, _aSize*sizeof(float));
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoAccelY_mss(uint8_t *size, float *data){
	*size = _aSize;
	for(int i=0; i<_aSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_ayFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoAccelZ_mss(uint8_t *size, float *data){
	*size = _aSize;
	for(int i=0; i<_aSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_azFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoGyroX_rads(uint8_t *size, float *data){
	*size = _gSize;
	for(int i=0; i<_gSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_gxFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoGyroY_rads(uint8_t *size, float *data){
	*size = _gSize;
	for(int i=0; i<_gSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_gyFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoGyroZ_rads(uint8_t *size, float *data){
	*size = _gSize;
	for(int i=0; i<_gSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_gzFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoMagX_uT(uint8_t *size, float *data){
	*size = _hSize;
	for(int i=0; i<_hSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_hxFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoMagY_uT(uint8_t *size, float *data){
	*size = _hSize;
	for(int i=0; i<_hSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_hyFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoMagZ_uT(uint8_t *size, float *data){
	*size = _hSize;
	for(int i=0; i<_hSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_hzFifo)[i];
}
//---------------------------------------------------------------------------------------------------
void MPU9250GetFifoTemperature_C(uint8_t *size, float *data){
	*size = _tSize;
	for(int i=0; i<_tSize*sizeof(float); i++)
		((uint8_t *)data)[i] = ((uint8_t *)_tFifo)[i];
}
//---------------------------------------------------------------------------------------------------




