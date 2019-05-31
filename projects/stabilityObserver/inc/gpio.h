/** @file: gpio.h
 * @brief  EDU-CIAA NXP GPIO driver
 *
 * This driver provide functions to configure and handle the EDU-CIAA NXP GPIO ports
 *
 * @note
 *
 * @author Albano Peñalva
 *
 * @section changelog
 *
 * |   Date	    | Description                                    	|
 * |:----------:|:--------------------------------------------------|
 * | 21/11/2018 | Document creation		                         	|
 * | 01/12/2018 | Enumeration modified for compatibility with SAPI	|
 *
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
/*****************************************************************************
 * Public macros/types/enumerations/variables definitions
 ****************************************************************************/

/**
 * @brief GPIO ports
 */
typedef enum
{
	GPIO_0 = 0,
	GPIO_1,
	GPIO_2,
	GPIO_3,
	GPIO_4,
	GPIO_5,
	GPIO_6,
	GPIO_7,
	GPIO_8
} gpioPin_t;

/**
 * @brief GPIO directions
 */
typedef enum
{
	INPUT,
	OUTPUT
} gpioDir_t;

/**
 * @brief GPIO resistors
 */
typedef enum
{
	NONE_RES,			/*!< Disable pull-down and pull-up resistor at pin */
	PULLUP,				/*!< Enable pull-up resistor at pin */
	PULLDOWN,			/*!< Enable pull-down resistor at pin */
	PULLUP_PULLDOWN		/*!< Enable pull-up and pull-down resistor at pin */
} gpioRes_t;

/**
 * @brief GPIO configuration structure
 */
typedef struct
{
	gpioPin_t pin;				/*!< GPIO pin number */
	gpioDir_t dir;				/*!< GPIO direction */
	gpioRes_t res;				/*!< Pull-up and pull-down resistors */
} gpioConf_t;

/*****************************************************************************
 * Public functions definitions
 ****************************************************************************/

/**
 * @brief		Initialize one GPIO pin with the corresponding configuration
 * @param[in]	gpio Structure with the pin configuration
 * @return		1 when success, 0 when fails
 * @note
 */
uint8_t GPIOInit(gpioConf_t gpio);

/**
 * @brief		Read the corresponding GPIO pin input value
 * @param[in]	pin GPIO pin to read
 * @return		GPIO pin input value
 * @note
 */
uint8_t GPIORead(gpioPin_t pin);

/**
 * @brief		Write an 1 at the corresponding GPIO pin
 * @param[in]	pin GPIO pin to write
 * @return		None
 * @note
 */
void GPIOSetHigh(gpioPin_t pin);

/**
 * @brief		Write an 0 at the corresponding GPIO pin
 * @param[in]	pin GPIO pin to write
 * @return		None
 * @note
 */
void GPIOSetLow(gpioPin_t pin);

/**
 * @brief		Toggle the corresponding GPIO pin output value
 * @param[in]	pin GPIO pin to write
 * @return		None
 * @note
 */
void GPIOToggle(gpioPin_t pin);

#endif /* GPIO_H_ */
