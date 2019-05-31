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

#include "gpio.h"

#include "chip.h"

/*****************************************************************************
 * Private macros/types/enumerations/variables definitions
 ****************************************************************************/

#define SUCCESS	1			/* */
#define ERROR 	0			/* */

/**
 * @brief Internal GPIO mapping structure
 */
typedef struct
{
	uint8_t hwPort;
	uint8_t hwPin;
	uint8_t gpioPort;
	uint8_t gpioPin;
	uint16_t mode;
} digitalIO_t;

/**
 * @brief Structure to map EDU-CIAA NXP GPIOs with the corresponding LPC4337 GPIOs
 */
const digitalIO_t gpio_map[] =
{
	{0x06, 0x01, 0x03, 0x00, SCU_MODE_FUNC0}, 	/* GPIO0 -> P6.1  GPIO3[0]  */
	{0x06, 0x04, 0x03, 0x03, SCU_MODE_FUNC0}, 	/* GPIO1 -> P6.4  GPIO3[3]  */
	{0x06, 0x05, 0x03, 0x04, SCU_MODE_FUNC0}, 	/* GPIO2 -> P6.5  GPIO3[4]  */
	{0x06, 0x07, 0x05, 0x0F, SCU_MODE_FUNC4}, 	/* GPIO3 -> P6.7  GPIO5[15] */
	{0x06, 0x08, 0x05, 0x10, SCU_MODE_FUNC4}, 	/* GPIO4 -> P6.8  GPIO5[16] */
	{0x06, 0x09, 0x03, 0x05, SCU_MODE_FUNC0}, 	/* GPIO5 -> P6.9  GPIO3[5]  */
	{0x06, 0x0A, 0x03, 0x06, SCU_MODE_FUNC0}, 	/* GPIO6 -> P6.10 GPIO3[6]  */
	{0x06, 0x0B, 0x03, 0x07, SCU_MODE_FUNC0}, 	/* GPIO7 -> P6.11 GPIO3[7]  */
	{0x06, 0x0C, 0x02, 0x08, SCU_MODE_FUNC0}, 	/* GPIO8 -> P6.12 GPIO2[8]  */
};

/*****************************************************************************
 * Public types/enumerations/variables declarations
 ****************************************************************************/

/*****************************************************************************
 * Private functions definitions
 ****************************************************************************/

/*****************************************************************************
 * Private functions declarations
 ****************************************************************************/

/*****************************************************************************
 * Public functions declarations
 ****************************************************************************/

uint8_t GPIOInit(gpioConf_t gpio)
{
	uint16_t res_mode;

	switch (gpio.res)
	{
	case NONE_RES:
		res_mode = SCU_MODE_INACT;
		break;

	case PULLUP:
		res_mode = SCU_MODE_PULLUP;
		break;

	case PULLDOWN:
		res_mode = SCU_MODE_PULLDOWN;
		break;

	case PULLUP_PULLDOWN:
		res_mode = SCU_MODE_REPEATER;
		break;
	}

	if (gpio.dir == INPUT)
	{
		Chip_SCU_PinMuxSet(gpio_map[gpio.pin].hwPort, gpio_map[gpio.pin].hwPin,
				gpio_map[gpio.pin].mode | SCU_MODE_INBUFF_EN | res_mode);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, gpio_map[gpio.pin].gpioPort,
				gpio_map[gpio.pin].gpioPin);
	}
	if (gpio.dir == OUTPUT)
	{
		Chip_SCU_PinMuxSet(gpio_map[gpio.pin].hwPort, gpio_map[gpio.pin].hwPin,
				gpio_map[gpio.pin].mode | res_mode);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, gpio_map[gpio.pin].gpioPort,
				gpio_map[gpio.pin].gpioPin);
		/* All outputs initialized at low state */
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio_map[gpio.pin].gpioPort,
				gpio_map[gpio.pin].gpioPin, FALSE);
	}

	return SUCCESS;
}

uint8_t GPIORead(gpioPin_t pin)
{
	return Chip_GPIO_GetPinState(LPC_GPIO_PORT, gpio_map[pin].gpioPort,
			gpio_map[pin].gpioPin);
}

void GPIOSetHigh(gpioPin_t pin)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio_map[pin].gpioPort,
			gpio_map[pin].gpioPin, TRUE);
}

void GPIOSetLow(gpioPin_t pin)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio_map[pin].gpioPort,
			gpio_map[pin].gpioPin, FALSE);
}

void GPIOToggle(gpioPin_t pin)
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, gpio_map[pin].gpioPort,
			gpio_map[pin].gpioPin);
}


