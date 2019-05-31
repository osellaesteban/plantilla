/**
 ** @defgroup Trabajos practicos de ESE
 ** @brief TP Final: Stability risk evaluation and registering based on the MPU9250 meassurements.
 **
 ** @{
 **
 **  # Stability risk evaluation and registering based on the MPU9250 meassurements. #
 **
 **  \brief: This program pretends to be a risk observer for ESC, using inertial meassurements.
 **
 ** | RV | YYYY.MM.DD | Autor          | Descripción de los cambios              |
 ** |----|------------|----------------|-----------------------------------------|
 ** |  1 | 2019.05.04 | eosella        | Version inicial del archivo             |
 ** |  2 | 2019.05.04 |dvertizdelvalle | 
 **
 **
 **  \ToDo: almost everything.
 **
 **  \file: stability
 **  \date: 14 may 2019
 **  \authors: Esteban Osella, Diana Vertiz del Valle
 **  \email: eosella@ingenieria.uner.edu.ar, dvertizdelvalle@ingenieria.uner.edu.ar
 **/

/*==================[inclusions]=============================================*/
#include "stability.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Display_ili9341.h"
#include "fonts.h"
#include "FreeRTOSConfig.h"
#include "Hardware_driver.h"
#include "menu.h"
#include "MPU_9250.h"
#include "spi_ili9341.h"
#include "SD_driver.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "switch.h"
#include "soc.h"
#include "queue.h"
#include "led.h"
#include "semphr.h"
#include "uart.h"
/*==================[macros and definitions]=================================*/


#define MAX_FILTER_SIZE		20
#define MPU_QUEUE_SIZE		200
#define VAR_COUNT			6
#define HORIZONT 			20

#define FILE_SIZE			300
#define SD_DROP_SIZE		30

#define DISP_SPI_PORT		1
#define DISP_GPIO_CS		BOARD_GPIO_1
#define DISP_GPIO_DC		BOARD_GPIO_5
#define DISP_GPIO_RST 		BOARD_GPIO_2
#define MPU_DRDY_GPIO		GPIO1


#define RISK_EVALUATION_REQUIRED_EVENT	( 1 << 0)
#define SD_WRITE_EVENT        			( 1 << 1)


/** @briev Evento para inidicar que la transmisción esta completa */

#define EVENTO_INICIAR        ( 1 << 2)
#define EVENTO_TRANSMITIDO    ( 1 << 3)
#define EVENTO_RECIBIDO       ( 1 << 4)
#define EVENT_MENU_START      ( 1 << 5)
#define EVENT_MENU_STOP       ( 1 << 6)

#define	QUEUE_SIZE				20

#define SATURATION_RED		31
#define SATURATION_GREEN	63
#define SATURATION_BLUE		31

/*==================[internal data declaration]==============================*/


/**
 * @brief Estructura que representa un elemento de un menú
 */


/*static const char *SelectMenuVariable = "\r\nPress number 1-7 to choose options to configure:\r\n"
						   " 1: Acceleration \r\n"
						   " 2: Angular Velocity \r\n"
						   " 3: Angles \r\n"
						   " 4: Magnetic field \r\n"
						   " 5: Filter parameters \r\n"
						   " 6: Timing parameters \r\n"
						   " 7: Saving parameters \r\n";*/



/* === Declaraciones de tipos de datos internos ============================ */

/** @brief Estructura de datos para transmisión
 **
 ** Estructura que contiene los datos necesarios para que la interrupción
 ** pueda continuar el envio de datos usando la función @ref EnviarCaracter
 ** de la transmisisión iniciada por la función@ref EnviarTexto.
 */
typedef struct {
   struct {
      const char * datos;
      uint8_t cantidad;
      uint8_t enviados;
   } tx;
   struct {
      char * datos;
      uint8_t cantidad;
      uint8_t recibidos;
   } rx;
} cola_t;

typedef enum {AX_NULL, AX_AX, AX_AY, AX_AZ,AX_GX,AX_GY,AX_GZ} axis_t;

typedef struct{
	float _ax,_ay,_az;
	float _gx,_gy,_gz;
	float _hx,_hy,_hz;
	float temp;
	uint8_t ev_risk;
	//rtc_t timestamp;
} MPU_Reading_t;




/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */


/*==================[internal data definition]===============================*/


enum
{
	flag_variable = 1,
	flag_Acc_threshold,
	flag_Ang_Acc_threshold,
	flag_filter,
	flag_filter_sub,
	flag_timing,
	//flag_saving
};

static const char *WelcomeMenu = "\r\nHello \r\n"
							"STABILITY OBSERVER \r\n"
							"Press TEC2 to initiate configuration or \'x\' to stop it\r\n";

char cadena[16]; /*cadena para recibir los comandos o valores*/
char number[10]; /*cadena para recibir valor flotante: 3 cifras punto y dos decimales*/
char * uart_buffer;

menuItem * menu;
//threshold_t Umbrales;
uint8_t menu_size;

/* === Definiciones de variables internas ================================== */

/** @brief Información para el envio de datos por la uart */
cola_t cola;
/** @brief Descriptor de la tarea de envio para reactivarla */
TaskHandle_t enviar;
TaskHandle_t Display;
QueueHandle_t Queue_RX_TX;

SemaphoreHandle_t SpiMutex;
/** @brief Descriptor del grupo de eventos */
//EventGroupHandle_t eventosUART;

typedef struct {
	uint16_t imu_sampling;
	uint8_t filter_timing;
	uint16_t file_saving_time;				/*in miliseconds*/
	uint8_t saving_time; 					/*in hours*/
	uint8_t display_refresh;
} timing_parameters_t ;

typedef struct{
	uint8_t size;
	float parameters[MAX_FILTER_SIZE];
}filter_params_t;


struct {
	 filter_params_t MPU_accelerometer;
	 filter_params_t MPU_giroscope;
	 filter_params_t MPU_magnetometer;
	 filter_params_t MPU_temperature;
} filter_params;

typedef struct {
	uint16_t L_AX_IN;	// Lower axis intersection
	uint16_t S_LB;		// Saturation lower bound
	uint16_t S_UB;		// Saturation upper bound
	uint16_t U_AX_IN; 	// Upper axis intersection
} membershipdef_t;

MPU_Reading_t last_MPU_reading;

/** @brief Event group description */
EventGroupHandle_t eventos;

/* Queue to hold the fresh read*/
xQueueHandle xqh_MPU_Unfiltered; 	// buffer to hold the fresh readings
TickType_t TickType_W2F_UnfileredBuffer;


/* Queue to hold data to be stored */
//xQueueHandle xqh_MPU_Writing_buffer;
//TickType_t TickType_W2F_WriteBuffer;

/* Queue to hold data to be shown */
//xQueueHandle xqh_MPU_Display_buffer ;
//TickType_t TickType_W2F_DisplayBuffer;

/* buffer for the filtering process */
MPU_Reading_t MPU_Filtering_buffer[MAX_FILTER_SIZE];

volatile uint32_t samplecount;

timing_parameters_t timing_parameters;

MPU_Reading_t threshold;

uint8_t horizont;

uint16_t evaluation;

uint16_t rgb_color;

uint16_t scale;

axis_t variables[VAR_COUNT];

membershipdef_t memberFunctions[3];

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void MPU_Initialize_Parameters()
{

	threshold._ax = 1;
	threshold._ay = 0;
	threshold._az = 0;
	threshold._gx = 0;
	threshold._gy = 0;
	threshold._gz = 1;
	threshold._hx = 0;
	threshold._hy = 0;
	threshold._hz = 0;

	horizont = HORIZONT;
	// variables to be evaluated

	variables[0] = AX_AX;
	variables[1] = AX_GZ;
	variables[2] = AX_NULL;
	variables[3] = AX_NULL;
	variables[4] = AX_NULL;
	variables[6] = AX_NULL;

	// member function initialization
	memberFunctions[0].L_AX_IN = 0;
	memberFunctions[0].S_LB = 0;
	memberFunctions[0].S_UB = (uint16_t )3 * HORIZONT*0.3;
	memberFunctions[0].U_AX_IN = (uint16_t )3 * HORIZONT*0.5;

	memberFunctions[1].L_AX_IN = memberFunctions[0].S_UB;
	memberFunctions[1].S_LB = memberFunctions[0].U_AX_IN;
	memberFunctions[1].S_UB = memberFunctions[1].S_LB;
	memberFunctions[1].U_AX_IN = (uint16_t )3 * HORIZONT*0.8;

	memberFunctions[2].L_AX_IN = memberFunctions[1].S_UB;
	memberFunctions[2].S_LB = memberFunctions[1].U_AX_IN;
	memberFunctions[2].S_UB = memberFunctions[2].S_LB;
	memberFunctions[2].U_AX_IN = 65535;

	scale = 2;//000;

	timing_parameters.display_refresh = 20;
	timing_parameters.file_saving_time = 100;
	timing_parameters.filter_timing = 200;
	timing_parameters.imu_sampling = 10;

	filter_params.MPU_accelerometer.size = 3;
	filter_params.MPU_accelerometer.parameters[0] = 1;
	filter_params.MPU_accelerometer.parameters[1] = 0;
	filter_params.MPU_accelerometer.parameters[2] = 0;
	filter_params.MPU_accelerometer.parameters[3] = 0;
	filter_params.MPU_accelerometer.parameters[4] = 0;

	filter_params.MPU_giroscope.size = 2;
	filter_params.MPU_giroscope.parameters[0] = 1;
	filter_params.MPU_giroscope.parameters[1] = 0;
	filter_params.MPU_giroscope.parameters[2] = 0;
	filter_params.MPU_giroscope.parameters[3] = 0;
	filter_params.MPU_giroscope.parameters[4] = 0;

	filter_params.MPU_magnetometer.size = 0;
	filter_params.MPU_magnetometer.parameters[0] = 1;
	filter_params.MPU_magnetometer.parameters[1] = 0;
	filter_params.MPU_magnetometer.parameters[2] = 0;
	filter_params.MPU_magnetometer.parameters[3] = 0;
	filter_params.MPU_magnetometer.parameters[4] = 0;

	filter_params.MPU_temperature.size = 0;
	filter_params.MPU_temperature.parameters[0] = 1;
	filter_params.MPU_temperature.parameters[1] = 0;
	filter_params.MPU_temperature.parameters[2] = 0;
	filter_params.MPU_temperature.parameters[3] = 0;
	filter_params.MPU_temperature.parameters[4] = 0;
}

//############################## UART Menu Handling ##########################################################

void Menu_actions(char * cadena){

	static uint8_t op_main;
	static uint8_t op_filter;
	static uint8_t sub_menu_size;
	char * string;
	char *token1, *token2, *token3, *token4, *token5;
	float aux1, aux2, aux3 ;


	switch(menu->flag){

		case flag_variable:
			op_main = ((uint8_t)cadena[0] - '0'); /*las opciones son 1, 2, 3, 4 o 5*/
			op_main--; 			/*para llevarlo a 0,1,2,3 o 4*/

			if((op_main < menu_size) && (op_main >= 0)){
				menu = menu[op_main].doAction(); /*la dirección de memoria cambia a la del submenú
											 de la opción seleccionada*/
				xEventGroupSetBits(eventos, EVENTO_INICIAR);
			}
			else{
				/*opción incorrecta*/
			}
			break;
		case flag_Acc_threshold:
				/*recibí un valor y vuelvo al menu principal*/
				token1 = strtok(cadena, " ");
				token2 = strtok(NULL, " ");
				token3 = strtok(NULL, " ");
				threshold._ax = strtof(token1,&string);
				threshold._ay = strtof(token2,&string);
				threshold._az = strtof(token3,&string);
				menu = menu[0].doAction();
				xEventGroupSetBits(eventos, EVENTO_INICIAR);
			break;

		case flag_Ang_Acc_threshold:
				/*recibí un valor y vuelvo al menu principal*/
				token1 = strtok(cadena, " ");
				token2 = strtok(NULL, " ");
				token3 = strtok(NULL, " ");
				threshold._gx = strtof(token1,&string);
				threshold._gy = strtof(token2,&string);
				threshold._gz = strtof(token3,&string);
				//sscanf(cadena, "%f%f%f", &threshold._gx, &threshold._gy, &threshold._gz);
				menu = menu[0].doAction();
				xEventGroupSetBits(eventos, EVENTO_INICIAR);
			break;

		case flag_filter:
				/**/
			op_filter = ((uint8_t)cadena[0] - '0'); /*las opciones son 1, 2, 3, 4 o 5*/
			op_filter--; 			/*para llevarlo a 0,1,2,3 o 4*/
			menu = menu[op_filter].doAction();
			sub_menu_size = menu_size;
			xEventGroupSetBits(eventos, EVENTO_INICIAR);
			break;

		case flag_filter_sub:
			if((op_filter < sub_menu_size) && (op_filter >= 0)){
				token1 = strtok(cadena, " ");
				token2 = strtok(NULL, " ");
				token3 = strtok(NULL, " ");
				token4 = strtok(NULL, " ");
				token5 = strtok(NULL, " ");

				if(op_filter==0){
					filter_params.MPU_accelerometer.parameters[0] = strtof(token1,&string);
					filter_params.MPU_accelerometer.parameters[1] = strtof(token2,&string);
					filter_params.MPU_accelerometer.parameters[2] = strtof(token3,&string);
					filter_params.MPU_accelerometer.parameters[3] = strtof(token4,&string);
					filter_params.MPU_accelerometer.parameters[4] = strtof(token5,&string);
				} else if(op_filter==1){
					filter_params.MPU_giroscope.parameters[0] = strtof(token1,&string);
					filter_params.MPU_giroscope.parameters[1] = strtof(token2,&string);
					filter_params.MPU_giroscope.parameters[2] = strtof(token3,&string);
					filter_params.MPU_giroscope.parameters[3] = strtof(token4,&string);
					filter_params.MPU_giroscope.parameters[4] = strtof(token5,&string);
				} else{
					filter_params.MPU_magnetometer.parameters[0] = strtof(token1,&string);
					filter_params.MPU_magnetometer.parameters[1] = strtof(token2,&string);
					filter_params.MPU_magnetometer.parameters[2] = strtof(token3,&string);
					filter_params.MPU_magnetometer.parameters[3] = strtof(token4,&string);
					filter_params.MPU_magnetometer.parameters[4] = strtof(token5,&string);
				}
			}
			else{
					/*opción incorrecta*/
			}
				menu = menu[0].doAction();
				xEventGroupSetBits(eventos, EVENTO_INICIAR);
			break;

		case flag_timing:
				timing_parameters.imu_sampling   = (uint16_t)atoi(cadena);
				menu = menu[0].doAction();
				xEventGroupSetBits(eventos, EVENTO_INICIAR);
			break;

	}


}

bool Sending_text(const char * cadena) {
   bool pendiente = FALSE;

   cola.tx.datos = cadena;
   cola.tx.cantidad = strlen(cadena);
   cola.tx.enviados = 0;

   if (cola.tx.cantidad) {
      Chip_UART_SendByte(USB_UART, cola.tx.datos[cola.tx.enviados]);
      cola.tx.enviados++;

      if (cola.tx.enviados < cola.tx.cantidad) {
         Chip_UART_IntEnable(USB_UART, UART_IER_THREINT);
         pendiente = TRUE;
      }
   }
   return (pendiente);
}

bool Receive_character(void) {
   uint8_t eventos;
   uint8_t habilitados;
   char caracter;
   bool completo = FALSE;

   eventos = Chip_UART_ReadLineStatus(USB_UART);
   habilitados = Chip_UART_GetIntsEnabled(USB_UART);

   if ((eventos & UART_LSR_RDR) && (habilitados & UART_LSR_RDR)) {
      caracter = Chip_UART_ReadByte(USB_UART);
      if ((caracter != 13) && (caracter != 10)) {
    	 Chip_UART_SendByte(USB_UART, caracter );
         cola.rx.datos[cola.rx.recibidos] = caracter;
         cola.rx.recibidos++;
         completo = (cola.rx.recibidos == cola.rx.cantidad);
      } else {
    	 cola.rx.datos[cola.rx.recibidos] = 0;
         cola.rx.recibidos++;
      	 Chip_UART_SendByte(USB_UART, "\r\n" );

         completo = TRUE;
      }

      if (completo) {
         Chip_UART_IntDisable(USB_UART, UART_LSR_RDR);
      }
   }
   return (completo);
}

bool Send_character(void) {
   uint8_t eventos;
   uint8_t habilitados;
   bool completo = FALSE;

   eventos = Chip_UART_ReadLineStatus(USB_UART);
   habilitados = Chip_UART_GetIntsEnabled(USB_UART);

/*el Tx está vacío y está habilitada la interrupción de Tx*/
   if ((eventos & UART_LSR_THRE) && (habilitados & UART_IER_THREINT)) {

      Chip_UART_SendByte(USB_UART, cola.tx.datos[cola.tx.enviados]);
      cola.tx.enviados++;

      if (cola.tx.enviados == cola.tx.cantidad) {
         Chip_UART_IntDisable(USB_UART, UART_IER_THREINT);
         completo = TRUE;
      }
   }
   return (completo);

}

bool Receiving_text(char * cadena, uint8_t espacio) {
   bool pendiente = TRUE;

   cola.rx.datos = cadena;
   cola.rx.cantidad = espacio;
   cola.rx.recibidos = 0;

   Chip_UART_IntEnable(USB_UART, UART_LSR_RDR);

   return (pendiente);
}

// ------------------ UART interruptions ------------------------

void UART2_IRQHandler(void) {
   if (Send_character()) {

	   BaseType_t xHigherPriorityTaskWoken;

	   xHigherPriorityTaskWoken = pdFALSE;
	   xEventGroupSetBitsFromISR(eventos, EVENTO_TRANSMITIDO, &xHigherPriorityTaskWoken );
	   /* Was the message posted successfully? */

	   portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

   };

   if (Receive_character()){

	   BaseType_t xHigherPriorityTaskWoken2;

	   xHigherPriorityTaskWoken2 = pdFALSE;
	   xEventGroupSetBitsFromISR(eventos, EVENTO_RECIBIDO, &xHigherPriorityTaskWoken2 );
	   /* Was the message posted successfully? */

	   portYIELD_FROM_ISR( xHigherPriorityTaskWoken2 );


   };
}

//########################################### UART transmission tasks ############################################

/*task for sending menu text to the console*/
void Uart_sending_task(void * parametros) {
   uint8_t i=0;
   menu = getMainMenu();
   while(1) {

      while(xEventGroupWaitBits(eventos, EVENTO_INICIAR,
    		  FALSE, FALSE, portMAX_DELAY) == 0);

      uart_buffer = menu[i].txt;
      if(uart_buffer != NULL)
      {
          Led_On(YELLOW_LED);
          if (Sending_text(uart_buffer)) {

             while(xEventGroupWaitBits(eventos, EVENTO_TRANSMITIDO,
                TRUE, FALSE, portMAX_DELAY) == 0);

          }
          Led_Off(YELLOW_LED);
      i++;
      menu_size= i;
      }
      else
      {
    	  i=0;
    	  xEventGroupClearBits(eventos,EVENTO_INICIAR);
      }
   }
}


void Uart_receiving_task(void * parametros) {


	while(1) {

		if (Receiving_text(cadena, sizeof(cadena))) {
		 /*when the string completes*/
		 xEventGroupWaitBits(eventos, EVENTO_RECIBIDO, TRUE, FALSE, portMAX_DELAY);

		 Led_On(YELLOW_LED);

			 Menu_actions(cadena);


		 Led_Off(YELLOW_LED);
		}
	}
}

//################################ MPU tasks #################################################

void MPU_Reading_task(void * parameter)
{
	MPU_Reading_t dynamics;

	uint8_t readg,reada, readm, readt,reads = 0;
	portBASE_TYPE xStatus;
	while(1)
	{
		vTaskDelay(timing_parameters.imu_sampling / portTICK_RATE_MS);
		reads = MPU9250ReadSensor();
		if (reads == 1)
		{
			readg = MPU9250ReadGyroscope(&dynamics._gx,&dynamics._gy, &dynamics._gz);
			reada = MPU9250ReadAccelerometer(&dynamics._ax, &dynamics._ay, &dynamics._az);
			readm = MPU9250ReadMagnetometer(&dynamics._hx, &dynamics._hy,&dynamics._hz);
			readt = MPU9250ReadTemperature(&dynamics.temp);
			if(readg+reada+readm+readt == 4)
			{
				xStatus = xQueueSendToBack( xqh_MPU_Unfiltered, &dynamics, TickType_W2F_UnfileredBuffer );
				/*dynamics._ax = 0;
				xStatus = xQueueReceive( xqh_MPU_Unfiltered, &dynamics, TickType_W2F_UnfileredBuffer );*/
				last_MPU_reading = dynamics;
				if( xStatus != pdPASS )
				{
					//--- buffer writing error handling
				}
				else{
					Led_Toggle(GREEN_LED);
					samplecount++;
				}
			}
			else
			{
				// *-- reading error handling
			}

		}
	}
}



void MPU_Filter_task(void * parameter)
{
	MPU_Reading_t reading;
	uint8_t maxfs, k = 0;
	portBASE_TYPE xStatus;

	while (1)
	{
		vTaskDelay(timing_parameters.filter_timing / portTICK_RATE_MS);

		/**
		 * compute the size of the filter
		 * ToDo: put a semaphore to the filter_params
		 * */
		maxfs = filter_params.MPU_accelerometer.size > filter_params.MPU_giroscope.size ?  filter_params.MPU_accelerometer.size : filter_params.MPU_giroscope.size;
		maxfs = maxfs > filter_params.MPU_magnetometer.size ? maxfs : filter_params.MPU_magnetometer.size;
		maxfs = maxfs > filter_params.MPU_temperature.size ? maxfs : filter_params.MPU_temperature.size;

		while(uxQueueMessagesWaiting( xqh_MPU_Unfiltered ) != 0 )
		{
			xStatus = xQueueReceive( xqh_MPU_Unfiltered, &reading, TickType_W2F_UnfileredBuffer );
			if( xStatus == pdPASS )
			{
				for (k = 1; k < MAX_FILTER_SIZE; k++)
				{
					MPU_Filtering_buffer[k] = MPU_Filtering_buffer[k-1];
				}
				MPU_Filtering_buffer[0] =  reading;
				//memset(&reading,0,sizeof(MPU_Reading_t));

				if (samplecount>filter_params.MPU_accelerometer.size)
				{
					k = 0;
					reading._ax = (float )MPU_Filtering_buffer[k]._ax*filter_params.MPU_accelerometer.parameters[k];
					reading._ay = (float )MPU_Filtering_buffer[k]._ay*filter_params.MPU_accelerometer.parameters[k];
					reading._az = (float )MPU_Filtering_buffer[k]._az*filter_params.MPU_accelerometer.parameters[k];
					for(k = 1;k<filter_params.MPU_accelerometer.size;k++)
					{
						reading._ax += (float )MPU_Filtering_buffer[k]._ax*filter_params.MPU_accelerometer.parameters[k];
						reading._ay += (float )MPU_Filtering_buffer[k]._ay*filter_params.MPU_accelerometer.parameters[k];
						reading._az += (float )MPU_Filtering_buffer[k]._az*filter_params.MPU_accelerometer.parameters[k];
					}
				}
				if (samplecount>filter_params.MPU_giroscope.size)
				{
					k = 0;
					reading._gx = MPU_Filtering_buffer[k]._gx*filter_params.MPU_giroscope.parameters[k];
					reading._gy = MPU_Filtering_buffer[k]._gy*filter_params.MPU_giroscope.parameters[k];
					reading._gz = MPU_Filtering_buffer[k]._gz*filter_params.MPU_giroscope.parameters[k];
					for(k = 1;k<filter_params.MPU_giroscope.size;k++)
					{
						reading._gx += MPU_Filtering_buffer[k]._gx*filter_params.MPU_giroscope.parameters[k];
						reading._gy += MPU_Filtering_buffer[k]._gy*filter_params.MPU_giroscope.parameters[k];
						reading._gz += MPU_Filtering_buffer[k]._gz*filter_params.MPU_giroscope.parameters[k];
					}
				}
				if (samplecount>filter_params.MPU_magnetometer.size)
				{
					k = 0;
					reading._gx = MPU_Filtering_buffer[k]._gx*filter_params.MPU_magnetometer.parameters[k];
					reading._gy = MPU_Filtering_buffer[k]._gy*filter_params.MPU_magnetometer.parameters[k];
					reading._gz = MPU_Filtering_buffer[k]._gz*filter_params.MPU_magnetometer.parameters[k];
					for(k = 1;k<filter_params.MPU_magnetometer.size;k++)
					{
						reading._gx += MPU_Filtering_buffer[k]._gx*filter_params.MPU_magnetometer.parameters[k];
						reading._gy += MPU_Filtering_buffer[k]._gy*filter_params.MPU_magnetometer.parameters[k];
						reading._gz += MPU_Filtering_buffer[k]._gz*filter_params.MPU_magnetometer.parameters[k];
					}
				}
				if (samplecount>filter_params.MPU_temperature.size)
				{
					k = 0;
					reading.temp =MPU_Filtering_buffer[k].temp*filter_params.MPU_temperature.parameters[k];
					for(k = 1;k<filter_params.MPU_temperature.size;k++)
					{
						reading.temp+=MPU_Filtering_buffer[k].temp*filter_params.MPU_temperature.parameters[k];
					}
				}
				/*xStatus = xQueueSendToBack(xqh_MPU_Writing_buffer, &reading, TickType_W2F_UnfileredBuffer);
				if (xStatus  != pdPASS)
				{
					// ERROR HANDLING FOR NOT WRITING IN BUFFER
				}*/
				xEventGroupSetBits(eventos, RISK_EVALUATION_REQUIRED_EVENT);
				/*if( !(samplecount % SD_DROP_SIZE) )
					xEventGroupSetBits(eventos, SD_WRITE_EVENT);*/
				MPU_Filtering_buffer[0] = reading;
				//samplecount--;
			}
		}
	}
}

//############################### Risk evaluation task ##################################

void MPU_Risk_Evaluation_Task(void *parametros)
{
	EventBits_t uxBits;
	portBASE_TYPE xStatus;
	MPU_Reading_t reading;
	uint8_t k,l;
	uint16_t color,scaled_evaluation;
	uint8_t count, saturation;
	while (1)
	{
		uxBits = xEventGroupWaitBits(eventos,RISK_EVALUATION_REQUIRED_EVENT,TRUE, FALSE, portMAX_DELAY);
		if (((uxBits & RISK_EVALUATION_REQUIRED_EVENT) == RISK_EVALUATION_REQUIRED_EVENT ) && HORIZONT < samplecount)
		{
			evaluation = 0;
			for (k = 0; k < HORIZONT-1; k++)
			{
				for (l = 0; l< VAR_COUNT; l++)
				{
					switch(variables[l])
					{
						case AX_NULL:
							break;
						case AX_AX:
							if (threshold._ax*threshold._ax < MPU_Filtering_buffer[k]._ax*MPU_Filtering_buffer[k]._ax )
								evaluation++;
							if (MPU_Filtering_buffer[k]._ax*MPU_Filtering_buffer[k-1]._ax<0)
								evaluation++;
							break;
						case AX_AY:
							if (threshold._ay*threshold._ay < MPU_Filtering_buffer[k]._ay*MPU_Filtering_buffer[k]._ay )
								evaluation++;
							if (MPU_Filtering_buffer[k]._ay*MPU_Filtering_buffer[k-1]._ay<0)
								evaluation++;
							break;
						case AX_AZ:
							if (threshold._az*threshold._az < MPU_Filtering_buffer[k]._az*MPU_Filtering_buffer[k]._az )
								evaluation++;
							if (MPU_Filtering_buffer[k]._az*MPU_Filtering_buffer[k-1]._az<0)
								evaluation++;
							break;
						case AX_GX:
							if (threshold._gx*threshold._gx < MPU_Filtering_buffer[k]._gx*MPU_Filtering_buffer[k]._gx)
								evaluation++;
							if (MPU_Filtering_buffer[k]._gx*MPU_Filtering_buffer[k-1]._gx<0)
								evaluation++;
							break;
						case AX_GY:
							if (threshold._gy*threshold._gy < MPU_Filtering_buffer[k]._gy*MPU_Filtering_buffer[k]._gy )
								evaluation++;
							if (MPU_Filtering_buffer[k]._gy*MPU_Filtering_buffer[k-1]._gy<0)
								evaluation++;
							break;
						case AX_GZ:
							if (threshold._gz*threshold._gz < MPU_Filtering_buffer[k]._gz*MPU_Filtering_buffer[k]._gz)
								evaluation++;
							if (MPU_Filtering_buffer[k]._gz*MPU_Filtering_buffer[k-1]._gz<0)
								evaluation++;
							break;
						default:
							break;
					}
				}
			}
			rgb_color = 0;

//			if(evaluation > 10)
//				xEventGroupSetBits(eventos, SD_WRITE_EVENT);


			scaled_evaluation = evaluation*scale;
			for (k = 0; k<3; k++)
			{
				switch(k)
				{
				case 0:
					saturation = SATURATION_GREEN;
					break;
				case 1:
					saturation = SATURATION_BLUE;
					break;
				case 2:
					saturation = SATURATION_RED;
					break;
				}
				color  = 0;
				if ((scaled_evaluation < memberFunctions[k].L_AX_IN)|| ( memberFunctions[k].U_AX_IN < scaled_evaluation ))
				{
					color  = 0;
				}
				else if((memberFunctions[k].L_AX_IN <= scaled_evaluation ) && (scaled_evaluation < memberFunctions[k].S_LB))
				{
					color = -memberFunctions[k].L_AX_IN+(scaled_evaluation/( memberFunctions[k].S_LB - memberFunctions[k].L_AX_IN )) * saturation ;
				}
				else if((memberFunctions[k].S_LB <= scaled_evaluation ) && (scaled_evaluation < memberFunctions[k].S_UB))
				{
					color = saturation;
				}
				else if((memberFunctions[k].S_UB <= scaled_evaluation ) && (scaled_evaluation < memberFunctions[k].U_AX_IN))
				{
					color = memberFunctions[k].U_AX_IN -(scaled_evaluation/( memberFunctions[k].U_AX_IN - memberFunctions[k].S_UB )) * saturation ;
				};
				// ToDo: add a semaphore to rgb_color variable
				switch(k)
				{
				 // Usually, there are 5 bits allocated for the red and blue color components (32 levels each) and 6 bits for the green component (64 levels),
					case 0: // GREEN
						rgb_color += (color << 5);
						break;
					case 1: // BLUE
						rgb_color += color;
						break;
					case 2: // RED
						rgb_color += (color << 11);
						break;
					default:
						break;
				}
			}

		}
	}
}

//############################### Display task ##################################

void MPU_Display_task(void * parameter)
{

	while(1)
	{

		 if(xSemaphoreTake( SpiMutex, portMAX_DELAY ))
		 {

				vTaskDelay(timing_parameters.display_refresh / portTICK_RATE_MS);
				ILI9341DrawInt(80, 140, evaluation, 4, &font_16x26, rgb_color, ILI9341_NAVY);


				xSemaphoreGive( SpiMutex );
		 }

	}


}

//############################### File saving task ##################################

void File_Saving_task(void * parameter)
{
	EventBits_t uxBits;
	portBASE_TYPE xStatus;
	MPU_Reading_t reading;
	uint8_t k = 0;

	while (1)
	{
		vTaskDelay(timing_parameters.file_saving_time / portTICK_RATE_MS);

		uxBits = xEventGroupWaitBits(eventos, SD_WRITE_EVENT,FALSE, FALSE, portMAX_DELAY);
		if (uxBits == SD_WRITE_EVENT)
		{
			 if(xSemaphoreTake( SpiMutex, portMAX_DELAY ))
			 {
				 	SD_spiConfig();
				 	if(k ==0)SD_mount();
					
					Led_On(RED_LED);

					reading = MPU_Filtering_buffer[k];
					SD_write("file.raw", &reading , sizeof(MPU_Reading_t));
					k++;

					if(k == MAX_FILTER_SIZE)
					{
						k=0;
						xEventGroupClearBits(eventos,SD_WRITE_EVENT);
						SD_umount();
					}
					Led_Off(RED_LED);
					xSemaphoreGive( SpiMutex );
			 }
		}
	}
}

//############################### Board task ##################################

void Teclado(void * parametros) {
   uint8_t tecla;
   uint8_t anterior = 0;

   while(1) {
      tecla = Read_Switches();
      if (tecla != anterior) {
         switch(tecla) {
            case TEC1:
            	xEventGroupSetBits(eventos, SD_WRITE_EVENT);
               break;
            case TEC2:
            	xEventGroupSetBits(eventos, EVENTO_INICIAR);
               break;
            case TEC3:
               break;
            case TEC4:
               break;
         }
         anterior = tecla;
      }
      vTaskDelay(100);

   }
}


/*==================[external functions definition]==========================*/

int main(void)
{

	//uint16_t i;
	uint8_t imu;
	initHardware();
	Init_Switches();
	Init_Uart_Ftdi();
	NVIC_SetPriority(26,configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(26);

	MPU_Initialize_Parameters();

	MPU9250InitI2C(100000, MPU9250_ADDRESS_AD0L);
	imu = WhoAmI();
	MPU9250Begin();

	SD_spiConfig();
	SD_spiBitRate_config (10000000);

	uint8_t initresult = ILI9341Init(DISP_SPI_PORT, DISP_GPIO_CS, DISP_GPIO_DC, DISP_GPIO_RST);
	ILI9341Fill(ILI9341_NAVY);

	/* Queue to hold the fresh read*/
	xqh_MPU_Unfiltered = xQueueCreate(MPU_QUEUE_SIZE, sizeof(MPU_Reading_t) ); 	// buffer to hold the fresh readings
	TickType_W2F_UnfileredBuffer = 0 / portTICK_RATE_MS;


	/* Creación del grupo de eventos */
	eventos = xEventGroupCreate();
	//eventos = xEventGroupCreate();
	SpiMutex = xSemaphoreCreateMutex();

	if (eventos != NULL) {

	xTaskCreate(Teclado, "Teclado", configMINIMAL_STACK_SIZE,NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(Uart_sending_task, "Enviar", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &enviar);
	xTaskCreate(Uart_receiving_task, "Recibir", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(MPU_Reading_task, "reading_task", configMINIMAL_STACK_SIZE*10, NULL, tskIDLE_PRIORITY+3, NULL);
	xTaskCreate(MPU_Filter_task, "filter_task", configMINIMAL_STACK_SIZE*10, NULL, tskIDLE_PRIORITY+2, NULL);
	xTaskCreate(File_Saving_task, "saving_task", configMINIMAL_STACK_SIZE*5, NULL, tskIDLE_PRIORITY+2, NULL);
	xTaskCreate(MPU_Display_task, "MPU_Display_task", configMINIMAL_STACK_SIZE*5,NULL, tskIDLE_PRIORITY +1, &Display);
	xTaskCreate(MPU_Risk_Evaluation_Task, "MPU_Risk_Evaluation_Task", configMINIMAL_STACK_SIZE*2,NULL, tskIDLE_PRIORITY +1, NULL);

	}


	SisTick_Init();
	vTaskStartScheduler();

	while (1);

	return 0;
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/



