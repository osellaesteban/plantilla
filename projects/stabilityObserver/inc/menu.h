/*
 * menu.h
 *
 *  Created on: 15 oct. 2018
 *      Author: esteban
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "chip.h"
#include  <stdint.h>


typedef  void* (* action )(void); /* <puntero a una función que retorna un puntero*/

enum
{
	flag_variable_menu = 1,
	flag_Acc_threshold_menu,
	flag_Ang_Acc_threshold_menu,
	flag_filter_menu,
	flag_filter_submenu,
	flag_timing_menu
};

/**
 * @brief Estructura que representa un elemento de un menú
 */
typedef  struct{
	char *txt; 			/** < Etiqueta de la opción en el menú */
	action  doAction;	/** < Función a ejecutar al elegir la opción */
	uint8_t flag;
} menuItem;


/**
 * @brief Función para retornar la dirección de memoria del menú principal
 */
void * getMainMenu(void);

void * AccelMenuThreshold(void);

void * AngVelMenuThreshold(void);

void * AngleMenuThreshold(void);

void * goFilterMenu(void);

void * goTimingMenu(void);

void * goSavingMenu(void);

void * getMainMenu(void);

void * go_AccParams(void);

void * go_GiroParams(void);

void * go_MagParams(void);

void * config_time(void);

void * get_parameters(void);

void * filter_number(void);

void * go_back(void);



//void Receive_commands(char * commands, uint8_t size){
//
//uint8_t indice;
//float aux;
//static uint8_t index;
//char *string, *value;
//
//	if(flag == command_flag){
//		for(indice = 0; indice < sizeof(comandos) / sizeof(comando_t); indice++)
//		{
//			if (strcmp(commands, comandos[indice].comando) == 0) {
//				index = indice;
//				//Led_Toggle(comandos[index].led);
//				Menu_actions(comandos[indice].index);
//			}
//
//		}
//		flag = value_flag;
//	}
//	else
//		if(flag == value_flag){
//			//sscanf (cadena,"%f ", &aux);
//			aux = (float)strtod(cadena,&value);
//			//sprintf(number,"%.2f\n", aux);
//			//aux = strtod(cadena,&value);
//			flag = command_flag;
//			printf("   strtod = %f\n", aux);
//		}
//
//}

#endif /* INC_MENU_H_ */
