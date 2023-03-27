/*
 * Eventos.h
 *
 *  Created on: 24 Mar 2023
 *      Author: Oscar
 */

#ifndef DRIVERS_EVENTOS_H_
#define DRIVERS_EVENTOS_H_

#include "event_groups.h"

#define EVENTO_ADC_MANUAL (1<<0)        // 1
#define EVENTO_ADC_PERIODICO (1<<4)     // 2

extern EventGroupHandle_t grupo_eventos;


#endif /* DRIVERS_EVENTOS_H_ */
