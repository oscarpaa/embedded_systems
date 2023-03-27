/*
 * configADC.h
 *
 *  Created on: 22/4/2016
 *      Author: jcgar
 */

#ifndef CONFIGADC_H_
#define CONFIGADC_H_

#include<stdint.h>

typedef struct
{
	uint16_t chan[7];
} MuestrasADC;

typedef struct
{
	uint32_t chan[7];
} MuestrasLeidasADC;


void configADC_ISR(void);
void configADC_DisparaADC(void);
void configADC_LeeADC(MuestrasADC *datos);
void configADC_IniciaADC(void);
void configADC_Promedio(uint32_t factor);
void configADC_Mode(uint8_t mode, uint32_t frecuency);

#endif /* CONFIGADC_H_ */
