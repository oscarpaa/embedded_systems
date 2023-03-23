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
	uint16_t chan1;
	uint16_t chan2;
	uint16_t chan3;
	uint16_t chan4;
	uint16_t chan5;
	uint16_t chan6;
	uint16_t temp;
} MuestrasADC;

typedef struct
{
	uint32_t chan1;
	uint32_t chan2;
	uint32_t chan3;
	uint32_t chan4;
	uint32_t chan5;
	uint32_t chan6;
	uint32_t temp;
} MuestrasLeidasADC;


void configADC_ISR(void);
void configADC_DisparaADC(void);
void configADC_LeeADC(MuestrasADC *datos);
void configADC_IniciaADC(void);


#endif /* CONFIGADC_H_ */
