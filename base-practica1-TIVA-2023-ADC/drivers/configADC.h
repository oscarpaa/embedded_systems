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
    uint16_t chan[3];
} MuestrasADC1;

typedef struct
{
	uint32_t chan[7];
} MuestrasLeidasADC;

typedef struct
{
    uint32_t chan[3];
} MuestrasLeidasADC1;

enum {
    MODE_ADC_MANUAL,
    MODE_ADC_AUTO,
};

void configADC_ISR(void);
void configADC_DisparaADC(void);
void configADC_LeeADC0(MuestrasADC *datos);
void configADC_IniciaADC0(void);

void configADC_ISR1(void);
void configADC_LeeADC1(MuestrasADC1 *datos);
void configADC_IniciaADC1(void);

void configADC_Promedio(uint32_t factor);
void configADC_Mode(uint8_t mode, float frecuency);

#endif /* CONFIGADC_H_ */
