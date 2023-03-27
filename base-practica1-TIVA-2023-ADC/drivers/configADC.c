#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


static QueueHandle_t cola_adc;

//Provoca el disparo de una conversion (hemos configurado el ADC con "disparo software" (Processor trigger)
void configADC_DisparaADC(void)
{
	ADCProcessorTrigger(ADC0_BASE,0);
}

void configADC_Promedio(uint32_t factor)
{
    ADCHardwareOversampleConfigure(ADC0_BASE, factor);
}

void configADC_Mode(uint8_t mode, uint32_t frecuency)
{
    switch (mode) {
        case ADC_STATE_MANUAL:
        {
            TimerControlTrigger(TIMER2_BASE, TIMER_A, false); // TIMER1,0 -> RGB, CPUUsage -> TIMER3
            TimerDisable(TIMER2_BASE, TIMER_A);

            ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);  //Disparo software (processor trigger)
        }
        break;

        case ADC_STATE_AUTO:
        {
            //Habilitar Temporizador 2
            SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
            SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
            //Configurar temporizador como periódico
            TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

            TimerControlTrigger(TIMER2_BASE, TIMER_A, true); // TIMER1,0 -> RGB, CPUUsage -> TIMER3

            //Cargar el valor que contará el temporizador
            TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/frecuency);
            TimerEnable(TIMER2_BASE, TIMER_A);

            ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_TIMER,0);
        }
        break;

        case ADC_STATE_CHANGE_FREC:
        {
            //Cargar el valor que contará el temporizador
            TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/frecuency);
        }
        break;
    }
}

void configADC_IniciaADC(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

    //HABILITAMOS EL GPIOE,GPIOD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);

    // Enable pin PE for ADC AIN0|AIN1|AIN2|AIN3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
    // Enable pin PD for ADC AIN6|AIN7
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_0);

    // 4 secuenciadores. 0->8 muestras; 1,2->4 muestras; 3-> 1 muestra
    //CONFIGURAR SECUENCIADOR 0
    ADCSequenceDisable(ADC0_BASE,0);

    //Configuramos la velocidad de conversion al maximo (1MS/s)
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

    ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);	//Disparo software (processor trigger)
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,0,2,ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE,0,3,ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE,0,4,ADC_CTL_CH6);
    ADCSequenceStepConfigure(ADC0_BASE,0,5,ADC_CTL_CH7);
    ADCSequenceStepConfigure(ADC0_BASE,0,6,ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE,0,7,ADC_CTL_IE|ADC_CTL_END);	//La ultima muestra provoca la interrupcion
    ADCSequenceEnable(ADC0_BASE,0); //ACTIVO LA SECUENCIA

    //Habilita las interrupciones
    ADCIntClear(ADC0_BASE,0);
    ADCIntEnable(ADC0_BASE,0);
    IntPrioritySet(INT_ADC0SS0,configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_ADC0SS0);

    //Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
    cola_adc=xQueueCreate(8,sizeof(MuestrasADC));
    if (cola_adc==NULL)
    {
        while(1);
    }
}


void configADC_LeeADC(MuestrasADC *datos)
{
	xQueueReceive(cola_adc,datos,portMAX_DELAY);
}

void configADC_ISR(void)
{
	portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

	MuestrasLeidasADC leidas;
	MuestrasADC finales;

	ADCIntClear(ADC0_BASE,0);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
	ADCSequenceDataGet(ADC0_BASE,0,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS

	//Pasamos de 32 bits a 16 (el conversor es de 12 bits, así que sólo son significativos los bits del 0 al 11)
	uint8_t i;
	for (i = 0; i < 7; ++i) {
	    finales.chan[i] = leidas.chan[i];
    }

	//Guardamos en la cola
	xQueueSendFromISR(cola_adc,&finales,&higherPriorityTaskWoken);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
