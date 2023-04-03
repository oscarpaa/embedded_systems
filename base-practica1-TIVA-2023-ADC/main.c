//*****************************************************************************
//
// Codigo de partida Practica 1.
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "drivers/configADC.h"
#include "commands.h"

#include <remotelink.h>
#include <serialprotocol.h>

//parametros de funcionamiento de la tareas
#define REMOTELINK_TASK_STACK (512)
#define REMOTELINK_TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define COMMAND_TASK_STACK (512)
#define COMMAND_TASK_PRIORITY (tskIDLE_PRIORITY+1)

#define STATES_TASK_STACK (256)
#define STATES_TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define EVENTS_TASK_STACK (512)
#define EVENTS_TASK_PRIORITY (tskIDLE_PRIORITY+1)

#define EVENT_ADC_MANUAL (1<<0)            // 1
#define EVENT_ADC_AUTO (1<<1)              // 2

//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;

static QueueHandle_t cola_estados;
static TaskHandle_t tarea_eventos;
static EventGroupHandle_t grupo_eventos;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask,  char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

static portTASK_FUNCTION(STATE_SWITCHESTask,pvParameters)
{
    MESSAGE_INTERRUPT_PARAMETER switches;
    while(1)
    {
        xQueueReceive(cola_estados,&switches,portMAX_DELAY);
        remotelink_sendMessage(MESSAGE_SWITCHES_INTERRUPT,(void *)&switches,sizeof(switches));
    }
}

static portTASK_FUNCTION(EVENTSTask,pvParameters)
{
    EventBits_t eventos;

    MuestrasADC adc_muestras;
    MESSAGE_ADC_SAMPLE_PARAMETER adc_manual;

    MuestrasADC1 adc1_muestras;

    MESSAGE_ADC_AUTO_SAMPLE16_PARAMETER adc_uni;
    MESSAGE_ADC_AUTO_SAMPLE32_PARAMETER adc_diff;

    uint8_t cont_uni = 0;
    uint8_t cont_diff = 0;

    while(1)
    {
        eventos = xEventGroupWaitBits(grupo_eventos, EVENT_ADC_MANUAL|EVENT_ADC_AUTO, pdFALSE, pdFALSE, portMAX_DELAY);

        if (eventos & EVENT_ADC_MANUAL)
        {
            configADC_LeeADC0(&adc_muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)

            //Copia los datos en el parametro (es un poco redundante)
            uint8_t i;
            for (i = 0; i < 7; ++i) {
                adc_manual.chan[i] = adc_muestras.chan[i];
            }

            //Encia el mensaje hacia QT
            remotelink_sendMessage(MESSAGE_ADC_SAMPLE,(void *)&adc_manual,sizeof(adc_manual));
        }
        else if (eventos & EVENT_ADC_AUTO)
        {
            configADC_LeeADC0(&adc_muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)

            //Copia los datos en el parametro (es un poco redundante)
            uint8_t i;
            for (i = 0; i < 6; ++i) {
                adc_uni.chan[i][cont_uni] = adc_muestras.chan[i];
            }

            if (cont_uni == 15)
            {
                cont_uni = 0;
                //Encia el mensaje hacia QT
                remotelink_sendMessage(MESSAGE_ADC_AUTO_SAMPLE16,(void *)&adc_uni,sizeof(adc_uni));
            }
            else
            {
                cont_uni++;
            }

            configADC_LeeADC1(&adc1_muestras);

            //Copia los datos en el parametro (es un poco redundante)
            for (i = 0; i < 3; ++i) {
                adc_diff.chan[i][cont_diff] = adc1_muestras.chan[i];
            }

            if (cont_diff == 31)
            {
                cont_diff = 0;
                //Encia el mensaje hacia QT
                remotelink_sendMessage(MESSAGE_ADC_AUTO_SAMPLE32,(void *)&adc_diff,sizeof(adc_diff));
            }
            else
            {
                cont_diff++;
            }
        }
    }
}


//Funcion callback que procesa los mensajes recibidos desde el PC (ejecuta las acciones correspondientes a las ordenes recibidas)
static int32_t messageReceived(uint8_t message_type, void *parameters, int32_t parameterSize)
{
    int32_t status=0;   //Estado de la ejecucion (positivo, sin errores, negativo si error)

    //Comprueba el tipo de mensaje
    switch (message_type)
    {
        case MESSAGE_PING:
        {
            status=remotelink_sendMessage(MESSAGE_PING,NULL,0);
        }
        break;
        case MESSAGE_LED_GPIO:
        {
                MESSAGE_LED_GPIO_PARAMETER parametro;

                if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,parametro.value);
                }
                else
                {
                    status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
                }
        }
        break;
        case MESSAGE_LED_PWM_BRIGHTNESS:
        {
            MESSAGE_LED_PWM_BRIGHTNESS_PARAMETER parametro;

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                RGBIntensitySet(parametro.rIntensity);
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;
        case MESSAGE_ADC_SAMPLE:
        {
            xEventGroupSetBits(grupo_eventos,EVENT_ADC_MANUAL);
            configADC_DisparaADC(); //Dispara la conversion (por software)
        }
        break;
        case MESSAGE_LED_MODE:
        {
            MESSAGE_LED_MODE_PARAMETER parametro;

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                if (parametro.mode)
                {
                    RGBDisable();
                    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
                }
                else
                {
                    RGBEnable();
                }
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;
        case MESSAGE_LED_PWM:
        {
            MESSAGE_LED_PWM_PARAMETER parametro;
            uint32_t arrayRGB[3];

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                arrayRGB[0] = parametro.red << 8;
                arrayRGB[1] = parametro.green << 8;
                arrayRGB[2] = parametro.blue << 8;

                RGBColorSet(arrayRGB);
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;
        case MESSAGE_SWITCHES_POLL:
        {
            MESSAGE_POLL_PARAMETER parametro;
            int32_t i32PinStatus = GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);

            parametro.sw1 = !(i32PinStatus & LEFT_BUTTON);
            parametro.sw2 = !(i32PinStatus & RIGHT_BUTTON);
            status=remotelink_sendMessage(MESSAGE_SWITCHES_POLL,(void *)&parametro,sizeof(parametro));
        }
        break;
        case MESSAGE_SWITCHES_INTERRUPT:
        {
            GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
        }
        break;
        case MESSAGE_SWITCHES_INTERRUPT_DISABLE:
        {
            GPIOIntDisable(GPIO_PORTF_BASE,ALL_BUTTONS);
        }
        break;
        case MESSAGE_OVERSAMPLE:
        {
            MESSAGE_OVERSAMPLE_PARAMETER parametro;

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                configADC_Promedio(parametro.factor);
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;
        case MESSAGE_ADC_AUTO_DISABLE:
        {
            xEventGroupClearBits(grupo_eventos,EVENT_ADC_AUTO);

            configADC_Mode(MODE_ADC_MANUAL,0);
            vTaskDelete(tarea_eventos);
            if((xTaskCreate(EVENTSTask, (portCHAR *)"Tarea Eventos", EVENTS_TASK_STACK,NULL,EVENTS_TASK_PRIORITY, &tarea_eventos) != pdTRUE))
            {
                while(1);
            }
        }
        break;
        case MESSAGE_ADC_AUTO:
        {
            MESSAGE_ADC_AUTO_PARAMETER parametro;
            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                xEventGroupClearBits(grupo_eventos,EVENT_ADC_MANUAL);
                xEventGroupSetBits(grupo_eventos, EVENT_ADC_AUTO);

                configADC_Mode(MODE_ADC_AUTO,parametro.frecuency);

                vTaskDelete(tarea_eventos);
                if((xTaskCreate(EVENTSTask, (portCHAR *)"Tarea Eventos", EVENTS_TASK_STACK,NULL,EVENTS_TASK_PRIORITY, &tarea_eventos) != pdTRUE))
                {
                    while(1);
                }
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;
        case MESSAGE_ADC_AUTO_FRECUENCY:
        {
            MESSAGE_ADC_AUTO_FRECUENCY_PARAMETER parametro;
            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0)
            {
                configADC_Mode(MODE_ADC_AUTO,parametro.frecuency);
            }
            else
            {
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
            }
        }
        break;

       default:
           //mensaje desconocido/no implementado
           status=PROT_ERROR_UNIMPLEMENTED_COMMAND; //Devuelve error.
    }


    return status;   //Devuelve status
}


//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	MAP_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);

	//Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1
	RGBInit(1);
	MAP_SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	MAP_SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	MAP_SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	//Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo

	//Volvemos a configurar los LEDs en modo GPIO POR Defecto
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	 //Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
	ButtonsInit();  // <----------

	GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
    IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_GPIOF);


    if((grupo_eventos = xEventGroupCreate()) == NULL)
    {
        while(1);
    }

	if ((cola_estados = xQueueCreate(20, sizeof(MESSAGE_INTERRUPT_PARAMETER))) == NULL)
	{
	    while(1);
	}

	/********************************      Creacion de tareas *********************/

	//Tarea del interprete de comandos (commands.c)
    if (initCommandLine(COMMAND_TASK_STACK,COMMAND_TASK_PRIORITY) != pdTRUE)
    {
        while(1);
    }

	//Esta funcion crea internamente una tarea para las comunicaciones USB.
	//Ademas, inicializa el USB y configura el perfil USB-CDC
	if (remotelink_init(REMOTELINK_TASK_STACK,REMOTELINK_TASK_PRIORITY,messageReceived)!=pdTRUE)
	{
	    while(1); //Inicializo la aplicacion de comunicacion con el PC (Remote). Ver fichero remotelink.c
	}


	//Para especificacion 2: Inicializa el ADC y crea una tarea...
	configADC_IniciaADC0();
	configADC_IniciaADC1();

    // Control asíncrono mediante eventos de entradas digitales
    if((xTaskCreate(STATE_SWITCHESTask, (portCHAR *)"ESTADO INT", STATES_TASK_STACK,NULL,STATES_TASK_PRIORITY, NULL) != pdTRUE))
    {
        while(1);
    }

	if((xTaskCreate(EVENTSTask, (portCHAR *)"Tarea Eventos", EVENTS_TASK_STACK,NULL,EVENTS_TASK_PRIORITY, &tarea_eventos) != pdTRUE))
    {
        while(1);
    }


	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}


void GPIOFIntHandler(void)
{
    BaseType_t higherPriorityTaskWoken=pdFALSE;
    MESSAGE_INTERRUPT_PARAMETER parametro;
    int32_t i32PinStatus = GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);

    parametro.sw1 = !(i32PinStatus & LEFT_BUTTON);
    parametro.sw2 = !(i32PinStatus & RIGHT_BUTTON);

    xQueueSendFromISR(cola_estados, &parametro, &higherPriorityTaskWoken);

    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

