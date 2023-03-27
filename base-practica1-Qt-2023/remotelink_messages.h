/*
 * remotelink_messages.h
 *
 *  Created on: March. 2019
 *
 */


#ifndef RL_MESSAGES_H
#define RL_MESSAGES_H
//Codigos de los mensajes y definicion de parametros para el protocolo RPC

// El estudiante debe agregar aqui cada nuevo mensaje que implemente. IMPORTANTE el orden de los mensajes
// debe SER EL MISMO aqui, y en el codigo equivalente en la parte del microcontrolador (Code Composer)

typedef enum {
    MESSAGE_REJECTED,
    MESSAGE_PING,
    MESSAGE_LED_GPIO,
    MESSAGE_LED_PWM_BRIGHTNESS,
    MESSAGE_ADC_SAMPLE,
    MESSAGE_LED_MODE,
    MESSAGE_LED_PWM,
    MESSAGE_SWITCHES_POLL,
    MESSAGE_SWITCHES_INTERRUPT,
    MESSAGE_SWITCHES_INTERRUPT_DISABLE,
    MESSAGE_OVERSAMPLE,
    MESSAGE_ADC_AUTO_ENABLE,
    MESSAGE_ADC_AUTO_FRECUENCY,
    MESSAGE_ADC_AUTO_DISABLE,
    MESSAGE_ADC_AUTO_SAMPLE16,
    //etc, etc...
} messageTypes;

//Estructuras relacionadas con los parametros de los mensahes. El estuadiante debera crear las
// estructuras adecuadas a los comandos usados, y asegurarse de su compatibilidad en ambos extremos

#pragma pack(1) //Cambia el alineamiento de datos en memoria a 1 byte.

typedef struct {
    uint8_t command;
} MESSAGE_REJECTED_PARAMETER;

typedef union{
    struct {
         uint8_t padding:1;
         uint8_t red:1;
         uint8_t blue:1;
         uint8_t green:1;
    } leds;
    uint8_t value;
} MESSAGE_LED_GPIO_PARAMETER;

typedef struct {
    float rIntensity;
} MESSAGE_LED_PWM_BRIGHTNESS_PARAMETER;

typedef struct
{
    uint16_t chan[7];
} MESSAGE_ADC_SAMPLE_PARAMETER;

typedef struct {
    uint8_t mode;
} MESSAGE_LED_MODE_PARAMETER;

typedef struct {
    uint8_t red;
    uint8_t blue;
    uint8_t green;
} MESSAGE_LED_PWM_PARAMETER;

typedef struct {
    uint8_t sw1;
    uint8_t sw2;
} MESSAGE_POLL_PARAMETER;

typedef struct {
    uint8_t sw1;
    uint8_t sw2;
} MESSAGE_INTERRUPT_PARAMETER;

typedef struct {
    uint32_t factor;
} MESSAGE_OVERSAMPLE_PARAMETER;

typedef struct {
    uint32_t frecuency;
} MESSAGE_ADC_AUTO_ENABLE_PARAMETER;

typedef struct {
    uint32_t frecuency;
} MESSAGE_ADC_AUTO_FRECUENCY_PARAMETER;

typedef struct {
    uint16_t chan[6][16];
} MESSAGE_ADC_AUTO_SAMPLE16_PARAMETER;

#pragma pack()  //...Pero solo para los comandos que voy a intercambiar, no para el resto.

#endif // RPCCOMMANDS_H


