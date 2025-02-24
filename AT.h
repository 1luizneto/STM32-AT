/*
 * AT.h for STM32CubeIDE
 *
 *  Created on: 05 de jan de 2025
 *  Author: Luiz Neto
 *
 */

#ifndef AT_H
#define AT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
//#include "Utility.h"

// Tamanho do buffer para transmissão e recepção
#define BUFFER_SIZE 1024

// Comandos AT básicos
#define AT       "AT\r\n"
#define AT_RST   "AT+RST\r\n"
#define AT_GMR   "AT+GMR\r\n"

// Comandos AT para Wi-Fi
#define AT_CWJAP(SSID, PASSWORD) "AT+CWJAP=\"" SSID "\",\"" PASSWORD "\"\r\n"

// Comandos AT para TCP/IP (placeholders)
#define AT_CIPSTA     "AT\r\n"
#define AT_CIPSTART   "AT\r\n"
#define AT_CIPSEND    "AT\r\n"
#define AT_CIPSNTPCFG "AT\r\n"
#define AT_CIPSNTPTIME "AT\r\n"

// Comandos AT para MQTT (placeholders)
#define AT_MQTTUSERCFG "AT\r\n"
#define AT_MQTTCONN    "AT\r\n"
#define AT_MQTTCONNCFG "AT\r\n"
#define AT_MQTTPUB     "AT\r\n"
#define AT_MQTTSUB     "AT\r\n"
#define AT_MQTTCLEAN   "AT\r\n"
#define AT_MQTTPUBRAW  "AT\r\n"

// Comandos AT para HTTP (placeholders)
#define AT_HTTPCLIENT "AT\r\n"
//#define AT_HTTPCLIENT "AT\r\n" // Observação: comando duplicado, uma das linhas pode ser removida

// Enumerações para modos de Wi-Fi
typedef enum
{
    NULL_MODE,
    STATION_MODE,
    SOFT_AP_MODE,
    SOFT_AP_STATION_MODE
} WIFI_MODES;

// Enumeração para estados de Wi-Fi (exemplo)
typedef enum
{
    ON_WIFIMODE,
    CONNECT_WIFI
} WIFI_States;

// Protótipos das funções
void USART1_Init(void);
void USART3_Init(void);
void DMA_Init(void);
void console_transmit(char *data);
void module_transmit(char *data);
void process_received_data(char *buffer, uint8_t *index, uint8_t *flag);
void handle_usart1_rx(void);
void handle_usart3_rx(void);
void handle_usart1_tx(void);
void handle_usart3_tx(void);
void test_Atcommand(void);
uint8_t send_at(void);
uint8_t send_at_rst(void);
void send_at_restore(void);
uint8_t send_at_cwmode(WIFI_MODES mode, uint8_t auto_connect);
void send_at_cwjap(void);
void send_at_mqtt_user_cfg(void);
void send_at_mqtt_conn(void);
void send_at_mqtt_pub(void);
void clear_buffer_usart1(void);
void clear_buffer_usart3(void);

#endif // AT_H
