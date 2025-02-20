#include "AT.h"

// Buffers para dados de recepção e transmissão
char usart1_rx_buffer[BUFFER_SIZE];
char usart3_rx_buffer[BUFFER_SIZE];
char usart1_tx_buffer[BUFFER_SIZE];
char usart3_tx_buffer[BUFFER_SIZE];

volatile uint8_t usart1_rx_index = 0;
volatile uint8_t usart3_rx_index = 0;
volatile uint8_t usart1_tx_index = 0;
volatile uint8_t usart3_tx_index = 0;
volatile uint8_t usart1_tx_length = 0;
volatile uint8_t usart3_tx_length = 0;
volatile uint8_t usart1_tx_busy = 0;
volatile uint8_t usart3_tx_busy = 0;
volatile uint8_t usart1_command_ready = 0;
volatile uint8_t usart3_command_ready = 0;

// Inicialização da USART1 (Terminal 1)
void USART1_Init(void)
{
    // Habilita o clock da USART1 e do GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configura os pinos PA9 (TX) e PA10 (RX) para função alternativa (USART1)
    GPIOA->MODER |= (0b10 << 18) | (0b10 << 20); // Modo alternativo
    GPIOA->AFR[1] |= (0b0111 << 4) | (0b0111 << 8); // AF7 (USART1)

    // Configura o baud rate para 115200 (ajuste conforme necessário)
    USART1->BRR = 730; // Para 84 MHz e 115200 bps

    // Habilita TX, RX, interrupção de RX, interrupção de TX e USART
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_UE);

    // Configura a interrupção da USART1 no NVIC
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);
}

// Inicialização da USART3 (Terminal 2)
void USART3_Init(void)
{
    // Habilita o clock da USART3 e do GPIOD
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    // Configura os pinos PD8 (TX) e PD9 (RX) para função alternativa (USART3)
    GPIOD->MODER |= (0b10 << 16) | (0b10 << 18); // Modo alternativo
    GPIOD->AFR[1] |= (0b0111 << 0) | (0b0111 << 4); // AF7 (USART3)

    // Configura o baud rate para 115200 (ajuste conforme necessário)
    USART3->BRR = 364; // Para 42 MHz e 115200 bps

    // Habilita TX, RX, interrupção de RX, interrupção de TX e USART
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_UE);

    // Configura a interrupção da USART3 no NVIC
    NVIC_SetPriority(USART3_IRQn, 0);
    NVIC_EnableIRQ(USART3_IRQn);
}

// Inicialização do DMA (se for utilizado)
void DMA_Init(void)
{
    // Habilita o clock do DMA
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Configura o DMA para a USART1 TX (Stream 7, Channel 4)
    DMA2_Stream7->CR &= ~DMA_SxCR_EN; 	// Desabilita o stream para configurar
    while (DMA2_Stream7->CR & DMA_SxCR_EN);	// Aguarda desabilitar

    DMA2_Stream7->CR |= DMA_SxCR_CHSEL_2; // Canal 4
    DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);	// Endereço do registrador de dados da USART1
    DMA2_Stream7->M0AR = (uint32_t)usart1_tx_buffer; // Endereço do buffer de transmissão
    DMA2_Stream7->NDTR = BUFFER_SIZE;
    // Configura direção (memória para periférico), incremento de memória e interrupção de transferência completa
    DMA2_Stream7->CR |= DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE;

    DMA2_Stream7->CR |= DMA_SxCR_EN; // Habilita o stream
    while (!(DMA2_Stream7->CR & DMA_SxCR_EN)); // Aguarda habilitar

    // Configura o DMA para a USART3 TX (Stream 3, Channel 4)
    DMA1_Stream3->CR &= ~DMA_SxCR_EN; // Desabilita o stream
    while (DMA1_Stream3->CR & DMA_SxCR_EN); // Aguarda desabilitar

    DMA1_Stream3->CR |= DMA_SxCR_CHSEL_2; // Canal 4
    DMA1_Stream3->PAR = (uint32_t)&USART3->DR; // Endereço do registrador de dados da USART3
    DMA1_Stream3->M0AR = (uint32_t)usart3_tx_buffer; // Endereço do buffer de transmissão
    DMA1_Stream3->NDTR = BUFFER_SIZE; // Tamanho do buffer
    // Configura direção (memória para periférico), incremento de memória e interrupção de transferência completa
    DMA1_Stream3->CR |= DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE;

    DMA1_Stream3->CR |= DMA_SxCR_EN; // Habilita o stream
    while (!(DMA1_Stream3->CR & DMA_SxCR_EN)); // Aguarda habilitar

    // Habilita as interrupções do DMA no NVIC
    NVIC_SetPriority(DMA2_Stream7_IRQn, 0);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    NVIC_SetPriority(DMA1_Stream3_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

// Transmite dados pela USART1 (console)
void console_transmit(char *data)
{
	uint16_t length = strlen(data);
    while (usart1_tx_busy); // Aguarda término da transmissão anterior

    // Copia os dados para o buffer de transmissão
    memcpy(usart1_tx_buffer, data, length);
    usart1_tx_length = length;
    usart1_tx_index = 0;
    usart1_tx_busy = 1;

    // Se estiver utilizando DMA, configure:
    // DMA2_Stream7->NDTR = length;
    // DMA2_Stream7->CR |= DMA_SxCR_EN;

    // Habilita a interrupção de TX
    USART1->CR1 |= USART_CR1_TXEIE;
}

// Transmite dados pela USART3 (módulo)
void module_transmit(char *data)
{
    uint16_t length = strlen(data);
    while (usart3_tx_busy); // Aguarda término da transmissão anterior

    // Copia os dados para o buffer de transmissão
    memcpy(usart3_tx_buffer, data, length);
    usart3_tx_length = length;
    usart3_tx_index = 0;
    usart3_tx_busy = 1;

    // Se estiver utilizando DMA, configure:
    // DMA1_Stream3->NDTR = length;

    // Habilita a interrupção de TX
    USART3->CR1 |= USART_CR1_TXEIE;
}

//limpar o buffer da USART3
void clear_buffer_usart3(void)
{
    memset(usart3_rx_buffer, 0, BUFFER_SIZE);
    usart3_rx_index = 0;
    usart3_command_ready = 0;
}

void clear_buffer_usart1(void)
{
    memset(usart1_rx_buffer, 0, BUFFER_SIZE);
    usart1_rx_index = 0;
    usart1_command_ready = 0;
}

// Processa os dados recebidos
void process_received_data(char *buffer, uint8_t *index, uint8_t *flag)
{
    if (*index > 0) //Verifica se há dados no buffer
    {
        buffer[*index] = '\0'; //Finaliza a string
        *flag = 1; //Indica que um comando completo foi recebido
        *index = 0; //Reseta o índice
    }
}

//Função para processar múltiplas linhas
void process_received_lines(void)
{
    uint8_t i = 0;
    uint8_t line_start = 0;

    //Processa o buffer enquanto houver dados
    while(i < usart3_rx_index) {
        if(usart3_rx_buffer[i] == '\n') {
            // Finaliza a linha
            usart3_rx_buffer[i] = '\0';
            // Envia a linha para o terminal
            console_transmit(&usart3_rx_buffer[line_start]);
            // Atualiza o início da próxima linha (pula o '\0')
            line_start = i + 1;
        }
        i++;
    }

    // Move os dados não processados para o início do buffer
    if(line_start < usart3_rx_index) {
        uint8_t remaining = usart3_rx_index - line_start;
        memmove(usart3_rx_buffer, &usart3_rx_buffer[line_start], remaining);
        usart3_rx_index = remaining;
    } else {
        usart3_rx_index = 0;
    }
}

// Trata a recepção da USART1
void handle_usart1_rx(void)
{
    char receivedChar = (char)USART1->DR; // Lê o dado recebido

    if (usart1_rx_index < BUFFER_SIZE - 1) // Armazena o caractere se houver espaço
    {
        usart1_rx_buffer[usart1_rx_index++] = receivedChar;
    }
}

// Trata a recepção da USART3
void handle_usart3_rx(void)
{
    char receivedChar = (char)USART3->DR; // Lê o dado recebido

    if (usart3_rx_index < BUFFER_SIZE - 1) // Armazena o caractere se houver espaço
    {
        usart3_rx_buffer[usart3_rx_index++] = receivedChar;
    }
}

// Tratador de interrupção do DMA para a USART1 TX (DMA2 Stream7)
void DMA2_Stream7_IRQHandler(void)
{
    if (DMA2->HISR & DMA_HISR_TCIF7)
    {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF7; // Limpa a flag de transferência completa
        usart1_tx_busy = 0; // Indica término da transmissão
    }
}

// Tratador de interrupção do DMA para a USART3 TX (DMA1 Stream3)
void DMA1_Stream3_IRQHandler(void)
{
    if (DMA1->LISR & DMA_LISR_TCIF3)
    {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF3; // Limpa a flag de transferência completa
        usart3_tx_busy = 0; // Indica término da transmissão
    }
}

// Trata a transmissão da USART1
void handle_usart1_tx(void)
{
    if (usart1_tx_index < usart1_tx_length) //Verifica se ainda há bytes para enviar
    {
        USART1->DR = usart1_tx_buffer[usart1_tx_index++]; // Transmite o próximo byte
    }
    else
    {
        // Desabilita a interrupção de TX
        USART1->CR1 &= ~USART_CR1_TXEIE;
        usart1_tx_busy = 0; // Transmissão concluída
    }
}

// Trata a transmissão da USART3
void handle_usart3_tx(void)
{
    if (usart3_tx_index < usart3_tx_length) // Verifica se ainda há bytes para enviar
    {
        USART3->DR = usart3_tx_buffer[usart3_tx_index++]; // Transmite o próximo byte
    }
    else
    {
        // Desabilita a interrupção de TX
        USART3->CR1 &= ~USART_CR1_TXEIE;
        usart3_tx_busy = 0; // Transmissão concluída
    }
}

// Interrupção da USART1 (Terminal 1)
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)
        handle_usart1_rx(); // Trata recepção

    if (USART1->SR & USART_SR_TXE)
        handle_usart1_tx(); // Trata transmissão
}

// Interrupção da USART3 (Terminal 2)
void USART3_IRQHandler(void)
{
    if (USART3->SR & USART_SR_RXNE)
        handle_usart3_rx(); // Trata recepção

    if (USART3->SR & USART_SR_TXE)
        handle_usart3_tx(); // Trata transmissão
}

// Testa os comandos AT recebidos
void test_Atcommand(void)
{
    // Processa os dados recebidos pela USART1
    process_received_data(usart1_rx_buffer, &usart1_rx_index, &usart1_command_ready);

    // Processa os dados recebidos pela USART3
    process_received_data(usart3_rx_buffer, &usart3_rx_index, &usart3_command_ready);

    // Se um comando foi recebido via USART1, envia-o para a USART3 (módulo)
    if (usart1_command_ready)
    {
        usart1_command_ready = 0; // Reseta a flag
        module_transmit(usart1_rx_buffer);
        memset(usart1_rx_buffer, 0, BUFFER_SIZE);
    }

    // Se um comando foi recebido via USART3, envia-o para a USART1 (console)
    if (usart3_command_ready)
    {
        usart3_command_ready = 0; // Reseta a flag
        console_transmit(usart3_rx_buffer);
        memset(usart3_rx_buffer, 0, BUFFER_SIZE);
    }
}

void send_ATcommand()
{

	// Processa os dados recebidos pela USART3
	process_received_data(usart3_rx_buffer, &usart3_rx_index, &usart3_command_ready);

	// Se um comando foi recebido via USART1, envia-o para a USART3 (módulo)
	if (usart1_command_ready)
	{
		usart1_command_ready = 0; // Reseta a flag
		module_transmit(usart1_rx_buffer);
		memset(usart1_rx_buffer, 0, BUFFER_SIZE);
	}

	// Se um comando foi recebido via USART3, envia-o para a USART1 (console)
	if (usart3_command_ready)
	{
		usart3_command_ready = 0; // Reseta a flag
		console_transmit(usart3_rx_buffer);
		memset(usart3_rx_buffer, 0, BUFFER_SIZE);
	}
}


// Envia o comando AT básico
uint8_t send_at(void)
{
	char command[16];
	uint32_t timeout = 1000;

	strcpy(command, AT);

	//Envia o comando para a USART3
	module_transmit(command);

	//Aguarda resposta com timeout usando delay_ms
	while (!usart3_command_ready && timeout > 0) {

		send_ATcommand();
		Delay_ms(50);  // Pequeno delay para evitar loop muito rápido
		timeout -= 50;
	}

	if (timeout <= 0)
	{
		return 0;  // Timeout
	}

	return 1;
}

// Envia o comando AT de reset
uint8_t send_at_rst(void)
{
	char command[16];
	uint32_t timeout = 5000;

	strcpy(command, AT_RST);

	//Envia o comando para a USART3
	module_transmit(command);

	//Aguarda resposta com timeout usando delay_ms
	while(!usart3_command_ready && timeout > 0)
	{
		send_ATcommand();
		Delay_ms(50);  // Pequeno delay para evitar loop muito rápido
		timeout -= 50;
	}

	if (timeout <= 0)
	{
		return 0;  // Timeout
	}

	return 1;
}


// Envia o comando AT de restauração (atualmente igual ao reset)
void send_at_restore(void)
{
    module_transmit(AT_RST);
}

// Configura o modo de Wi-Fi via comando AT+CWMODE
uint8_t send_at_cwmode(WIFI_MODES mode, uint8_t auto_connect)
{
    char command[64];
    uint32_t timeout = 1000;

    // Monta o comando com base no modo desejado
    switch(mode)
    {
        case NULL_MODE:
            strcpy(command, "AT+CWMODE=0\r\n");
            break;
        case STATION_MODE:
            strcpy(command, "AT+CWMODE=1\r\n");
            break;
        case SOFT_AP_MODE:
            strcpy(command, "AT+CWMODE=2\r\n");
            break;
        case SOFT_AP_STATION_MODE:
            strcpy(command, "AT+CWMODE=3\r\n");
            break;
        default:
            return 0;
    }

    // Limpa o buffer da USART3
    memset(usart3_rx_buffer, 0, BUFFER_SIZE);
    usart3_rx_index = 0;
    usart3_command_ready = 0;

    // Envia o comando para a USART3
    module_transmit(command);

    // Aguarda resposta com timeout usando delay_ms
    while (!usart3_command_ready && timeout > 0)
    {
        //process_received_data(usart3_rx_buffer, &usart3_rx_index, &usart3_command_ready);
        Delay_ms(100);  // Pequeno delay para evitar loop muito rápido
        timeout -= 100;
    }

    if (timeout <= 0)
    {
        return 0;  // Timeout
    }

    // Resposta recebida, envia para o terminal
    console_transmit(usart3_rx_buffer);

    // (Opcional) Limpa o buffer de resposta
    clear_buffer_usart3();
    clear_buffer_usart1();

    return 1;  // Sucesso
}


// Envia o comando AT+CWJAP (exemplo comentado)
// Para habilitar, descomente e ajuste os parâmetros conforme necessário
void send_at_cwjap(void)
{
    module_transmit(AT_CWJAP("Assert", "w1f1:Assert!!"));
}

// Configuração de usuário para MQTT (placeholder)
void send_at_mqtt_user_cfg(void)
{
    // Implementação para configuração do MQTT
}

// Conexão MQTT (placeholder)
void send_at_mqtt_conn(void)
{
    // Implementação para conexão MQTT
}

// Publicação de mensagem MQTT (placeholder)
void send_at_mqtt_pub(void)
{
    // Implementação para publicação MQTT
}



