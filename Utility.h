/*
 * Utility.h for STM32CubeIDE
 *
 *  Created on: 04 de dez de 2024
 *  Author: Prof. Dr. Fagner de Araujo Pereira e Luiz Oliveira
 *
 */

#ifndef UTILITY_H_
#define UTILITY_H_

//Definições Globais
//Números dos pinos GPIO
#define PIN_0	((uint8_t) 0)
#define PIN_1	((uint8_t) 1)
#define PIN_2	((uint8_t) 2)
#define PIN_3	((uint8_t) 3)
#define PIN_4	((uint8_t) 4)
#define PIN_5	((uint8_t) 5)
#define PIN_6	((uint8_t) 6)
#define PIN_7	((uint8_t) 7)
#define PIN_8	((uint8_t) 8)
#define PIN_9	((uint8_t) 9)
#define PIN_10	((uint8_t) 10)
#define PIN_11	((uint8_t) 11)
#define PIN_12	((uint8_t) 12)
#define PIN_13	((uint8_t) 13)
#define PIN_14	((uint8_t) 14)
#define PIN_15	((uint8_t) 15)

//Modos de operação de um pino GPIO
#define	INPUT		((uint8_t) 0b00)	//modo de entrada digital
#define	OUTPUT		((uint8_t) 0b01)	//modo de saída digital
#define	ALTERNATE	((uint8_t) 0b10)	//modo de função alternativa
#define	ANALOG		((uint8_t) 0b11)	//modo analógico

//Tipos de saída de um pino GPIO
#define	PUSH_PULL	((uint8_t) 0)	//saída push-pull
#define	OPEN_DRAIN	((uint8_t) 1)	//saída open-drain

//Níveis lógicos de saída de um pino
#define	LOW		((uint8_t) 0)	//nível lógico baixo
#define	HIGH	((uint8_t) 1)	//nível lógico alto

//Modos de operação dos resistores de pull-up e pull-down
#define	PULL_UP		((uint8_t) 0b01)	//resistor de pull-up
#define	PULL_DOWN	((uint8_t) 0b10)	//resistor de pull-down

//Canais do ADC
#define ADC_IN0  ((uint8_t) 0)
#define ADC_IN1  ((uint8_t) 1)
#define ADC_IN2  ((uint8_t) 2)
#define ADC_IN3  ((uint8_t) 3)
#define ADC_IN4  ((uint8_t) 4)
#define ADC_IN5  ((uint8_t) 5)
#define ADC_IN6  ((uint8_t) 6)
#define ADC_IN7  ((uint8_t) 7)
#define ADC_IN8  ((uint8_t) 8)
#define ADC_IN9  ((uint8_t) 9)
#define ADC_IN10 ((uint8_t) 10)
#define ADC_IN11 ((uint8_t) 11)
#define ADC_IN12 ((uint8_t) 12)
#define ADC_IN13 ((uint8_t) 13)
#define ADC_IN14 ((uint8_t) 14)
#define ADC_IN15 ((uint8_t) 15)

//Modos de operação do ADC
#define	SINGLE_CHANNEL	((uint8_t) 0)
#define MULTI_CHANNEL	((uint8_t) 1)

//Resoluções do ADC (em bits)
#define ADC_RES_12BITS	0b00
#define ADC_RES_10BITS	0b01
#define ADC_RES_8BITS	0b10
#define ADC_RES_6BITS	0b11

//Canais do DAC
#define DAC_CHANNEL1	((uint8_t) 0)
#define DAC_CHANNEL2	((uint8_t) 1)

//Resoluções do DAC (em bits)
#define DAC_RES_12BITS	((uint8_t) 0)
#define DAC_RES_8BITS	((uint8_t) 1)



//Protótipos de funções úteis

//Funções de configuração do sistema de clock
void Utility_Init(void);		//inicialização de funções da biblioteca
void Configure_Clock(void);		//configuração do sistema de clock

//Funções de timers e temporização
void TIM2_Setup(void);				//configuração do Timer2 como base de tempo de 1us
void Delay_us(uint32_t delay);		//atraso em us
void Delay_ms(uint32_t delay);		//atraso em ms

//Funções de manipulação de GPIO
void GPIO_Clock_Enable(GPIO_TypeDef* GPIOx);								//habilita o clock de um GPIO
void GPIO_Pin_Mode(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode);			//configura o modo de operação de um pino de um GPIO
void GPIO_Output_Mode(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode);		//configura o tipo de saída de um pino de um GPIO
static inline void GPIO_Write_Pin(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t level);		//escreve um nível lógico em um pino de um GPIO
static inline void GPIO_Toggle_Pin(GPIO_TypeDef* GPIOx, uint8_t pin);					//inverte o nível lógico em um pino de um GPIO
static inline void GPIO_Write_Port(GPIO_TypeDef* GPIOx, uint16_t value);				//escreve um valor numa porta GPIO
void GPIO_Resistor_Enable(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode);	//habilita os resistores de pull-up ou pull-down
static inline uint8_t GPIO_Read_Pin(GPIO_TypeDef* GPIOx, uint8_t pin);		//lê e retorna o nível lógico em um pino de um GPIO
static inline uint16_t GPIO_Read_Port(GPIO_TypeDef* GPIOx);					//lê e retorna os níveis lógicos de uma porta GPIO

//Funções de manipulação do ADC
void ADC_Init(ADC_TypeDef *ADCx, uint8_t mode, uint8_t resolution);			//inicializa o ADC em SINGLE_CHANNEL ou MULTI_CHANNEL e especifica resolução
void ADC_SingleChannel(ADC_TypeDef *ADCx, uint8_t channel);					//configura o ADC para converter um único canal
uint16_t ADC_GetSingleConversion(ADC_TypeDef *ADCx);						//lê o valor convertido de um único canal
void ADC_MultiChannel(ADC_TypeDef *ADCx, uint8_t numChannels, uint8_t *channels);			//configura o ADC para converter múltiplos canais
void ADC_GetMultiConversions(ADC_TypeDef *ADCx, uint8_t numChannels, uint16_t *results);	//lê os valores convertidos de múltiplos canais

//Funções de manipulação do DAC
void DAC_Init(uint8_t channel);											//inicializa um canal do ADC
void DAC_SetValue(uint8_t channel, uint16_t value, uint8_t resolution);	//escreve um valor com a resolução especificada no ADC selecionado

//Funções de periféricos de comunicação
//void USART1_Init(void);				//configuração da USART1 para debug com printf

//Funções de geração de números aleatórios
uint32_t Random_Number(void);		//retorna um número aleatório de 32 bits



//Declarações de funções úteis

//Funções de configuração do sistema de clock do STM32
//Inicialização de funções da biblioteca
void Utility_Init(void)
{
	Configure_Clock();	//inicializa o sistema de clock
	TIM2_Setup();		//configura o Timer2 como base de tempo de 1us
}

//Configuração do sistema de clock para velocidade máxima em todos os barramentos utilizando um cristal externo de 8MHz
//HCLK = 168 MHz
//APB1 = 42 MHz
//APB2 = 84 MHz
//USB, RNG, SDIO = 48 MHz
void Configure_Clock(void)
{
	//Parâmetros do PLL principal
	#define PLL_M	4
	#define PLL_N	168
	#define PLL_P	2
	#define PLL_Q	7

	//Configurações da Flash e do regulador de tensão para permitir operação em alta velocidade
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;	//habilita prefetch, caches e a latência da flash
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;	//habilita o clock da interface de energia
	PWR->CR |= PWR_CR_VOS;				//regulador de tensão no modo 2

	//Configuração do modo de agrupamento de prioridades das interrupções
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//Reseta os registradores do módulo RCC para o estado inicial
	RCC->CIR = 0;				//desabilita todas as interrupções de RCC
	RCC->CR |= RCC_CR_HSION;	//liga o oscilador HSI
	RCC->CFGR = 0;				//reseta o registrador CFGR
	//Desliga HSE, CSS e o PLL e o bypass de HSE
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_HSEBYP);
	RCC->PLLCFGR = 0x24003010;	//reseta o registrador PLLCFGR

	//Configura a fonte de clock (HSE), os parâmetros do PLL e prescalers dos barramentos AHB, APB
	RCC->CR |= RCC_CR_HSEON;				//habilita HSE
	while(!((RCC->CR) & RCC_CR_HSERDY));	//espera HSE ficar pronto
    RCC->CFGR |= 0x9400;	//HCLK = SYSCLK/1, PCLK2 = HCLK/2, PCLK1 = HCLK/4

    //Configura a fonte de clock e os parâmetros do PLL principal
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (0x400000) | (PLL_Q << 24);

    RCC->CR |= RCC_CR_PLLON;			//habilita o PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));	//espera o PLL ficar pronto verificando a flag PLLRDY

    RCC->CFGR |= 0x2;					//seleciona o PLL como fonte de SYSCLK
    while((RCC->CFGR & 0xC) != 0x8);	//espera o PLL ser a fonte de SYSCLK
}



//Funções de timers e temporização
//Configura o timer 2 como base de tempo de 1us
void TIM2_Setup(void)
{
	//O modo padrão do contador é com contagem crescente
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;	//liga o clock do Timer2
	TIM2->PSC = 83;						//prescaler para incrementos a cada 1uS
	TIM2->EGR = TIM_EGR_UG;				//update event para escrever o valor do prescaler
	TIM2->CR1 |= TIM_CR1_CEN;			//habilita o timer
}

//Criação de atraso em us
void Delay_us(uint32_t delay)
{
	TIM2->CNT = 0;				//inicializa o contador com 0
	while(TIM2->CNT < delay);	//aguarda o tempo passar
}

//Criação de atraso em ms
void Delay_ms(uint32_t delay)
{
	uint32_t max = 1000*delay;
	TIM2->CNT = 0;				//inicializa o contador com 0
	while(TIM2->CNT < max);		//aguarda o tempo passar
}



//Funções de manipulação de GPIO
//Habilita o clock de um GPIO
void GPIO_Clock_Enable(GPIO_TypeDef* GPIOx)
{
	RCC->AHB1ENR |= (1 << ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE)));
}

//Configura o modo de operação de um pino de um GPIO
void GPIO_Pin_Mode(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode)
{
	GPIOx->MODER &= ~(0b11 << 2*pin);	//reseta os 2 bits do modo de operação
	GPIOx->MODER |= (mode << 2*pin);	//configura o modo selecionado
}

//Configura o tipo de saída de um pino de um GPIO
void GPIO_Output_Mode(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode)
{
	GPIOx->OTYPER &= ~(1 << pin);	//reseta o tipo de saída
	GPIOx->OTYPER |= (mode << pin);	//configura o tipo selecionado
}

//Escreve um nível lógico em um pino de um GPIO
static inline void GPIO_Write_Pin(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t level)
{
	if(level)
		GPIOx->ODR |= (1 << pin);	//nível alto no pino
	else
		GPIOx->ODR &= ~(1 << pin);	//nível baixo no pino

	//GPIOx->BSRR |= (1 << (pin + 16 * (1 - level)));	//alternativa usando o registrador BSRR
}

//Inverte o nível lógico em um pino de um GPIO
static inline void GPIO_Toggle_Pin(GPIO_TypeDef* GPIOx, uint8_t pin)
{
	GPIOx->ODR ^= (1 << pin);	//inverte o nível lógico no pino
}

//Escreve um valor numa porta GPIO
static inline void GPIO_Write_Port(GPIO_TypeDef* GPIOx, uint16_t value)
{
	GPIOx->ODR = value;
}

//Habilita os resistores de pull-up ou pull-down
void GPIO_Resistor_Enable(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode)
{
	GPIOx->PUPDR &= ~(0b11 << 2*pin);	//desabilita qualquer resistor
	GPIOx->PUPDR |= (mode << 2*pin);	//habilita o resistor selecionado
}

//Lê e retorna o nível lógico em um pino de um GPIO
static inline uint8_t GPIO_Read_Pin(GPIO_TypeDef* GPIOx, uint8_t pin)
{
	return ((GPIOx->IDR & (1 << pin)) >> pin);
}

//Lê e retorna os níveis lógicos de uma porta GPIO
static inline uint16_t GPIO_Read_Port(GPIO_TypeDef* GPIOx)
{
	return GPIOx->IDR;
}



//Funções de manipulação do ADC
//Inicializa o periférico ADCx de acordo com a configuração do clock e outros parâmetros.
//Configura o clock do ADC selecionado e habilita o ADC.
//@param  ADCx: onde x pode ser 1, 2 ou 3 para selecionar o ADC desejado.
//@param  mode: modo de operação do ADC (SINGLE_CHANNEL ou MULTI_CHANNEL).
//@param  resolution: resolução da conversão do ADC (ADC_RES_12BITS, ADC_RES_10BITS, ADC_RES_8BITS ou ADC_RES_6BITS).
void ADC_Init(ADC_TypeDef *ADCx, uint8_t mode, uint8_t resolution)
{
	//Configuração do Clock do ADCx
	//Habilitação do Clock
	RCC->APB2ENR |= (1 << ((uint32_t)ADCx - (ADC1_BASE)) / ((ADC2_BASE) - (ADC1_BASE))) << 8;

	//Configuração do prescaler do ADC
	ADC->CCR |= (0b01 << 16);	//prescaler /4 (84MHz/4 = 21MHz)

	//Configura modo de operação
	if(!mode)	//SINGLE_CHANNEL
	{
		ADCx->CR1 &= ~ADC_CR1_SCAN;	//desabilita o escaneamento de canais
		ADC1->CR2 &= ~ADC_CR2_EOCS;
	}
	else		//MULTI_CHANNEL
	{
		ADCx->CR1 |= ADC_CR1_SCAN;	//habilita o escaneamento de canais
		ADC1->CR2 |= ADC_CR2_EOCS;	//bit EOC setado ao final de cada conversão
	}

	//Definição da resolução
	ADCx->CR1 &= ~ADC_CR1_RES;		//limpa o campo da resolução
	ADCx->CR1 |= resolution << 24;	//escreve a resolução

	//Liga o ADCx
	ADCx->CR2 |= ADC_CR2_ADON;
	Delay_ms(1);	//aguarda o ADC iniciar
}

//Inicializa o ADC e configura o canal específico para conversão.
//Esta função configura o pino de entrada do canal selecionado como analógico
//e configura o sequenciador do ADC para realizar a conversão nesse canal.
//@param  ADCx: onde x pode ser 1, 2 ou 3 para selecionar o periférico ADC.
//@param  canal: número do canal a ser convertido (ADC_IN0 a ADC_IN15).
void ADC_SingleChannel(ADC_TypeDef *ADCx, uint8_t channel)
{
	//Configuração do canal a ser convertido pelo ADCx
	if(channel < 8)
	{
		GPIO_Clock_Enable(GPIOA);				//habilita o clock do GPIOA
		GPIO_Pin_Mode(GPIOA, channel, ANALOG);	//configura o pino PAx como função analógica
	}
	else if(channel == 8 || channel == 9)
	{
		GPIO_Clock_Enable(GPIOB);					//habilita o clock do GPIOB
		GPIO_Pin_Mode(GPIOB, channel - 8, ANALOG);	//configura o pino PB(x-8) como função analógica
	}
	else if(channel >= 10 && channel <= 15)
	{
		GPIO_Clock_Enable(GPIOC);					//habilita o clock do GPIOC
		GPIO_Pin_Mode(GPIOC, channel - 10, ANALOG);	//configura o pino PC(x-10) como função analógica
	}

	//Seleciona o canal a ser convertido pelo ADCx
	ADCx->SQR1 &= ~0xF << 20;	//seleciona a quantidade de canais (1 canal)
	ADCx->SQR3 &= ~0x1F;		//limpa o campo do canal
	ADCx->SQR3 |= channel;		//escreve o canal
}

//Inicia a conversão do ADC e retorna o valor lido.
//Esta função ativa a conversão do canal configurado e aguarda até que a conversão
//esteja completa antes de retornar o valor lido.
//@param  ADCx: onde x pode ser 1, 2 ou 3 para selecionar o ADC.
//@retval Retorna o valor da conversão realizada pelo ADC.
uint16_t ADC_GetSingleConversion(ADC_TypeDef *ADCx)
{
	ADCx->CR2 |= ADC_CR2_SWSTART;		//inicia a conversão
	while (!(ADCx->SR & ADC_SR_EOC));	//aguarda o fim da conversão
	return ADCx->DR;					//retorna o valor convertido
}

//Configura o ADC para múltiplos canais.
//Configura os pinos de entrada como analógicos para os canais selecionados
//e configura o sequenciador do ADC para realizar a sequência de conversões.
//@param  ADCx: onde x pode ser 1, 2 ou 3 para selecionar o ADC.
//@param  numChannels: quantidade de canais a serem convertidos.
//@param  channels: array contendo, ordenadamente, os canais a serem convertidos.
void ADC_MultiChannel(ADC_TypeDef *ADCx, uint8_t numChannels, uint8_t *channels)
{
    //Configuração dos pinos de entrada dos canais como analógicos
    for(int i = 0; i < numChannels; i++)
    {
        if(channels[i] < 8)
        {
            GPIO_Clock_Enable(GPIOA); 					//habilita o clock do GPIOA
            GPIO_Pin_Mode(GPIOA, channels[i], ANALOG);	//configura o pino PAx como analógico
        }
        else if(channels[i] >= 8 && channels[i] <= 9)
        {
            GPIO_Clock_Enable(GPIOB);						//habilita o clock do GPIOB
            GPIO_Pin_Mode(GPIOB, channels[i] - 8, ANALOG);	//configura o pino (PBx - 8) como analógico
        }
        else if(channels[i] >= 10 && channels[i] <= 15)
        {
            GPIO_Clock_Enable(GPIOC);						//habilita o clock do GPIOC
            GPIO_Pin_Mode(GPIOC, channels[i] - 10, ANALOG);	//configura o pino (PCx - 10) como analógico
        }
    }

    //Configuração do sequenciador do ADC com os canais selecionados
    int deslocamento = 0;
    for (int i = 0; i < numChannels; i++)
    {
        if(i < 6)
        {
            ADCx->SQR3 &= ~(0x1F << (i * 5));		//limpa os bits do campo do canal
            ADCx->SQR3 |= (channels[i] << (i * 5));	//configura o canal na posição do registrador
        }
        else if(i < 12)
        {
            deslocamento = (i - 6) * 5;
            ADCx->SQR2 &= ~(0x1F << deslocamento);			//limpa os bits do campo do canal
            ADCx->SQR2 |= (channels[i] << deslocamento);	//configura o canal na posição
        }
        else
        {
            deslocamento = (i - 12) * 5;
            ADCx->SQR1 &= ~(0x1F << deslocamento); 			//limpa os bits do campo do canal
            ADCx->SQR1 |= (channels[i] << deslocamento);	//configura o canal na posição
        }
    }

    //Define a quantidade de conversões no sequenciador
    ADCx->SQR1 &= ~(0xF << 20);				//limpa o campo de bits
    ADCx->SQR1 |= (numChannels - 1) << 20;	//escreve a quantidade de canais a ser convertido
}

//Realiza a conversão de múltiplos canais e armazena os resultados.
//Lê os valores convertidos pelo ADC para cada canal configurado no sequenciador.
//@param  ADCx: onde x pode ser 1, 2 ou 3 para selecionar o ADC.
//@param  numChannels: quantidade de canais a serem convertidos.
//@param  results: ponteiro do array onde os resultados serão armazenados.
void ADC_GetMultiConversions(ADC_TypeDef *ADCx, uint8_t numChannels, uint16_t *results)
{
    ADCx->CR2 |= ADC_CR2_SWSTART;  			//inicia a sequência de conversão
    for (int i = 0; i < numChannels; i++)
    {
        while (!(ADCx->SR & ADC_SR_EOC)); 	//aguarda o fim da conversão
        results[i] = ADCx->DR;            	//armazena o valor convertido no array
	}
}



//Funções de manipulação do DAC
//Inicializa o DAC e os pinos GPIO de acordo com canal selecionado.
//Esta função configura o pino GPIO associado ao DAC, habilita a interface digital
//e seleciona o canal apropriado com base no valor do parâmetro channel.
//@param  channel: identifica o canal DAC desejado (DAC_CHANNEL1 ou DAC_CHANNEL2).
void DAC_Init(uint8_t channel)
{
	GPIO_Clock_Enable(GPIOA);			//liga o clock do GPIOA
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;	//habilita o clock da interface digital do DAC

	if(!channel)	//seleciona canal 1
	{
		GPIO_Pin_Mode(GPIOA, PIN_4, ANALOG);	//configura PA4 como analógico
		DAC->CR |= DAC_CR_EN1;					//habilita o canal 1
		DAC->CR |= DAC_CR_BOFF1;				//desabilita o buffer 1
	}
	else			//seleciona canal 2
	{
		GPIO_Pin_Mode(GPIOA, PIN_5, ANALOG);	//configura PA5 como analógico
		DAC->CR |= DAC_CR_EN2;					//habilita o canal 2
		DAC->CR |= DAC_CR_BOFF2;				//desabilita o buffer 2
	}
}

//Escreve o valor de saída, com a resolução especificada, no canal selecionado do DAC.
//Esta função escreve o valor de saída no DAC selecionado com base no canal e resolução especificados.
//@param  channel: identifica o canal do DAC desejado (DAC_CHANNEL1 ou DAC_CHANNEL2).
//@param  value: valor a ser escrito no DAC (12 bits ou 8 bits conforme a resolução).
//@param  resolution: resolução do DAC (DAC_RES_12BITS ou DAC_RES_8BITS)
void DAC_SetValue(uint8_t channel, uint16_t value, uint8_t resolution)
{
	switch((channel << 1) | resolution)
	{
		case 0b00:
			DAC->DHR12R1 = value;	//escrita de 12 bits no canal 1
			break;

		case 0b01:
			DAC->DHR8R1 = value;	//escrita de 8 bits no canal 1
			break;

		case 0b10:
			DAC->DHR12R2 = value;	//escrita de 12 bits no canal 2
			break;

		case 0b11:
			DAC->DHR8R2 = value;	//escrita de 8 bits no canal 2
			break;
	}
}



//Funções de periféricos de comunicação
//Configuração básica da USART1
//void USART1_Init(void)
//{
//	//Configuração da USART1
//	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;				//habilita o clock da USART1
//	USART1->BRR = 730;									//ajusta baud rate para 1 Mbps (frequência do periférico de 84MHz)
//	//O estado default do registrador USART1->CR1 garante:
//	//1 stop bit, 8 bits de dados, sem bit de paridade,
//	//oversampling de 16 amostras por bit
//	USART1->CR1 |= (USART_CR1_TE | USART_CR1_UE);		//habilita apenas o transmissor e a USART1
//	//USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE);	//habilita o transmissor, receptor, interrupção de RX e a USART1
//
//	//Habilita a interrupção da USART1 no NVIC
//	//NVIC_SetPriority(USART1_IRQn, 0);	//seta a prioridade da USART1
//	//NVIC_EnableIRQ(USART1_IRQn);		//habilita a interrupção da USART1
//
//	//Configuração dos pinos PA9 (TX) e PA10(RX)
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			//habilita o clock do GPIOA
//	GPIOA->MODER |= (0b10 << 20) | (0b10 << 18);	//pinos PA10 e PA9 em função alternativa
//	GPIOA->AFR[1] |= (0b0111 << 8) | (0b0111 << 4);	//função alternativa 7 (USART1)
//}
//
////Redefinição da função de envio de dados pela USART1
//int __io_putchar(int ch)
//{
//	USART1->DR = ch;						//transmite o dado
//	while (!(USART1->SR & USART_SR_TXE));	//espera pelo fim da transmissão
//	return ch;
//}
//
////Redefinição da função de recebimento de dados pela USART1
//int __io_getchar(void)
//{
//   return (uint16_t)(USART1->DR);	//lê o dado recebido
//}

//ISR da USART1. Todas as ISR's estão definidas no arquivo startup_stm32f407vetx.s
//void USART1_IRQHandler(void)
//{
//	__io_putchar(__io_getchar());	//lê o dado e reenvia pela USART1
//}



//Funções de geração de números aleatórios
//Retorna um número aleatório de 32 bits
uint32_t Random_Number(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;	//habilita o clock do RNG
	RNG->CR |= RNG_CR_RNGEN;			//liga o RNG
	while(!(RNG->SR & RNG_SR_DRDY));	//aguarda um número aleatório estar pronto
	(void)RNG->DR;						//despreza o primeiro número aleatório gerado
	while(!(RNG->SR & RNG_SR_DRDY));	//aguarda novo número aleatório estar pronto
	RNG->CR &= ~RNG_CR_RNGEN;			//desliga o RNG
	return RNG->DR;						//retorna o número aleatório
}


#endif /* UTILITY_H_ */
