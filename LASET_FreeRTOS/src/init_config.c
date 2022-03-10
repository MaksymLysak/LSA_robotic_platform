
#include "init_config.h"
#include "stm32f10x_it.h"

#define CNT1_INIT 0                  /* The initial value of the counter */
#define CNT2_INIT 0                  /* The initial value of the counter */

// PWM Motores
uint16_t IN1A = 0;
uint16_t IN2A = 0;
uint16_t IN1B = 0;
uint16_t IN2B = 0;

void prvInitial_configuration(void){
	prvSetupRCC();
	prvSetupGPIO();
	prvTIMERs();
	prvSetupUSART2();
	pvrIntrp();
}


 void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
    ErrorStatus HSEStartUpStatus;

   RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON);
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);

        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );

        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/

 void prvSetupGPIO( void )
{

    GPIO_InitTypeDef GPIO_InitStructure;

    // GPIO Encoder (B)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 |GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Pinos Motor (A) IN1A | IN2A
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );

	// GPIO Encoder (A)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// GPIO Motor (B) IN1B | IN2B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*-----------------------------------------------------------*/

 void prvTIMERs( void )
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	// Configuração TIM4 Enconders
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_DeInit(TIM4);

    TIM_TimeBaseStructure.TIM_Period = 62720; //auto-reload 0 até 65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //prescaler de 0 até 65535
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICFilter = 0;   /* Filter parameters of input channel */
	TIM_ICInit(TIM4, &TIM_ICInitStruct); /* Input channel initialization */
	TIM_SetCounter(TIM4, CNT1_INIT);      /*CNT Set initial value */
	TIM_ClearFlag(TIM4,TIM_IT_Update);   /* The interrupt sign is clear 0*/
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); /* Interrupt enable */
	TIM_Cmd(TIM4,ENABLE);                /* Can make CR register */

	// Configuração TIM2 Enconders
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_DeInit(TIM2);

    TIM_TimeBaseStructure.TIM_Period = 62720; //auto-reload 0 até 65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //prescaler de 0 até 65535
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICFilter = 0;   /* Filter parameters of input channel */
	TIM_ICInit(TIM2, &TIM_ICInitStruct); /* Input channel initialization */
	TIM_SetCounter(TIM2, CNT2_INIT);      /*CNT Set initial value */
	TIM_ClearFlag(TIM2,TIM_IT_Update);   /* The interrupt sign is clear 0*/
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); /* Interrupt enable */
	TIM_Cmd(TIM2,ENABLE);                /* Can make CR register */

	// Configuração TIM3 PWM Motores
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_DeInit(TIM3);

	TIM_TimeBaseStructure.TIM_Period = 100; //auto-reload 0 até 65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler = 711; //prescaler de 0 até 65535
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// IN1A
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = IN1A; //0 at´e 65535
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	// IN2A
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = IN2A; //0 at´e 65535
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	// IN1B
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = IN1B; //0 at´e 65535
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	// IN2B
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = IN2B; //0 at´e 65535
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_Cmd(TIM3, ENABLE);
}



 void pvrIntrp(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configura o Priority Group com 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Interrupção global do TIM com prioridade 0 sub-prioridade */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	// TIM3 Int
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	// Interrupt USART
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}

void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    /* Configure the USART2 */
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }
