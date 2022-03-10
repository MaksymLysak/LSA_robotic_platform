/**
  ******************************************************************************
  * @file    main.c
  * @author  3S0 FreeRTOs
  * @version V1.0
  * @date    24/10/2017
  * @brief   FreeRTOS Example project.
  ******************************************************************************
*/


/*
 *
 * LASET
 * 2021-2022
 *
 */


/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "struct_data.h"

#include "fun_for_USART.h"



/* Task priorities. */
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainEncoder_TASK_PRIORITY ( tskIDLE_PRIORITY + 1)
#define mainDistance_TASK_PRIORITY ( tskIDLE_PRIORITY + 1)

/* Max length for messages*/
#define msg_length 30



/* Configure RCC clocks */
static void prvSetupRCC( void );

/* Configure GPIO. */
static void prvSetupGPIO( void );

/* Função Interrupção */
static void pvrIntrp(void);

/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/***************************************/

/* Encoders configuration. */
static void prvTIMERs( void );




/* Function Declaration */
static void prvReadEncoders(void *pvParameters);// read encoders
static void prvControlador(void *pvParametrs); 	// control
static void prvReadUsart(void *pvParameters);	// process usart received messages
static void prvMotorDrive(void *pvParameters); // set PWM to motors
static void prvSendMessage(void *pvParameters); // send messages through USART




/* Task 1 handle variable. */
TaskHandle_t HandleTask1;
/* Task 2 handle variable. */
TaskHandle_t HandleTask2;
/* Task 3 handle variable. */
TaskHandle_t HandleTask3;
/* Task 4 handle variable. */
TaskHandle_t HandleTask4;
/* Task 5 handle variable. */
TaskHandle_t HandleTask5;
/* Task 6 handle variable. */
TaskHandle_t HandleTask6;
/* Task 7 handle variable. */
TaskHandle_t HandleTask7;
/* Task 8 handle variable. */
TaskHandle_t HandleTask8;
/* Task 9 handle variable. */
TaskHandle_t HandleTask9;
/* Task 10 handle variable. */
TaskHandle_t HandleTask10;

#define CNT1_INIT 0                  /* The initial value of the counter */
#define CNT2_INIT 0                  /* The initial value of the counter */



//Caraceterísticas da Roda
float diametroRoda = 0.07 ;//m

//Distância percorrida
double ticks2m;


// PWM Motores
uint16_t IN1A = 0;
uint16_t IN2A = 0;
uint16_t IN1B = 0;
uint16_t IN2B = 0;



/* Fila de mensagem */
QueueHandle_t xQueueTicks;
QueueHandle_t xQueueUsart;
QueueHandle_t xQueuePWM;
QueueHandle_t xQueueMessageOut;
QueueHandle_t xQueueVelCommand;


// Struct
//typedef struct
//{
//	int distB;
//	int voltasB;
//	int rpmB;
//	int velB;
//}DadosB;

int main( void )
{

	//ticks2m = (3.1415*diametroRoda)/4480;

	/*Setup the hardware, RCC, GPIO, etc...*/

	prvSetupRCC();
    prvSetupGPIO();
    prvTIMERs();
    prvSetupUSART2();
    pvrIntrp();

    /* Create queues*/
	xQueueTicks = xQueueCreate( 5, sizeof( Num_ticks ) ); /* Queue to send Encoder's ticks */
	if( xQueueTicks == NULL )
	{/* Queue was not created and must not be used. */
		return 0;
	}
	xQueueUsart = xQueueCreate( msg_length, sizeof( char ) ); /* Queue to send received character from ISR */
	if( xQueueTicks == NULL )
	{/* Queue was not created and must not be used. */
		return 0;
	}
	xQueuePWM = xQueueCreate( 2, sizeof( My_PWM ) ); /* Queue to send PWM*/
	if( xQueueTicks == NULL )
	{/* Queue was not created and must not be used. */
		return 0;
	}
	xQueueMessageOut = xQueueCreate( 10, msg_length * sizeof( char ) ); /* Queue to send PWM*/
	if( xQueueTicks == NULL )
	{/* Queue was not created and must not be used. */
		return 0;
	}
	xQueueVelCommand = xQueueCreate( 5, sizeof( uint8_t ) ); /* Queue to send velocity command to controller*/
	if( xQueueTicks == NULL )
	{/* Queue was not created and must not be used. */
		return 0;
	}





    xTaskCreate(prvReadEncoders, "EncoderReads", configMINIMAL_STACK_SIZE, NULL, mainEncoder_TASK_PRIORITY, &HandleTask1);
    xTaskCreate(prvControlador, "Controlador", configMINIMAL_STACK_SIZE, NULL, mainEncoder_TASK_PRIORITY,&HandleTask2);
    xTaskCreate(prvReadUsart, "ReadUsart", configMINIMAL_STACK_SIZE+200, NULL, mainDistance_TASK_PRIORITY, &HandleTask3);
	xTaskCreate(prvMotorDrive, "MotorDrive", configMINIMAL_STACK_SIZE, NULL, mainDistance_TASK_PRIORITY, &HandleTask4);
	xTaskCreate(prvSendMessage, "SendUsart", configMINIMAL_STACK_SIZE, NULL, mainDistance_TASK_PRIORITY, &HandleTask5);




    /* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}

/*-----------------------------------------------------------*/

static void prvReadEncoders(void *pvParameters)
{
	Num_ticks ticks;

	for(;;)
	{
		//Guardar os valores dos Ticks encoderA
		ticks.esq =TIM4->CNT;
		//Guardar os valores dos Ticks encoderB
		ticks.esq =TIM2->CNT;

		xQueueSendToBack(xQueueTicks,(void*)&ticks,(TickType_t)portMAX_DELAY);
		vTaskDelay( ( TickType_t ) 50 / portTICK_PERIOD_MS  );
	}
}

/*-----------------------------------------------------------*/

static void prvControlador(void *pvParametrs)
{
	Num_ticks ticks_old, ticks_new; ticks_new.esq=0; ticks_new.dir=0;
	Var_ticks delta_ticks;
	uint8_t command=0;

	My_PWM pwm;
	pwm.esq=0; pwm.dir=0;

	char buf[msg_length];

	for(;;){

		ticks_old = ticks_new; /* safe previous ticks*/
		xQueueReceive(xQueueTicks,(void*)&ticks_new,(TickType_t)portMAX_DELAY); /*Receive ticks from encoders*/

		delta_ticks.esq = ticks_new.esq - ticks_old.esq;
		delta_ticks.dir = ticks_new.dir - ticks_old.dir;

		sprintf(buf,"%d\n",delta_ticks.esq);
		xQueueSendToBack(xQueueMessageOut,&buf,(TickType_t)2);

		if(xQueueReceive(xQueueVelCommand,&command,(TickType_t)2)== pdPASS ){
			pwm.esq=command;
			pwm.dir=command;
		}


//		teste = (float)delta_ticks.esq*3.14*0.07/(64*70);
//		teste = 500*3.14*0.07/(64*70);

		//vTaskDelay( ( TickType_t ) 10 / portTICK_PERIOD_MS  );
		//Float2String(teste,buf);
	}
}



//TIM_SetCompare1(TIM3,30);
//TIM_SetCompare2(TIM3,0);

//TIM2->CNT = 0;

static void prvMotorDrive(void *pvParameters){
	char buf[msg_length];
	My_PWM pwm;
	pwm.esq=0; pwm.dir=0;
	uint8_t teste=100;

	for(;;){
//		xQueueReceive( xQueuePWM, &pwm,( TickType_t ) portMAX_DELAY );
		sprintf(buf,"ESQ:%d Dir:%d\n", teste,teste);//pwm.esq,pwm.dir);
		xQueueSendToBack(xQueueMessageOut,&buf,(TickType_t)2);

		TIM_SetCompare2(TIM3,teste);//pwm.dir);
		TIM_SetCompare1(TIM3,teste);//pwm.esq);

		vTaskDelay( ( TickType_t ) 100 / portTICK_PERIOD_MS  );
		//teste++;
	}
}

static void prvReadUsart(void *pvParameters)
{
	char buf[30];
	uint8_t command=0;

	uint16_t rx;
	for(;;){
//		xQueueReceive(xQueueUsart, &message,(TickType_t)portMAX_DELAY);
		if( xQueueReceive( xQueueUsart, &rx,( TickType_t ) portMAX_DELAY ) == pdPASS )
		{/* process receive message */


			if(rx ==42317){	command=255;
				xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42323){	command=0; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42321){	command=50; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42327){	command=100; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42309){	command=150; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42322){	command=200; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
		}
	}
}

static void prvSendMessage(void *pvParameters){
	char mes[msg_length];

	for(;;){
		xQueueReceive( xQueueMessageOut, &mes,( TickType_t ) portMAX_DELAY );
		prvSendMessageUSART2(mes);

	}
}

/*-----------------------------------------------------------*/





// Controlo PID

/*static void prvPID(void *pvParameters)
{
	char buff[50];
	int pwmB; // saída pwm
	int velOut = 0;
	int velB; // velocidade do motor
	int lastErroB = 0; // último erro
	int velB_target = 2; // velocidade requirida
	int erroB = 0; // erro atual
	int I_erroB = 0; // ganho integral
	int D_erroB = 0; // ganho derivativo
	// Constantes
	int Kp = 1;
	int Ki = 1;
	int Kd = 0;

	DadosB dados;

	for(;;)
	{

		xQueueReceive(xQueueDadosB,&dados,(TickType_t)portMAX_DELAY);

		dados.velB = velB;
		//P = ref - target
		erroB = velB - velB_target;
		//D = (P - erro)
		D_erroB = erroB - lastErroB;
		//I = I + P
		I_erroB = I_erroB + erroB;

		velOut = (Kp*erroB + Ki*I_erroB + Kd*D_erroB);

		velOut = velOut/100;

		if(I_erroB < -2)
		{
			I_erroB =-2;
		}

		if(I_erroB > 2)
		{
			I_erroB = 2;
		}



//		sprintf(buff,"%d  %d  %d  %d  %d \r\n",erroB,D_erroB,I_erroB,velOut,velB);
//		prvSendMessageUSART2(buff);
		lastErroB = erroB;


	}
}

static void prvReceiveUsart(void *pvParameters)
{
	char ulvar;
	char buff[75];
	char vel[10];

	int i = 0;
	int velTarget;

	DadosB dados;

	for(;;)
	{

		if(xQueueUsart != 0 && flag_rx == 0 )
		{
			xQueueReceive(xQueueUsart, &ulvar,(TickType_t)portMAX_DELAY);
		}

		if(xQueueDadosB != 0 )
		 {
			xQueueReceive( xQueueDadosB, &dados, ( TickType_t )portMAX_DELAY);
		 }

		if(ulvar == '1' && flag_rx == 0)
		{
			sprintf(buff,"Voltas: %d \n",dados.voltasB);
			prvSendMessageUSART2(buff);
		}
		if (ulvar == '2' && flag_rx == 0)
		{
			sprintf(buff,"Distancia(m): %d \n",dados.distB);
			prvSendMessageUSART2(buff);
		}
		if (ulvar == '3' && flag_rx == 0)
		{
			sprintf(buff,"Velocidade(RPM): %d \n",dados.rpmB);
			prvSendMessageUSART2(buff);
		}
		if (ulvar == '4' && flag_rx == 0)
		{
			sprintf(buff,"Velocidade(m/s): %d \n",dados.velB);
			prvSendMessageUSART2(buff);
		}
		if(ulvar == '5' && flag_rx == 0)
		{
			flag_rx = 1;
		}

		if(flag_rx == 1)
		{
			xQueueReceive(xQueueUsart, &ulvar,(TickType_t)portMAX_DELAY);
			vel[i] = ulvar;
			i++;
			if(vel[i-1] == '\r')
			{
				vel[i-1]= '\0';
				velTarget = atoi(vel);
				xQueueSendToBack( xQueueVelTarget, &velTarget, ( TickType_t ) 1);
				i = 0;
				flag_rx = 0;
			}
		}
		if (ulvar == '6' && flag_rx == 0)
		{
			sprintf(buff,"V:%d | D:%d | RPM:%d | Vel:%d \n",dados.voltasB,dados.distB,dados.rpmB,dados.velB);
			prvSendMessageUSART2(buff);
		}
	}
}


static void prvPWM(void *pvParameters)
{
	int pwm = 30;

	for(;;)
	{
		/*if(xQueueVelTarget !=0)
		{
			xQueueReceive( xQueueVelTarget, &pwm, ( TickType_t ) portMAX_DELAY);
		}
		//vTaskDelay( ( TickType_t ) 100 / portTICK_PERIOD_MS  );
		TIM_SetCompare1(TIM3,30);
		TIM_SetCompare2(TIM3,0);
	}
}




/*-----------------------------------------------------------*/

static void prvSetupRCC( void )
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

static void prvSetupGPIO( void )
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

static void prvTIMERs( void )
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



static void pvrIntrp(void)
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
