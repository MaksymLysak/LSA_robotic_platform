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

#include "init_config.h"

#include "fun_for_USART.h"

#include "odometry_msg.h"

#include <math.h>


/* Task priorities. */
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainEncoder_TASK_PRIORITY ( tskIDLE_PRIORITY + 1)
#define mainDistance_TASK_PRIORITY ( tskIDLE_PRIORITY + 1)

/* Max length for messages*/
#define msg_length 30

#define wheelDiameter 0.07
#define baseDist 0.189
#define tickRevolution 64
#define gearBox 70
#define ticks2m(ticks) ((ticks*wheelDiameter*M_PI)/(tickRevolution*gearBox))



/* Function Declaration */
static void prvReadEncoders(void *pvParameters);// read encoders
static void prvControlador(void *pvParametrs); 	// control
static void prvReadUsart(void *pvParameters);	// process usart received messages
static void prvMotorDrive(void *pvParameters); // set PWM to motors
static void prvSendMessage(void *pvParameters); // send messages through USART
static void prvOdometryTrack(void *pvParameters); // Make track of position and orientation



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



/* Fila de mensagem */
QueueHandle_t xQueueTicks;
QueueHandle_t xQueueUsart;
QueueHandle_t xQueuePWM;
QueueHandle_t xQueueMessageOut;
QueueHandle_t xQueueVelCommand;

/* Semaphers*/
SemaphoreHandle_t xSemaphore_Ticks;

/*-----------------------------------------------------------*/

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
	prvInitial_configuration();

    /* Create queues*/
	xQueueTicks 	 = xQueueCreate( 5, 		sizeof( Var_ticks )		  ); /* Queue to send Encoder's ticks */
	xQueueUsart 	 = xQueueCreate( msg_length,sizeof( char ) 			  ); /* Queue to send received character from ISR */
	xQueuePWM 		 = xQueueCreate( 2,  		sizeof( My_PWM ) 		  ); /* Queue to send PWM*/
	xQueueMessageOut = xQueueCreate( 10, 		sizeof( char )*60 ); /* Queue to send PWM*/
	xQueueVelCommand = xQueueCreate( 5, 		sizeof( uint8_t )      	  ); /* Queue to send velocity command to controller*/
	if(( xQueueTicks == NULL )&&( xQueueUsart == NULL )&&( xQueuePWM == NULL )&&( xQueueMessageOut == NULL )&&( xQueueVelCommand == NULL ))
	{/* Queue was not created and must not be used. */
		return 0;
	}

	/* Create semaphore */
	xSemaphore_Ticks = xSemaphoreCreateBinary();
	if( xSemaphore_Ticks == NULL )/* There was insufficient FreeRTOS heap available for the semaphore to*/
	{						/* be created successfully. */
		return 0;
	}

	/* Create Tasks */
    xTaskCreate(prvReadEncoders, "EncoderReads", configMINIMAL_STACK_SIZE, NULL, mainEncoder_TASK_PRIORITY, &HandleTask1);
    xTaskCreate(prvControlador, "Controlador", configMINIMAL_STACK_SIZE+300, NULL, mainEncoder_TASK_PRIORITY,&HandleTask2);
    xTaskCreate(prvReadUsart, "ReadUsart", configMINIMAL_STACK_SIZE+200, NULL, mainDistance_TASK_PRIORITY, &HandleTask3);
	xTaskCreate(prvMotorDrive, "MotorDrive", configMINIMAL_STACK_SIZE, NULL, mainDistance_TASK_PRIORITY, &HandleTask4);
	xTaskCreate(prvSendMessage, "SendUsart", configMINIMAL_STACK_SIZE, NULL, mainDistance_TASK_PRIORITY, &HandleTask5);
	xTaskCreate(prvOdometryTrack, "Odometry", configMINIMAL_STACK_SIZE+200, NULL, mainDistance_TASK_PRIORITY, &HandleTask6);

    /* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}

/*-----------------------------------------------------------*/

static void prvReadEncoders(void *pvParameters)
{
	Num_ticks ticks, ticksOld; ticksOld.dir=0; ticksOld.esq=0;
	Var_ticks d_ticks;

	for(;;)
	{
		ticks.dir =TIM4->CNT;	//Guardar os valores dos Ticks encoderA
		ticks.esq =TIM2->CNT;	//Guardar os valores dos Ticks encoderB

		/* Calculate ticks variation of each encoder */
		d_ticks.esq = ticks.esq - ticksOld.esq;
		if(d_ticks.esq/2000){
			d_ticks.esq=d_ticks.esq/abs(d_ticks.esq)*(62720-MAX(ticks.esq,ticksOld.esq)+MIN(ticks.esq,ticksOld.esq));
		}
		d_ticks.dir = ticks.dir - ticksOld.dir;
		if(d_ticks.dir/2000){
			d_ticks.dir=d_ticks.dir/abs(d_ticks.dir)*(62720-MAX(ticks.esq,ticksOld.dir)+MIN(ticks.esq,ticksOld.dir));
		}

		/* Send to other tasks */
		xQueueSendToBack(xQueueTicks,&d_ticks,(TickType_t)portMAX_DELAY);

		/* Store to next iteration */
		ticksOld.esq=ticks.esq;
		ticksOld.dir=ticks.dir;

		vTaskDelay( ( TickType_t ) 10 / portTICK_PERIOD_MS  );
	}
}

/*-----------------------------------------------------------*/

static void prvOdometryTrack(void *pvParameters){

	Odometry odom=init_odom_var();
	Var_ticks v_ticks;
	float d_ticks, d_theta_i, theta=0, d_x=0, d_y=0, x=0, y=0;
	char buf_x[15],buf_y[15],buf_th[15],buf[msg_length];


	for(;;){
		xQueuePeek( xQueueTicks, &v_ticks , ( TickType_t ) portMAX_DELAY );
		xSemaphoreGive( xSemaphore_Ticks );

		/* encoder variation */
		d_ticks = ticks2m(	((float)v_ticks.esq + (float)v_ticks.dir)/2	);

		d_theta_i = ticks2m(((float)v_ticks.dir - (float)v_ticks.esq)/baseDist);

		theta += d_theta_i;


		d_x = d_ticks*cos(theta);
		d_y = d_ticks*sin(theta);

		x+=d_x;
		y+=d_y;

		Float2String(x, buf_x);
		Float2String(y, buf_y);
		Float2String(theta, buf_th);
		sprintf(buf,"X:%s Y:%s th:%s\n",buf_x,buf_y,buf_th);
		xQueueSendToBack(xQueueMessageOut,&buf,(TickType_t)1);
	}
}

/*-----------------------------------------------------------*/

static void prvControlador(void *pvParametrs)
{
	Num_ticks ticks_old, ticks_new; ticks_new.esq=0; ticks_new.dir=0; ticks_old.esq=0;
	Var_ticks delta_ticks_real, delta_ticks_des;
	uint8_t command=0;
	My_PWM pwm_des, pwm_real;
	pwm_real.esq=0;

	float erro=0, pwm=0,P=0;

	char buf[msg_length], buf_p[15],buf_erro[15],buf_d[15],buf_v[15];

	int16_t veloc=0, vel_final=0;

	for(;;){

		/* Update ticks*/
		ticks_old.esq = ticks_new.esq; /* safe previous ticks*/
		xSemaphoreTake( xSemaphore_Ticks, ( TickType_t ) portMAX_DELAY );
		xQueueReceive(xQueueTicks,(void*)&delta_ticks_real,(TickType_t)portMAX_DELAY); /*Receive ticks from encoders*/
		/* ticks variation in deltaT*/

		//delta_ticks_real.dir = ticks_new.dir - ticks_old.dir;

		veloc=delta_ticks_real.esq;

		/* Receive Velocity command */
		if(xQueueReceive(xQueueVelCommand,&command,(TickType_t)1)== pdPASS ){
			//delta_ticks_des.esq=command;
			//delta_ticks_des.dir=command;
			vel_final=command;
		}

		/*--------------------------------------------------------*/
		erro = (float)(vel_final-veloc);

		if (erro!=0){
			float abs_erro=abs(erro);
			if		(abs_erro>30)	{P=7;}
			else if	(abs_erro>20)	{P=5;}
			else if	(abs_erro>15)	{P=2;}
			else if	(abs_erro>10)	{P=1;}
			else if	(abs_erro>5)	{P=0.5;}
			else if	(abs_erro>0)	{P=0.1;}
			else 					{P=0;}
			P=P*(erro/abs_erro);
			pwm=MAX(0,MIN(100,pwm+P));
		}

		pwm_real.esq=pwm;
		pwm_real.dir=pwm;


		/*--------------------------------------------------------*/


		//pwm_real.dir=MAX(0,MIN(100,pwm_real.dir));
		Float2String(pwm, buf_v);
		//Float2String(erro, buf_erro);

		//sprintf(buf,"%d	%s	%s	%s\n",veloc,buf_erro, buf_p,buf_v);
		sprintf(buf,"%s\n",buf_v);
		//xQueueSendToBack(xQueueMessageOut,&buf,(TickType_t)1);


		/* Send P*/
		xQueueSendToBack(xQueuePWM,&pwm_real,(TickType_t)portMAX_DELAY);

//		teste = (float)delta_ticks.esq*3.14*0.07/(64*70);
//		teste = 500*3.14*0.07/(64*70);

		//vTaskDelay( ( `TickType_t ) 10 / portTICK_PERIOD_MS  );
		//Float2String(teste,buf);
	}
}



//TIM_SetCompare1(TIM3,30);
//TIM_SetCompare2(TIM3,0);

//TIM2->CNT = 0;

static void prvMotorDrive(void *pvParameters){
	//char buf[msg_length];
	My_PWM pwm;
	pwm.esq=0; pwm.dir=0;

	for(;;){
		xQueueReceive( xQueuePWM, &pwm,( TickType_t ) portMAX_DELAY );
		//sprintf(buf,"ESQ:%d Dir:%d\n", pwm.esq,pwm.dir);//pwm.esq,pwm.dir);
		//xQueueSendToBack(xQueueMessageOut,&buf,(TickType_t)2);

		TIM_SetCompare2(TIM3,pwm.dir); // laranja perto do cabo
		TIM_SetCompare1(TIM3,pwm.esq); // azul  mais longe do cabo
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

			if(rx ==42317){	command=100;xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42323){	command=0;  xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42321){	command=20; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42327){	command=40; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42309){	command=60; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
			if(rx ==42322){	command=80; xQueueSendToBack(xQueueVelCommand,&command,(TickType_t)portMAX_DELAY);}
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

/*-----------------------------------------------------------*/


