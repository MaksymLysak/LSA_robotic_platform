

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
/*STM32 and FreeRTOS includes*/
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* My includes */
#include "struct_data.h"

#include "init_config.h"

#include "fun_for_USART.h"

#include "odometry_msg.h"




/* Task priorities. */
#define normalTaskPriority	( tskIDLE_PRIORITY + 1)

/* Max length for messages*/
#define msg_length 120

/* Odometry parameter */
#define wheelDiameter 0.069
#define baseDist 0.189
#define tickRevolution 64
#define gearBox 70

/* Converters between ticks and meters*/
#define ticks2m(ticks) ((ticks*wheelDiameter*M_PI)/(tickRevolution*gearBox))
#define m2ticks(m) (m*tickRevolution*gearBox)/(wheelDiameter*M_PI)


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
//QueueHandle_t xQueueOdometry;

/* Semaphers*/
SemaphoreHandle_t xSemaphore_Ticks1;
SemaphoreHandle_t xSemaphore_Ticks2;

/*-----------------------------------------------------------*/

int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
	prvInitial_configuration();

    /* Create queues*/
	xQueueTicks 	 = xQueueCreate( 5, 		sizeof( Var_ticks )		  ); /* Queue to send Encoder's ticks */
	xQueueUsart 	 = xQueueCreate( msg_length,sizeof( char ) 			  ); /* Queue to send received character from ISR */
	xQueuePWM 		 = xQueueCreate( 2,  		sizeof( My_PWM ) 		  ); /* Queue to send PWM*/
	xQueueMessageOut = xQueueCreate( 10, 		sizeof( char )*msg_length ); /* Queue to send PWM*/
	xQueueVelCommand = xQueueCreate( 5, 		sizeof( uint8_t )      	  ); /* Queue to send velocity command to controller*/
//	xQueueVelCommand = xQueueCreate( 5, 		sizeof( Twist )      	  ); /* Queue to send velocity command to controller*/
//	xQueueOdometry   = xQueueCreate( 5, 		sizeof( Odometry )     	  ); /* Queue to send odometry data*/

	if(/*( xQueueOdometry == NULL )&&*/( xQueueTicks == NULL )&&( xQueueUsart == NULL )&&( xQueuePWM == NULL )&&( xQueueMessageOut == NULL )&&( xQueueVelCommand == NULL ))
	{/* Queue was not created and must not be used. */
		return 0;
	}

	/* Create semaphore */
	xSemaphore_Ticks1 = xSemaphoreCreateBinary();
	xSemaphore_Ticks2 = xSemaphoreCreateBinary();
	if(( xSemaphore_Ticks1 == NULL )&&( xSemaphore_Ticks2 == NULL ))
	{/* There was insufficient FreeRTOS heap available for the semaphore to be created successfully */
		return 0;
	}

	xSemaphoreGive( xSemaphore_Ticks1 );

	/* Create Tasks */
    xTaskCreate(prvReadEncoders, "EncoderReads",configMINIMAL_STACK_SIZE, NULL, normalTaskPriority, &HandleTask1);
    xTaskCreate(prvControlador, "Controlador", 	configMINIMAL_STACK_SIZE+300, NULL, normalTaskPriority,&HandleTask2);
    xTaskCreate(prvReadUsart, "ReadUsart", 		configMINIMAL_STACK_SIZE+200, NULL, normalTaskPriority, &HandleTask3);
	xTaskCreate(prvMotorDrive, "MotorDrive", 	configMINIMAL_STACK_SIZE, NULL, normalTaskPriority, &HandleTask4);
	xTaskCreate(prvSendMessage, "SendUsart", 	configMINIMAL_STACK_SIZE, NULL, normalTaskPriority, &HandleTask5);
	xTaskCreate(prvOdometryTrack, "Odometry", 	configMINIMAL_STACK_SIZE+200, NULL, normalTaskPriority, &HandleTask6);

    /* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}

/*-----------------------------------------------------------*/

static void prvReadEncoders(void *pvParameters)
{
	Num_ticks ticks, ticksOld;
	Var_ticks d_ticks;

	ticksOld.dir=0; ticksOld.esq=0;

	for(;;)
	{
		ticks.dir =TIM4->CNT;	/* Save value of Ticks from encoderA */
		ticks.esq =TIM2->CNT;	/* Save value of Ticks from encoderB */

		/* Calculate ticks variation of each encoder */
		d_ticks.esq = ticks.esq - ticksOld.esq;
		if(d_ticks.esq/2000){  /* Overflow correction */
			d_ticks.esq=d_ticks.esq/abs(d_ticks.esq)*(62720-MAX(ticks.esq,ticksOld.esq)+MIN(ticks.esq,ticksOld.esq));
		}
		d_ticks.dir = ticks.dir - ticksOld.dir;
		if(d_ticks.dir/2000){	/* Overflow correction */
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
	float d_s, d_theta_i, theta=0, d_x=0, d_y=0;
//	char buf_x[15],buf_y[15],buf_th[15],buf[msg_length];


	for(;;){
		/* Receive data, Give permition to next task to read and delete */
		xSemaphoreTake( xSemaphore_Ticks1, ( TickType_t ) portMAX_DELAY );
		xQueuePeek( xQueueTicks, &v_ticks , ( TickType_t ) portMAX_DELAY );
		xSemaphoreGive( xSemaphore_Ticks2 );

		/* Linear variation */
		d_s = ticks2m(	((float)v_ticks.esq + (float)v_ticks.dir)/2	);

		/* Angular variation */
		d_theta_i = ticks2m(((float)v_ticks.dir - (float)v_ticks.esq)/baseDist);

		/* Robot orientation */
		theta += d_theta_i;

		/* Linear components of motion */
		d_x = d_s*cos(theta);
		d_y = d_s*sin(theta);

		odom.pose.position.x +=d_x;
		odom.pose.position.y +=d_x;
		odom.pose.orientation.w=cos(theta * 0.5);
		odom.pose.orientation.z=sin(theta * 0.5);
		odom.twist.linear.x=d_s*100;
		odom.twist.angular.z=d_theta_i*100;

		/* Send out */
		/* xQueueSendToBack(xQueueOdometry,&odom,(TickType_t)portMAX_DELAY); */


//		float2String(odom.pose.position.x, buf_x);
//		float2String(odom.pose.position.y, buf_y);
//		float2String(theta, buf_th);
//		sprintf(buf,"X:%s Y:%s th:%s\n",buf_x,buf_y,buf_th);
//		xQueueSendToBack(xQueueMessageOut,&buf,(TickType_t)1);
	}
}

/*-----------------------------------------------------------*/

#define mDimLen 10
static void prvControlador(void *pvParametrs)
{
	Var_ticks delta_ticks_real, delta_ticks_des;
	uint8_t command=0;
	My_PWM pwm_des, pwm_real;
//	My_PWM pwm; // iniciar a zero
//	Twist cmd_vel;
//	cmd_vel.angular.z=0;
//	cmd_vel.linear.x = 0;

	pwm_real.esq=0;

	int16_t veloc=0, vel_final=0;

	float erro=0,erro_ant=0, erro_acum=0, float_pwm=0,P=0,Kp=0.02,D=0,Kd=0.14,I=0, Ki=0.0002, PID=0;

	/* debug variabels*/
	char buf_erro[mDimLen], buf_erro_ant[mDimLen], buf_P[mDimLen], buf_I[mDimLen], buf_D[mDimLen],buf_PID[mDimLen],buf_pwm[mDimLen];
	char buf[msg_length];



	for(;;){

		/* Receive ticks variation*/
		xSemaphoreTake( xSemaphore_Ticks2, ( TickType_t ) portMAX_DELAY );
		xQueueReceive(xQueueTicks,(void*)&delta_ticks_real,(TickType_t)portMAX_DELAY); /*Receive ticks from encoders*/
		xSemaphoreGive( xSemaphore_Ticks1 );


		veloc=delta_ticks_real.esq;

		/* Receive Velocity command */
		if(xQueueReceive(xQueueVelCommand,&command,(TickType_t)1)== pdPASS ){
			vel_final=command;
		}
//		if(xQueueReceive(xQueueVelCommand,&cmd_vel,(TickType_t)1)== pdPASS ){
//			delta_ticks_des.esq = m2ticks(cmd_vel.linear.x/100) - m2ticks(cmd_vel.angular.z/100)*baseDist;
//			delta_ticks_des.dir = m2ticks(cmd_vel.linear.x/100) + m2ticks(cmd_vel.angular.z/100)*baseDist;
//
//			/* Define motor direction pins */
//			pwm.en1_esq=0;
//			pwm.en2_esq=0;
//			if (delta_ticks_des.esq<0)	pwm.en2_esq=1;
//			if (delta_ticks_des.esq>0)  pwm.en1_esq=1;
//			pwm.en1_dir=0;
//			pwm.en2_dir=0;
//			if (delta_ticks_des.esq<0)	pwm.en2_esq=1;
//			if (delta_ticks_des.esq>0)	pwm.en1_esq=1;
//
//			/* Remove sign*/
//			delta_ticks_des.esq = abs(delta_ticks_des.esq);
//			delta_ticks_des.dir = abs(delta_ticks_des.dir);
//		}


		/*--- Controller ---*/
		erro_ant=erro;
		erro = ((float)vel_final-(float)veloc);

		P=erro*Kp;
		erro_acum+=erro;
		I=Ki*erro_acum;
		D=Kd*(erro-erro_ant);

		PID=MAX(-10,MIN(10,P+I+D));

		if (vel_final==0){float_pwm =0;}
		else{
			float_pwm=MAX(0,MIN(100,float_pwm+PID));
		}



		pwm_real.esq=float_pwm;
		pwm_real.dir=0;
		/*--- End controller ---*/

		/* Send PWM*/
		xQueueSendToBack(xQueuePWM,&pwm_real,(TickType_t)portMAX_DELAY);

		/* debug */
		float2String(erro, buf_erro);
		float2String(erro_ant, buf_erro_ant);
		float2String(P, buf_P);
		float2String(I, buf_I);
		float2String(D, buf_D);
		float2String(PID, buf_PID);
		float2String(float_pwm, buf_pwm);

		sprintf(buf,"V:%d  E:%s  Ea:%s  P:%s  I:%s  D:%s  PID:%s  PWM:%s\n",veloc,buf_erro,buf_erro_ant,buf_P,buf_I,buf_D,buf_PID,buf_pwm);
		xQueueSendToBack(xQueueMessageOut,&buf,(TickType_t)1);
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

		TIM_SetCompare2(TIM3,pwm.dir); // laranja perto do cabo
		TIM_SetCompare1(TIM3,pwm.esq); // azul  mais longe do cabo
	}
}

static void prvReadUsart(void *pvParameters)
{
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
		/* Receive and send the message */
		xQueueReceive( xQueueMessageOut, &mes,( TickType_t ) portMAX_DELAY );
		prvSendMessageUSART2(mes);

	}
}

/*-----------------------------------------------------------*/

