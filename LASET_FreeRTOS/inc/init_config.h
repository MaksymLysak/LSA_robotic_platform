#ifndef Init_func
#define Init_func

/* Call all functions below*/
void prvInitial_configuration(void);

/* Configure RCC clocks */
void prvSetupRCC( void );

/* Configure GPIO. */
void prvSetupGPIO( void );

/* Configure Timers */
void prvTIMERs( void );

/* Configure Interrupt */
void pvrIntrp(void);

/* Configure USART2 */
void prvSetupUSART2( void );

#endif /* Init_func */
