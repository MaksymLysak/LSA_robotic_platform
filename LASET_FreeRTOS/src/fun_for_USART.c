//#include "fun_for_USART.h"
#include <stdio.h>
#include <string.h>
#include "stm32f10x_it.h"


 void prvSendMessageUSART2(char *message)
{
uint16_t cont_aux=0;

    while(cont_aux != strlen(message))
    {
        USART_SendData(USART2, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        cont_aux++;
    }
}

 void Float2String(float num, char* str)
{
	char *tmpSign = (num < 0) ? "-" : "";
	float tmpVal = (num < 0) ? -num : num;

	int tmpInt1 = tmpVal;                  // Get the integer (678).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	int tmpInt2 = tmpFrac * 10000;  	   // Turn into integer (123).

	// Print as parts, note that you need 0-padding for fractional bit.
	sprintf (str, "%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);

}
