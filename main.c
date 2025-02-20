#include "main.h"
#include "Utility.h"
#include "AT.h"


int main(void)
{
    Utility_Init();
    USART1_Init();
    USART3_Init();

    // DMA_Init();
    // Wifi_Init("Assert", "w1f1:Assert!!");
    //send_at_cwmode(STATION_MODE, 1);
    //send_at_cwmode(STATION_MODE, 1);

    while (1)
    {
    	send_at_rst();
    	//test_Atcommand();

    	while(1);

    }
}



