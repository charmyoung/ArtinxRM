#include "main.h"

/*RELAY1---PE12---*/
/*RELAY2---PE13---*/
/*RELAY3---PE14---*/
/*RELAY4---PE15---*/
/*RELAY5---PB10---*/
/*RELAY6---PB11---*/
/*RELAY7---PB12---*/
/*RELAY8---PB13---*/
/*RELAY9---PB14---*/
/*RELAY10--PB15---*/

void Relay_Configuration(void)
{
    GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE,ENABLE);
                           
    gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
    
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE, &gpio);
    
	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &gpio);
    
    RELAY1_OFF();
    RELAY2_OFF();
    RELAY3_OFF();
    RELAY4_OFF();
    RELAY5_OFF();
    RELAY6_OFF();
    RELAY7_OFF();
    RELAY8_OFF();
    RELAY9_OFF();
    RELAY10_OFF();
}

void Relay_Test(void)
{
    static int relay_delay = 160;
    
    relay_delay -= 20;
    if(relay_delay < 25)
    {
        relay_delay = 160;
    }

    RELAY1_ON();
    delay_ms(relay_delay);
    RELAY1_OFF();
    delay_ms(relay_delay);
    
    RELAY2_ON();
    delay_ms(relay_delay);
    RELAY2_OFF();
    delay_ms(relay_delay);
    
    RELAY3_ON();
    delay_ms(relay_delay);
    RELAY3_OFF();
    delay_ms(relay_delay);
    
    RELAY4_ON();
    delay_ms(relay_delay);
    RELAY4_OFF();
    delay_ms(relay_delay);
    
    RELAY5_ON();
    delay_ms(relay_delay);
    RELAY5_OFF();
    delay_ms(relay_delay);
    
    RELAY6_ON();
    delay_ms(relay_delay);
    RELAY6_OFF();
    delay_ms(relay_delay);
    
    RELAY7_ON();
    delay_ms(relay_delay);
    RELAY7_OFF();
    delay_ms(relay_delay);
    
    RELAY8_ON();
    delay_ms(relay_delay);
    RELAY8_OFF();
    delay_ms(relay_delay);
    
    RELAY9_ON();
    delay_ms(relay_delay);
    RELAY9_OFF();
    delay_ms(relay_delay);
    
    RELAY10_ON();
    delay_ms(relay_delay);
    RELAY10_OFF();
    delay_ms(relay_delay);
}
