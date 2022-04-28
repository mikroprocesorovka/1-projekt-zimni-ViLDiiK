


#include "stm8s.h"
#include "milis.h"
#include "stdio.h"
#include "stm8_hd44780.h"
#include "swspi.h"

void init(void);
void init_pwm(void);

void uart(void);
void uart_putchar(char data); 	// pošle jeden znak na UART
void uart_puts(char* retezec); 	// pošle celý øetìzec a UART
char text[24];

//#define FREQUENCY_M	 50
//#define CHANGE_PERIOD_M 100
#define TIMER_TOP_M (20000-1)
void motor(void);

//#define FREQUENCY_S 1000
//#define CHANGE_PERIOD_S 100
#define TIMER_TOP_S (3000-1)
void sirenka(void);

#define L_PATTERN 0b01110000 // 3x125ns (8MHZ SPI)
#define H_PATTERN 0b01111100 // 5x125ns (8MHZ SPI), first and last bit must be zero (to remain MOSI in Low between frames/b
void send_RGB(uint8_t*, uint16_t);
void RGB(void);
void RGB_init(void);
uint8_t kod_barvy = 0x02;
uint8_t kod_nebarvy = 0x00;
uint8_t color[3] = { 	// mask
0x10, 0x00, 0x00,
};
uint8_t colors[72]={ // 24 LED
0x00,0x00,0x00,
0x00,0x00,0x00,	
0x10,0x00,0x00,
0x00,0x10,0x00,
0x00,0x00,0x10,	
0x00,0x00,0x00,
0x00,0x00,0x00,
0x00,0x00,0x00,	
0x00,0x00,0x00,
0x00,0x00,0x00,
0x00,0x00,0x00,	
0x00,0x00,0x00,
0x00,0x00,0x00,
0x00,0x00,0x00,	
0x00,0x00,0x00,
0x00,0x00,0x00,
0x00,0x00,0x00,	
0x00,0x00,0x00,
0x00,0x00,0x00,
0x00,0x00,0x00,	
0x00,0x00,0x00,
0x00,0x00,0x00,
0x00,0x00,0x00,	
0x00,0x00,0x00,
};
uint16_t size;
uint32_t count = 10;
uint8_t color_mode = 0;
uint8_t i;
uint8_t j;

void process(void);

void main(void){
	init();

  while (1){
		motor(); 	// PD4
		
		sirenka();// PD0
		RGB();		// PC6
		process();
  }
}

void init(void)
{
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // 16MHz z interního RC oscilátoru
	UART1_Init(115200,UART1_WORDLENGTH_8D,UART1_STOPBITS_1,UART1_PARITY_NO,UART1_SYNCMODE_CLOCK_DISABLE,UART1_MODE_TX_ENABLE);
	UART1_Cmd(ENABLE);
	
	init_milis(); 
	init_pwm();
	RGB_init();
	
	TIM3_SetCompare2(0);
	TIM2_SetCompare1(0);
	
	size = sizeof(colors);
	
	GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_SLOW);
}

void init_pwm(void){
	GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);//sirenka
	GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);//motor

	TIM2_TimeBaseInit(TIM2_PRESCALER_16,3000-1); // sirenka
	TIM3_TimeBaseInit(TIM2_PRESCALER_16,20000-1);// motor

	TIM2_OC1Init( 					// inicializujeme kanál 1 (TM2_CH1)
		TIM2_OCMODE_PWM1, 				// režim PWM1
		TIM2_OUTPUTSTATE_ENABLE,	// Výstup povolen (TIMer ovládá pin)
		0,					
		TIM2_OCPOLARITY_HIGH			// Zátìž rozsvìcíme hodnotou HIGH 
		);

	TIM3_OC2Init(
		TIM2_OCMODE_PWM1,
		TIM2_OUTPUTSTATE_ENABLE,
		0,
		TIM2_OCPOLARITY_HIGH
		);
	
	TIM2_OC1PreloadConfig(ENABLE);
	TIM3_OC2PreloadConfig(ENABLE);
	TIM2_Cmd(ENABLE);
	TIM3_Cmd(ENABLE);
}

void RGB_init(void){
// Software slave managment (disable CS/SS input), BiDirectional-Mode release MISO pin to general purpose
SPI->CR2 |= SPI_CR2_SSM | SPI_CR2_SSI | SPI_CR2_BDM | SPI_CR2_BDOE; 
SPI->CR1 |= SPI_CR1_SPE | SPI_CR1_MSTR; // Enable SPI as master at maximum speed (F_MCU/2, there 16/2=8MHz)
}

// 	UART
// pošle jeden znak na UART
void uart_putchar(char data){
 while(UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
 UART1_SendData8(data);
}

// pošle UARTem øetìzec 
void uart_puts(char* retezec){ 
 while(*retezec){
  uart_putchar(*retezec);
  retezec++;
 }
}

void uart(void)
{
	static uint16_t last_time=0;  
	
  if(milis() - last_time >= 500){
		last_time = milis();
		
		sprintf(text,"Count: %5i  \n\r", (uint16_t)count);
		uart_puts(text);
		
		//GPIO_WriteReverse(GPIOC,GPIO_PIN_5); // kontrola
  }
}

void motor(void){
	uint16_t pulse = 0;
	/*
	static uint16_t pulse = 0; // výchozí šíøka pulzu
	static uint16_t last_time = 0;  
	static int8_t zmena = CHANGE_PERIOD_M;	//bude mít 10 a -10 a o tuhle hodnotu bude rozsvìcet a zhasínat LEDku

  if(milis() - last_time >= FREQUENCY_M){ //rychlost zmìny
		last_time = milis();
		pulse = pulse + zmena;
		if(pulse > (TIMER_TOP_M - CHANGE_PERIOD_M) || pulse < CHANGE_PERIOD_M){
			zmena = zmena*(-1);		// pokud jsme na konci rozsahu, zaèneme od zaèátku v us
		} 	
		TIM3_SetCompare2(pulse); 			// zapíšeme šíøku pulzu do timeru
	}*/
	
	pulse = (TIMER_TOP_M * count) / 24;
	TIM3_SetCompare2(pulse);
}

void sirenka(void){
	//static uint16_t pulse = 0; // výchozí šíøka pulzu
	//static uint16_t last_time = 0;  
	//static int8_t zmena = CHANGE_PERIOD_M;	//bude mít 10 a -10 a o tuhle hodnotu bude rozsvìcet a zhasínat LEDku

	if (count % 2 == 1)
	{
		TIM2_SetCompare1(TIMER_TOP_S / 2); 
	}
	else
	{
		TIM2_SetCompare1(0);
	}
	/*
  if(milis() - last_time >= FREQUENCY_M){ //rychlost zmìny
		last_time = milis();

		TIM2_SetCompare1(TIMER_TOP_M / 2); 			// zapíšeme šíøku pulzu do timeru
	}*/
}

void RGB(void)
{
	static uint16_t last_time=0; 
	static uint16_t actualization_last_time;
	
	if (milis() - last_time > 400)
	{
		last_time = milis();
		//GPIO_WriteReverse(GPIOC,GPIO_PIN_5); // kontrola
		switch (color_mode){
			case 0:
				color[0] -= 0x01;
				color[1] = 0x10 - color[0];
				if (color[0] == 0x00)
				{
					color_mode = 1;
				}
				break;
			case 1:
				color[1] -= 0x01;
				color[2] = 0x10 - color[1];
				if (color[1] == 0x00)
				{
					color_mode = 2;
				}
				break;
			case 2:
				color[2] -= 0x01;
				color[0] = 0x10 - color[2];
				if (color[2] == 0x00)
				{
					color_mode = 0;
				}
				break;
		}
	}
	
	if (milis() - actualization_last_time >= 50)
	{
		actualization_last_time = milis();
		
		for (i = 0; i < size; i++) 	// vymazání zobrazení
		{
			colors[i] = kod_nebarvy;
		}
		
		for (i = 0; i < count; i++)
		{
			for (j = 0; j < 3; j++)			// nakreslení RGB diod
			{
				colors[3 * i + j] = color[3 * i + j];
			}
		}
		
		send_RGB(colors,size);
	}
}

void send_RGB(uint8_t* data, uint16_t length){
	uint8_t mask;
	disableInterrupts();
	while(length){
		length--;
		mask=0b10000000;
		while(mask){
			while(!(SPI->SR & SPI_SR_TXE));
			if(mask & data[length]){
				SPI->DR = H_PATTERN;
			}else{
				SPI->DR = L_PATTERN;
			}
			mask = mask >> 1;
		}
	}
	enableInterrupts();
	while(SPI->SR & SPI_SR_BSY);
}

void process(void)
{
	static uint16_t last_time = 0; 
	
	if (milis() - last_time >= 500)
	{
		last_time = milis();
		
		count += 1;
		if (count > 24)
		{
			count = 0;
		}
		uart();		// USB
	}
}

// pod tímto komentáøem nic nemìòte 
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
