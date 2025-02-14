#include "stm32h743xx.h"
#include "drv_LED.hpp"
#include "Basic.hpp"
#include "AC_Math.hpp"
#include "ControlSystem.hpp"

/*
	LED璋冧寒搴﹀嚱鏁?
	R銆丟銆丅锛氫寒搴︾櫨鍒嗘瘮锛?-100锛?
*/
extern float ExtLedR;
extern float ExtLedG;
extern float ExtLedB;
void set_LedBrightness( float R , float G , float B )
{
	if( B >= 0 && B <= 100 )
	{
		ExtLedR = R;
		TIM2->CCR2 = pow(B/100,3)*TIM2->ARR;
	}
	if( G >= 0 && G <= 100 )
	{
		ExtLedG = G;
		TIM2->CCR3 = pow(G/100,3)*TIM2->ARR;
	}
	if( R >= 0 && R <= 100 )
	{
		ExtLedB = B;
		TIM2->CCR4 = pow(R/100,3)*TIM2->ARR;
	}
}

/*
	铚傞福鍣ㄩ鐜囪皟鑺傚嚱鏁?
	freq:铚傞福鍣ㄩ鐜?
*/
void set_BuzzerFreq( unsigned short freq )
{
	float B = (float)TIM2->CCR2/(float)TIM2->ARR;
	float G = (float)TIM2->CCR3/(float)TIM2->ARR;
	float R = (float)TIM2->CCR4/(float)TIM2->ARR;
	
	if( freq < 200 )
		freq = 200;
	TIM2->ARR = 10e6 / freq;
	//if( TIM2->CCR1 != 0 )
		TIM2->CCR1 = TIM2->ARR / 2 * 1.0;
	
	TIM2->CCR2 = B*TIM2->ARR;
	TIM2->CCR3 = G*TIM2->ARR;
	TIM2->CCR4 = R*TIM2->ARR;
}

/*
	铚傞福鍣ㄩ福鍝嶅嚱鏁?
	on:鏄惁楦ｅ搷
*/
void set_BuzzerOnOff( bool on )
{
	bool inFlight;
	get_is_inFlight(&inFlight);
	if( on && !inFlight )
		TIM2->CCR1 = TIM2->ARR / 2 * 1.0;
	else
		TIM2->CCR1 = 0;
}

void init_drv_LED(void)
{
	/*
		Buzzer(TIM2_CH1) PA0
		LED_B(TIM2_CH2) PA1
		LED_G(TIM2_CH3) PA2
		LED_R(TIM2_CH4) PA3
	*/
	
	//寮€鍚疓PIOA澶栬鏃堕挓
	RCC->AHB4ENR|=(1<<0);
	os_delay(1e-2);
  
	//閰嶇疆寮曡剼澶嶇敤妯″紡
	set_register( GPIOA->MODER , 0b10 , 0 , 2 );	//PA0
	set_register( GPIOA->MODER , 0b10 , 2 , 2 );	//PA1
	set_register( GPIOA->MODER , 0b10 , 4 , 2 );	//PA2
	set_register( GPIOA->MODER , 0b10 , 6 , 2 );	//PA3
	
	//路盲脙霉脝梅脥脝脥矛拢卢LED驴陋脗漏脡脧脌颅
	set_register( GPIOA->OTYPER, 0, 0, 1 );//PA0
	set_register( GPIOA->OTYPER, 0, 1, 1 );//PA1
	set_register( GPIOA->OTYPER, 0, 2, 1 );//PA2
	set_register( GPIOA->OTYPER, 0, 3, 1 );//PA3
//	set_register( GPIOA->PUPDR , 1 , 2 , 2 );	//PA1
//	set_register( GPIOA->PUPDR , 1 , 4 , 2 );	//PA2
//	set_register( GPIOA->PUPDR , 1 , 6 , 2 );	//PA3

//	set_register( GPIOA->PUPDR, 1 , 2*0 , 2  );//PA0	
	
	//閰嶇疆寮曡剼TIM2澶嶇敤
	set_register( GPIOA->AFR[0] , 1 , 0 , 4 );	//PA0
	set_register( GPIOA->AFR[0] , 1 , 4 , 4 );	//PA1
	set_register( GPIOA->AFR[0] , 1 , 8 , 4 );	//PA2
	set_register( GPIOA->AFR[0] , 1 , 12 , 4 );	//PA3
	
	//寮€鍚疶IM2澶栬鏃堕挓
	RCC->APB1LENR|=(1<<0);
	os_delay(1e-2);

	//璁℃暟鍣ㄩ鐜?0mhz
	TIM2->PSC = (APB1TIMERCLK / 10000000) - 1;
	
	//璁惧畾璁℃暟鍣ㄩ噸瑁呰浇鍊?
	TIM2->ARR = 10e6 / 1000;
	
	//閰嶇疆PWM1妯″紡
	TIM2->CCMR1 = (0b111<<12)|(1<<11)|(0<<8) | (0b110<<4)|(1<<3)|(0<<0);
	TIM2->CCMR2 = (0b111<<12)|(1<<11)|(0<<8) | (0b111<<4)|(1<<3)|(0<<0);
	
	//澶嶄綅杈撳嚭
	TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM2->CCR4 = 0;
	
	//寮€鍚畾鏃跺櫒
	TIM2->CCER = (1<<12) | (1<<8) | (1<<4) | (1<<0)   |   (1<<13) | (1<<9) | (1<<5);;
	TIM2->CR1 = (1<<7)|(1<<0);
}