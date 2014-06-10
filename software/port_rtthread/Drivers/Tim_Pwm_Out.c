#include "tim_pwm_out.h"

uint32_t CCR1_Val = 21000;
uint32_t CCR2_Val = 21000;
uint32_t CCR3_Val = 21000;
uint32_t CCR4_Val = 21000;
#define Moto_PwmMax 999
void Tim_Pwm_Out_Init(void)
{
	
}

void Moto_PwmRflash(int32_t MOTO1_PWM,int32_t MOTO2_PWM,int32_t MOTO3_PWM,int32_t MOTO4_PWM)
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	if(MOTO1_PWM<0)	MOTO1_PWM = 0;
	if(MOTO2_PWM<0)	MOTO2_PWM = 0;
	if(MOTO3_PWM<0)	MOTO3_PWM = 0;
	if(MOTO4_PWM<0)	MOTO4_PWM = 0;
	
	 TIM5->CCR1 = 21*MOTO1_PWM+21000;
   TIM5->CCR2= 21*MOTO2_PWM+21000;
	 TIM5->CCR3= 21*MOTO3_PWM+21000;
	 TIM5->CCR4= 21*MOTO4_PWM+21000;
}
