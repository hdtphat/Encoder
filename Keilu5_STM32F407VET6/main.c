#include "stm32f4xx.h"                  // Device header
#include "System_Clock.h"
#include <stdint.h>

#define PI 3.14159265358979323846
static uint32_t Encoder_CPR = 96; // Encoder's Pulses Per Revolution (x1 mode)
static uint32_t Vel_count = 0; // Number of Pulses to calculate rotating speed every 5ms
static uint32_t Vel_RPM = 0; // Motor's Rotating Speed (round per minute)
static double Vel_RAD = 0.0; // Motor's Angular Velocity (RAD per second)
static uint32_t Pos_cur_state = 0x00; // Encoder's current state
static uint32_t Pos_pre_state = 0x00; // Encoder's previous state
static int64_t Pos_count = 0; // Number of Pulses to define position of motor

void GPIO_config (void);
void TIM4_config (void);
void TIM4_IRQHandler(void);
void EXTI_config (void);
void EXTI0_IRQHandler (void);
void EXTI1_IRQHandler (void);

int main (void){
	SystemClock_config();
	GPIO_config();
	TIM4_config();
	EXTI_config();
	
	while(1);
}

void GPIO_config (void){
	// Enable GPIAO clock
	RCC->AHB1ENR |= (1UL<<0);
	// Set PA0 and PA1 as input
	GPIOA->MODER &= ~((3UL<<0) | (3UL<<2));
	// Set PA0 and PA1 as pull-up mode
	GPIOA->PUPDR |= (1UL<<0) | (1UL<<2);
}

void TIM4_config (void){
	// Enable TIM4 clock 
	RCC->APB1ENR |= (1UL<<2);
	// Set TIM4 prescaler 
	TIM4->PSC = 840-1; // TIM4CLK = 0.1MHz, each count takes 10ns
	// Set max count 
	TIM4->ARR = 1000; // timer interupt every 10ms
	// Reset counter
	TIM4->CNT = 0;
	// Enable timer interrupt
	TIM4->DIER |= (1UL<<0);
	NVIC_EnableIRQ(TIM4_IRQn);	
	// Enable counter 
	TIM4->CR1 |= (1UL<<0);
}

void EXTI_config (void){
	// Enable SYSCFGR
	RCC->APB2ENR |= (1UL<<14);
	// Configure the EXTI Registers (port: 0-A, 1-B, 2-C, 3-D, 4-E)
	SYSCFG->EXTICR[0] = 0x0000; // EXTICR1: pin3 <- pin0
	SYSCFG->EXTICR[0] = 0x0000; // EXTICR1: pin7 <- pin4
	SYSCFG->EXTICR[0] = 0x0000; // EXTICR1: pin11 <- pin8
	SYSCFG->EXTICR[0] = 0x0000; // EXTICR1: pin15 <- pin12
	// Disable the EXTI Mask
	EXTI->IMR |= (1UL<<0) | (1UL<<1);
	// Configure the Rising Edge Trigger
	EXTI->RTSR |= (1UL<<0) | (1UL<<1);
	// Configure the Falling Edge Trigger
	EXTI->FTSR |= (1UL<<0) | (1UL<<1);
	// Set interupt priority
	NVIC_SetPriority (EXTI0_IRQn, 1);
	NVIC_SetPriority (EXTI1_IRQn, 1);
	// Enable interupt
	NVIC_EnableIRQ (EXTI0_IRQn);
	NVIC_EnableIRQ (EXTI1_IRQn);
}

void TIM4_IRQHandler(void){ // TIM4 interupt every 10ms
	uint32_t temp;
	// Clear interupt flag
	TIM4->SR = 0;
	// Caculate Rotating Speed
	temp = Vel_count*(1000/10)*60; // Number of count per minute
	Vel_RPM = temp/(Encoder_CPR*4); // Rotating Speed (RPM)
	Vel_RAD = (double)(Vel_RPM)*(2*PI/60); // Angular Velocity (RAD/s)
	Vel_count = 0; // Refresh counter
}

void EXTI0_IRQHandler (void){ // PA0 external interupt
	if (EXTI->PR & (1UL<<0)){
		// Clear interupt flag
		EXTI->PR |= (1UL<<0);  
		// Increase Vel_count
		Vel_count++;
		// Store previous state for later comparing
		Pos_pre_state = Pos_cur_state;
		// Get current state of encoder
		Pos_cur_state = (GPIOA->IDR)&(3UL<<0);
		// Define position
		switch(Pos_pre_state){
			case 0x00: // Previous state: 0b00
				if(Pos_cur_state == 0x02) Pos_count++; // Current state: 0b10
				else Pos_count--; // Current state: 0b01
				break;
			case 0x01: // Previous state: 0b01
				if(Pos_cur_state == 0x00) Pos_count++; // Current state: 0b00
				else Pos_count--; // Current state: 0b11
				break;
			case 0x02: // Previous state: 0b10
				if(Pos_cur_state == 0x03) Pos_count++; // Current state: 0b11
				else Pos_count--; // Current state: 0b00
				break;
			case 0x03: // Previous state: 0b11
				if(Pos_cur_state == 0x01) Pos_count++; // Current state: 0b01
				else Pos_count--; // Current state: 0b10
				break;
		}
	}
}

void EXTI1_IRQHandler (void){ // PA1 external interupt
	if (EXTI->PR & (1UL<<1)){
		// Clear interupt flag
		EXTI->PR |= (1UL<<1);  
		// Increase Vel_count
		Vel_count++;
		// Store previous state for later comparing
		Pos_pre_state = Pos_cur_state;
		// Get current state of encoder
		Pos_cur_state = (GPIOA->IDR)&(3UL<<0);
		// Define position
		switch(Pos_pre_state){
			case 0x00: // Previous state: 0b00
				if(Pos_cur_state == 0x02) Pos_count++; // Current state: 0b10
				else Pos_count--; // Current state: 0b01
				break;
			case 0x01: // Previous state: 0b01
				if(Pos_cur_state == 0x00) Pos_count++; // Current state: 0b00
				else Pos_count--; // Current state: 0b11
				break;
			case 0x02: // Previous state: 0b10
				if(Pos_cur_state == 0x03) Pos_count++; // Current state: 0b11
				else Pos_count--; // Current state: 0b00
				break;
			case 0x03: // Previous state: 0b11
				if(Pos_cur_state == 0x01) Pos_count++; // Current state: 0b01
				else Pos_count--; // Current state: 0b10
				break;
		}
	}
}
