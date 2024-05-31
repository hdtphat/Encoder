#include "stm32f10x.h"                  // Device header
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
void TIM4_IRQHandler (void);
void EXTI_config (void);
void EXTI9_5_IRQHandler (void);

int main (void){
	GPIO_config();
	__enable_irq();
	TIM4_config();
	EXTI_config();
	
	while(1);
}

void GPIO_config (void){
	// enable clock to port A
	RCC->APB2ENR |= (1U<<2);
	// set PB6, PA7 as input
	GPIOA->CRL = 0x24444444;
	GPIOA->ODR = 0;
}

void TIM4_config (void){

/*
	APB1CLK = 36MHz
	As APB1_prescaler is not equal to 1
	So Clock to TIM4 = 36*2 = 72MHz
	
	TIM4_PSC = 7200
	TIM4_CLK = 72/7200 = 0.01MHz
	So each count would take 100ns
	
	TIM4_ARR = 100
	So TIM4 interupts every 100*100 = 10000ns = 10ms
*/

	// Enable TIM4 clock
	RCC->APB1ENR |= (1UL<<2);
	// Set the prescaler of timer
	TIM4->PSC = 7200-1;
	// Configure max counting value
	TIM4->ARR = 100;
	// Reset counter
	TIM4->CNT = 0;
	// Enable timer interrupt
	TIM4->DIER |= (1UL<<0);
	// Enable NVIC for timer 4
	NVIC_EnableIRQ(TIM4_IRQn);	
	// Enable counter 
	TIM4->CR1 |= (1UL<<0);
}

void EXTI_config (void){
	// Enable AFIO
	RCC->APB2ENR |= (1UL<<0);
	// Configure the EXTI Registers (port: 0-A, 1-B, 2-C, 3-D, 4-E)
	AFIO->EXTICR[0] = 0x0000; // EXTICR1: pin3 <- pin0
	AFIO->EXTICR[0] = 0x0000; // EXTICR1: pin7 <- pin4
	AFIO->EXTICR[0] = 0x0000; // EXTICR1: pin11 <- pin8
	AFIO->EXTICR[0] = 0x0000; // EXTICR1: pin15 <- pin12
	// Disable the EXTI Mask
	EXTI->IMR |= (1UL<<6) | (1UL<<5);
	// Configure the Rising Edge Trigger
	EXTI->RTSR |= (1UL<<6) | (1UL<<5);
	// Configure the Falling Edge Trigger
	EXTI->FTSR |= (1UL<<6) | (1UL<<5);
	// Set interupt priority
	NVIC_SetPriority (EXTI9_5_IRQn, 1);
	// Enable interupt
	NVIC_EnableIRQ (EXTI9_5_IRQn);
}

void TIM4_IRQHandler (void){ // TIM4 interupts every 10ms
	uint32_t temp;
	// Clear interupt flag
	TIM4->SR = 0;
	// Caculate Rotating Speed
	temp = Vel_count*(1000/10)*60; // Number of count per minute
	Vel_RPM = temp/(Encoder_CPR*4); // Rotating Speed (RPM)
	Vel_RAD = (double)(Vel_RPM)*(2*PI/60); // Angular Velocity (RAD/s)
	Vel_count = 0; // Refresh counter
}

void EXTI9_5_IRQHandler (void){ // PA5 and PA6 external interupt
	// PA5 external interupt
	if(EXTI->PR & (1UL<<5)){
		EXTI->PR |= (1UL<<5);
		// Increase Vel_count
		Vel_count++;
		// Store previous state for later comparing
		Pos_pre_state = Pos_cur_state;
		// Get current state of encoder
		Pos_cur_state = ((GPIOA->IDR)&(3UL<<5))>>5;
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
	// PA6 external interupt
	if(EXTI->PR & (1UL<<6)){
		EXTI->PR |= (1UL<<6);
		// Increase Vel_count
		Vel_count++;
		// Store previous state for later comparing
		Pos_pre_state = Pos_cur_state;
		// Get current state of encoder
		Pos_cur_state = ((GPIOA->IDR)&(3UL<<5))>>5;
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
