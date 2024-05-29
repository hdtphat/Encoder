#include "stm32f4xx.h"                  // Device header
#include "System_Clock.h"
#include <stdint.h>

/*
AHB = HCLK = FCLK 168MHz														
PCLK1 = APB1 = 42MHz (max)												
PCLK2 = APB2 = 84MHz (max)	
*/

void SystemClock_config (void){
	//	ENABLE HSE and wait for the HSE to become Ready		
	RCC->CR |= (1U<<16); 
	while( !(RCC->CR & (1U<<17)) ){} 
	//	2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR	
	RCC->APB1ENR |= (1U<<28);					
	PWR->CR |= (3U<<14); // VOS = 11 -> Scale 1 mode (STM32F42xx and STM32F43xx)
	//	3. Configure the FLASH PREFETCH and the LATENCY Related Settings	
	FLASH->ACR |= (1U<<9); // ICEN = 1 -> Instruction cache is enabled (STM32F42xx and STM32F43xx)
	FLASH->ACR |= (1U<<10); // DCEN = 1 -> Data cache is enabled (STM32F42xx and STM32F43xx)
	FLASH->ACR |= (1U<<8); // PRFTEN = 1 -> Prefetch is enabled (STM32F42xx and STM32F43xx)
	FLASH->ACR |= (5U<<0);
	//	4. Configure the PRESCALARS HCLK, PCLK1, PCLK2	
	RCC->CFGR &= 0xFF0F; // AHB prescaler = 1
	RCC->CFGR |= (5U<<10); // APB1 prescaler = 4
	RCC->CFGR |= (4U<<13); // APB2 prescaler = 2
	//	5. Configure the MAIN PLL			
	RCC->PLLCFGR &= 0xFFFC0000; // Clear bit0-17
	RCC->PLLCFGR |= (4U<<0);  // Dividsion factor PLLM = 4
	RCC->PLLCFGR |= (168U<<6); // Multiplication factor PPN = 168
	RCC->PLLCFGR &= ~(3U<<16); // Dividsion factor PLLP = 2
	RCC->PLLCFGR |= (1U<<22); // Select HSE clock as PLL clock entry
	//	6. Enable the PLL and wait for it to become ready		
	RCC->CR |= (1U<<24);					
	while( !(RCC->CR & (1U<<25)) ){}				
	//	7. Select the Clock Source and wait for it to be set	
	RCC->CFGR |= (2U<<0);		
	while( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL ){}
}
