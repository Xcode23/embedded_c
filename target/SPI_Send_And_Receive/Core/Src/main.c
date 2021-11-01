#include "stm32f207xx.h"
//#include "stm32f2xx_hal.h"
#include <stdint.h>
#include <string.h>

#define RX_BUFFER_IS_EMPTY (!(spi2_struct->SR & 0x1))
#define RX_BUFFER_IS_NOT_EMPTY (spi2_struct->SR & 0x1)
#define TX_BUFFER_IS_EMPTY (spi2_struct->SR & (0x1 << 1))
#define TX_BUFFER_IS_NOT_EMPTY (!(spi2_struct->SR & (0x1 << 1)))

int main(void)
{
	 //turn on necessary clocks
	    volatile RCC_TypeDef* rcc_struct = RCC;
	    rcc_struct->AHB1ENR |= (0x1 << 1); //enable rcc clock for GPIOB
	    rcc_struct->AHB1ENR |= (0x1 << 2); //enable rcc clock for GPIOC
	    rcc_struct->APB1ENR |= (0x1 << 14); //enable rcc clock for SPI2

	    //configure GPIO ports
	    volatile GPIO_TypeDef* gpiob_struct = GPIOB;
	    volatile GPIO_TypeDef* gpioc_struct = GPIOC;
	    //PIN B12 NSS
	    gpiob_struct->MODER &= ~(0x3 << 24);//clear mode bits
	    gpiob_struct->MODER |= (0x2 << 24);//set pin to alternate function mode
	    gpiob_struct->AFR[1] &= ~(0x0F << 16);//clear alternate function mode bits
	    gpiob_struct->AFR[1] |= (0x5 << 16);//set alternate function mode to AF5
	    //turn on internal pullup
	    // gpiob_struct->PUPDR &= ~(0x3 << 24);//reset pullup register
	    // gpiob_struct->PUPDR &= ~(0x1 << 24);//set pullup register

	    //PIN B13 SCK
	    gpiob_struct->MODER &= ~(0x3 << 26);//clear mode bits
	    gpiob_struct->MODER |= (0x2 << 26);//set pin to alternate function mode
	    gpiob_struct->AFR[1] &= ~(0x0F << 20);//clear alternate function mode bits
	    gpiob_struct->AFR[1] |= (0x5 << 20);//set alternate function mode to AF5
	    //turn on internal pullup
	    // gpiob_struct->PUPDR &= ~(0x3 << 26);//reset pullup register
	    // gpiob_struct->PUPDR &= ~(0x1 << 26);//set pullup register

	    //PIN B15 MOSI
	    gpiob_struct->MODER &= ~(0x3 << 30);//clear mode bits
	    gpiob_struct->MODER |= (0x2 << 30);//set pin to alternate function mode
	    gpiob_struct->AFR[1] &= ~(0x0F << 28);//clear alternate function mode bits
	    gpiob_struct->AFR[1] |= (0x5 << 28);//set alternate function mode to AF5
	    //turn on internal pullup
	    // gpiob_struct->PUPDR &= ~(0x3 << 30);//reset pullup register
	    // gpiob_struct->PUPDR &= ~(0x1 << 30);//set pullup register

	    //PIN B15 MOSI
	    gpiob_struct->MODER &= ~(0x3 << 30);//clear mode bits
	    gpiob_struct->MODER |= (0x2 << 30);//set pin to alternate function mode
	    gpiob_struct->AFR[1] &= ~(0x0F << 28);//clear alternate function mode bits
	    gpiob_struct->AFR[1] |= (0x5 << 28);//set alternate function mode to AF5
	    //turn on internal pullup
	    // gpiob_struct->PUPDR &= ~(0x3 << 30);//reset pullup register
	    // gpiob_struct->PUPDR &= ~(0x1 << 30);//set pullup register

	    //PIN C2 MISO
	    gpioc_struct->MODER &= ~(0x3 << 4);//clear mode bits
	    gpioc_struct->MODER |= (0x2 << 4);//set pin to alternate function mode
	    gpioc_struct->AFR[0] &= ~(0x0F << 8);//clear alternate function mode bits
	    gpioc_struct->AFR[0] |= (0x5 << 8);//set alternate function mode to AF5
	    //turn on internal pullup
	    // gpiob_struct->PUPDR &= ~(0x3 << 30);//reset pullup register
	    // gpiob_struct->PUPDR &= ~(0x1 << 30);//set pullup register

	    //configure SPI
	    volatile SPI_TypeDef* spi2_struct = SPI2;
	    spi2_struct->CR1 &= ~(0x1);//reset CPHA
	    spi2_struct->CR1 &= ~(0x1 << 1);//reset CPOL
	    spi2_struct->CR1 |= (0x1 << 2);//set SPI mode to Master
	    //set SPI speed to 2MHz by dividing by 8
	    spi2_struct->CR1 &= ~(0x7 << 3);
	    spi2_struct->CR1 |= (0x2 << 3);
	    spi2_struct->CR1 &= ~(0x1 << 9);//reset SSM to configure MCU for Hardware SS management
	    spi2_struct->CR2 |= (0x1 << 2);//set SSOE to configure MCU to set SS to Master
	    //set SPI to full duplex
	    spi2_struct->CR1 &= ~(0x1 << 15);//reset BIDIMODE for 2-line unidirectional
	    spi2_struct->CR1 &= ~(0x1 << 10);//reset RXONLY for full duplex
	    spi2_struct->CR1 &= ~(0x1 << 11);//DFF: 8-bit dataframe

	    //enable SPI
	    spi2_struct->CR1 |= (0x1 << 6);

	    for(int i=0; i<10000000;i++);

	    volatile uint8_t size = 0;
	    volatile uint8_t useless = 0;
	    while(TX_BUFFER_IS_NOT_EMPTY);
	    spi2_struct->DR = 15;
	    volatile uint8_t _test_value = RX_BUFFER_IS_EMPTY;
	    while(RX_BUFFER_IS_EMPTY);
	    while(RX_BUFFER_IS_NOT_EMPTY)useless = spi2_struct->DR;//dummy read
	    while(TX_BUFFER_IS_NOT_EMPTY);
	    spi2_struct->DR = 0;//dummy write
	    while(RX_BUFFER_IS_EMPTY);
	    size=spi2_struct->DR;

	    //disable SPI
	    while( spi2_struct->SR & (0x1 << 7) );
	    spi2_struct->CR1 &= ~(0x1 << 6);

	    while(1);
}
