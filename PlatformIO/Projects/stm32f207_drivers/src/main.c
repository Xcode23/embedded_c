#include "stm32f207xx.h"
#include <stdint.h>
#include <string.h>

#define RX_BUFFER_IS_EMPTY (!(spi2_struct->SR & 0x1))
#define RX_BUFFER_IS_NOT_EMPTY (spi2_struct->SR & 0x1)
#define TX_BUFFER_IS_EMPTY (spi2_struct->SR & (0x1 << 1))
#define TX_BUFFER_IS_NOT_EMPTY (!(spi2_struct->SR & (0x1 << 1)))
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000


void spi_send_and_receive_message(void);
void spi_send_message(void);
void init_spi(void);
void i2c_send_message(void);
void init_i2c(uint8_t address);
void i2c_send_data(uint8_t slave_address, uint8_t* data_buffer, int length);
void i2c_receive_data(uint8_t slave_address, uint8_t* data_buffer, int length);
void init_usart(void);
void usart_send_data(uint8_t* data_buffer, size_t length);
void usart_receive_data(uint8_t* data_buffer, size_t length);
void usart_send_receive_message(void);



int main(void) {
    //toggle_led_with_button();
    //spi_send_and_receive_message();
    usart_send_receive_message();
}

void init_usart(void){
    //turn on necessary clocks
    volatile RCC_TypeDef* rcc_struct = RCC;
    rcc_struct->AHB1ENR |= (0x1 << 3); //enable rcc clock for GPIOD
    rcc_struct->AHB1ENR |= (0x1 << 2); //enable rcc clock for GPIOC
    rcc_struct->APB1ENR |= (0x1 << 17); //enable rcc clock for USART2

    volatile GPIO_TypeDef* gpiod_struct = GPIOD;
    //PD3 AF7 USART2_CTS
    gpiod_struct->MODER &= ~(0x3 << 6);//clear mode bits
    gpiod_struct->MODER |= (0x2 << 6);//set pin to alternate function mode
    gpiod_struct->PUPDR &= ~(0x3 << 6);//clear pullup/down bits
    gpiod_struct->PUPDR |= (0x1 << 6);//turn on internal pull up resistor
    gpiod_struct->AFR[0] &= ~(0x0F << 12);//clear alternate function mode bits
    gpiod_struct->AFR[0] |= (0x7 << 12);//set alternate function mode to AF7
    //PD4 AF7 USART2_RTS
    gpiod_struct->MODER &= ~(0x3 << 8);//clear mode bits
    gpiod_struct->MODER |= (0x2 << 8);//set pin to alternate function mode
    gpiod_struct->PUPDR &= ~(0x3 << 8);//clear pullup/down bits
    gpiod_struct->PUPDR |= (0x1 << 8);//turn on internal pull up resistor
    gpiod_struct->AFR[0] &= ~(0x0F << 16);//clear alternate function mode bits
    gpiod_struct->AFR[0] |= (0x7 << 16);//set alternate function mode to AF7
    //PD5 AF7 USART2_TX
    gpiod_struct->MODER &= ~(0x3 << 10);//clear mode bits
    gpiod_struct->MODER |= (0x2 << 10);//set pin to alternate function mode
    gpiod_struct->PUPDR &= ~(0x3 << 10);//clear pullup/down bits
    gpiod_struct->PUPDR |= (0x1 << 10);//turn on internal pull up resistor
    gpiod_struct->AFR[0] &= ~(0x0F << 20);//clear alternate function mode bits
    gpiod_struct->AFR[0] |= (0x7 << 20);//set alternate function mode to AF7
    //PD6 AF7 USART2_RX
    gpiod_struct->MODER &= ~(0x3 << 12);//clear mode bits
    gpiod_struct->MODER |= (0x2 << 12);//set pin to alternate function mode
    gpiod_struct->PUPDR &= ~(0x3 << 12);//clear pullup/down bits
    gpiod_struct->PUPDR |= (0x1 << 12);//turn on internal pull up resistor
    gpiod_struct->AFR[0] &= ~(0x0F << 24);//clear alternate function mode bits
    gpiod_struct->AFR[0] |= (0x7 << 24);//set alternate function mode to AF7

    //configure USART
    volatile USART_TypeDef* usart2_struct = USART2;


	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	usart2_struct->CR1 |= (0x1 << 2);//USART_CR1_RE
	usart2_struct->CR1 |= (0x1 << 3);//USART_CR1_TE

    //Implement the code to configure the Word length configuration item
	//USART_CR1_M


    //Configuration of parity control bit fields
	//USART_CR1_PCE
    //USART_CR1_PS
    
	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	//USART_CR2_STOP



	//Implement the code to enable Hardware flow control
    //USART_CR3_CTSE
	//USART_CR3_RTSE


	//Implement the code to configure the baud rate of 115200 bps
    uint32_t usartdiv = ((25 * 16000000) / (4 * 115200));
    uint32_t M_part = usartdiv/100;
    uint32_t F_part = (usartdiv - (M_part * 100));
    F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
    uint32_t register_value = 0;
    register_value |= (M_part << 4);
    register_value |= F_part;
    usart2_struct->BRR = register_value;

}

void usart_send_data(uint8_t* data_buffer, size_t length){

    volatile USART_TypeDef* usart2_struct = USART2;

    //Loop over bytes to be transfered
	for(uint32_t i = 0 ; i < length; i++)
	{
		//Wait until TXE flag is set in the SR
		while(! (usart2_struct->SR & (0x1 << 7)));

		//This is 8bit data transfer
		usart2_struct->DR = *data_buffer;

		//Increment the buffer address
		data_buffer++;
	}

	//wait until tranfer is complete USART_SR_TC
	while( ! (usart2_struct->SR & (0x1 << 6)));
}

void usart_receive_data(uint8_t* data_buffer, size_t length){

    volatile USART_TypeDef* usart2_struct = USART2;

	for(uint32_t i = 0 ; i < length; i++)
	{
		//Wait until RXNE flag is set in the SR
		while(! (usart2_struct->SR & (0x1 << 5)));

		
		//We are going to receive 8bit data in a frame
		//No parity is used , so all 8bits will be of user data

		//read 8 bits from DR
		*data_buffer = (uint8_t) (usart2_struct->DR  & (uint8_t)0xFF);

		//Now , increment the pRxBuffer
		data_buffer++;
	}
}

void usart_send_receive_message(void){
    init_usart();
    volatile GPIO_TypeDef* gpioc_struct = GPIOC;

    //PIN C13 User Button
    gpioc_struct->MODER &= ~(0x3 << 26);//set mode to input

    volatile USART_TypeDef* usart2_struct = USART2; 
    //Enable usart
    usart2_struct->CR1 |= (0x1 << 13);//USART_PE

    while(1){
        if(gpioc_struct->IDR & (0x1 << 13)){
            for(uint32_t i = 0; i<500000; i++);
            char message[]="This is a String";
            char message2[64];
            for(int i = 0; i< strlen(message)-1; i++){
                usart_send_data((uint8_t*)(message+i), 1);
                usart_receive_data((uint8_t*)(message2+i), 1);
            }
            message2[strlen(message)]=0;
        }
    }
}

void i2c_send_message(void){
    init_i2c(0x61);
    volatile GPIO_TypeDef* gpioc_struct = GPIOC;

    //PIN C13 User Button
    gpioc_struct->MODER &= ~(0x3 << 26);//set mode to input

    //enable I2C
    volatile I2C_TypeDef* i2c1_struct = I2C1;
    i2c1_struct->CR1 |= (0x1 << 0);
    //turn on ACK
    i2c1_struct->CR1 |= (0x1 << 10);

    while(1){
        if(gpioc_struct->IDR & (0x1 << 13)){
            for(uint32_t i = 0; i<500000; i++);
            uint8_t message[64];
            message[0]=0x51;
            i2c_send_data(0x68, message, 1);
            i2c_receive_data(0x68, message, 1);
            uint8_t temp = message[0];
            message[0]=0x52;
            i2c_send_data(0x68, message, 1);
            i2c_receive_data(0x68, message, temp);
            i2c1_struct->CR1 |= (0x1 << 9);
        }
    }
}

void init_i2c(uint8_t address){
    //turn on necessary clocks
    volatile RCC_TypeDef* rcc_struct = RCC;
    rcc_struct->AHB1ENR |= (0x1 << 1); //enable rcc clock for GPIOB
    rcc_struct->AHB1ENR |= (0x1 << 2); //enable rcc clock for GPIOC
    rcc_struct->APB1ENR |= (0x1 << 21); //enable rcc clock for I2C1

    //configure GPIO ports
    volatile GPIO_TypeDef* gpiob_struct = GPIOB;
    
    //PIN B8 SCL
    gpiob_struct->MODER &= ~(0x3 << 16);//clear mode bits
    gpiob_struct->MODER |= (0x2 << 16);//set pin to alternate function mode
    gpiob_struct->OTYPER |= (0x1 << 8);//set pin to open drain mode
    gpiob_struct->PUPDR &= ~(0x3 << 16);//clear pullup/down bits
    gpiob_struct->PUPDR |= (0x1 << 16);//turn on internal pull up resistor
    gpiob_struct->AFR[1] &= ~(0x0F << 0);//clear alternate function mode bits
    gpiob_struct->AFR[1] |= (0x4 << 0);//set alternate function mode to AF4

    //PIN B9 SDA
    gpiob_struct->MODER &= ~(0x3 << 18);//clear mode bits
    gpiob_struct->MODER |= (0x2 << 18);//set pin to alternate function mode
    gpiob_struct->OTYPER |= (0x1 << 9);//set pin to open drain mode
    gpiob_struct->PUPDR &= ~(0x3 << 18);//clear pullup/down bits
    gpiob_struct->PUPDR |= (0x1 << 18);//turn on internal pull up resistor
    gpiob_struct->AFR[1] &= ~(0x0F << 4);//clear alternate function mode bits
    gpiob_struct->AFR[1] |= (0x4 << 4);//set alternate function mode to AF4

    //configure I2C
    volatile I2C_TypeDef* i2c1_struct = I2C1;
    //turn on ACK
    i2c1_struct->CR1 |= (0x1 << 10);
    //configure i2c for system clock frequency
    i2c1_struct->CR2 |= 16; //the system clock is 16 MHz= HSI(16MHz) AHB1 and APB1 (x/1)
    //set device address
    i2c1_struct->OAR1 |= (address << 1);
    //this bit needs to be set because reasons
    i2c1_struct->OAR1 |= (0x1 << 14);
    //configure CCR
    uint16_t ccr_value = 0;
	uint32_t speed = I2C_SCL_SPEED_SM;
	if(speed <= I2C_SCL_SPEED_SM)
	{
        //mode is standard mode
		i2c1_struct->CCR |= ( 0 << 15);
		//mode is standard mode
		ccr_value = (16000000 / ( 2 * speed ) );
		i2c1_struct->CCR |= ccr_value;
	}else
	{
		//mode is fast mode
		i2c1_struct->CCR  |= ( 1 << 15);
		//i2c1_struct->CCR  |= (dutycycle << 14); // duty cycle
		if(i2c1_struct->CCR & (0x1 << 14))
		{
			ccr_value = (16000000 / ( 3 * speed ) );
		}else
		{
			ccr_value = (16000000 / ( 25 * speed ) );
		}
		i2c1_struct->CCR |= ccr_value ;
	}
    //configure TRISE
    if(speed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		i2c1_struct->TRISE |= (16000000 /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
        uint64_t temp = ( (16000000U * 300U) / 1000000000U ) + 1;
		i2c1_struct->TRISE |= temp;

	}
}

void i2c_send_data(uint8_t slave_address, uint8_t* data_buffer, int length){
    volatile I2C_TypeDef* i2c1_struct = I2C1;

    // 1. Generate the START condition
    i2c1_struct->CR1 |= (0x1 << 8);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while(!(i2c1_struct->SR1 & (0x1 << 0)));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
    slave_address = slave_address << 1;
    slave_address &= ~(0x1 << 0);
    i2c1_struct->DR = slave_address;

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
    while( !(i2c1_struct->SR1 & (0x1 << 1)) );

	//5. clear the ADDR flag according to its software sequence
    //ADDR is cleared by reading SR1 and SR2
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
    uint8_t dummy_read = i2c1_struct->SR1;
    dummy_read = i2c1_struct->SR2;
    (void)dummy_read; 

	//6. send the data until len becomes 0

    while(length > 0){
        while( !(i2c1_struct->SR1 & (0x1 << 7)) );//wait until TX is empty
        i2c1_struct->DR = *data_buffer;
        data_buffer++;
        length--;
    }

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
    while( !(i2c1_struct->SR1 & (0x1 << 7)) );//wait until TX is empty
    while( !(i2c1_struct->SR1 & (0x1 << 2)) );//wait until BTF is 1

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
    //i2c1_struct->CR1 |= (0x1 << 9);

}

void i2c_receive_data(uint8_t slave_address, uint8_t* data_buffer, int length){
    volatile I2C_TypeDef* i2c1_struct = I2C1;

    //1. Generate the START condition
    i2c1_struct->CR1 |= (0x1 << 8);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while(!(i2c1_struct->SR1 & (0x1 << 0)));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
    slave_address = slave_address << 1;
    slave_address |= (0x1 << 0);
    i2c1_struct->DR = slave_address;

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
    while(!(i2c1_struct->SR1 & (0x1 << 1)));


    //procedure to read only 1 byte from slave
    if(length == 1){
		//Disable Acking
        i2c1_struct->CR1 &= ~(0x1 << 10);

		//clear the ADDR flag
        uint8_t dummy_read = i2c1_struct->SR1;
        dummy_read = i2c1_struct->SR2;
        (void)dummy_read;

		//wait until  RXNE becomes 1
        while( !(i2c1_struct->SR1 & (0x1 << 6)) );

		//generate STOP condition
        //i2c1_struct->CR1 |= (0x1 << 9);

		//read data in to buffer
        *data_buffer=i2c1_struct->DR;
    }


    //procedure to read data from slave when Len > 1
	if(length > 1)
	{
		//clear the ADDR flag
        uint8_t dummy_read = i2c1_struct->SR1;
        dummy_read = i2c1_struct->SR2;
        (void)dummy_read;

		//read the data until Len becomes zero
		for ( uint32_t i = length ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
            while(!(i2c1_struct->SR1 & (0x1 << 6)));

		    if(i == 2){ //if last 2 bytes are remaining

				//Disable Acking
                i2c1_struct->CR1 &= ~(0x1 << 10);

				//generate STOP condition
                //i2c1_struct->CR1 |= (0x1 << 9);
		    }

			//read the data from data register in to buffer
            *data_buffer=i2c1_struct->DR;

			//increment the buffer address
            data_buffer++;
		}

	}

    //re-enable ACKing
    i2c1_struct->CR1 |= (0x1 << 10);
}

void spi_send_and_receive_message(void) {
    init_spi();

    volatile SPI_TypeDef* spi2_struct = SPI2;

    //enable SPI
    spi2_struct->CR1 |= (0x1 << 6);

    volatile int value = 10;

    while(TX_BUFFER_IS_NOT_EMPTY); 
    spi2_struct->DR = 43;
    while(RX_BUFFER_IS_EMPTY);
    value = spi2_struct->DR;

    while(TX_BUFFER_IS_NOT_EMPTY); 
    spi2_struct->DR = 10;
    while(RX_BUFFER_IS_EMPTY);
    value = spi2_struct->DR;

    while(TX_BUFFER_IS_NOT_EMPTY); 
    spi2_struct->DR = 10;
    while(RX_BUFFER_IS_EMPTY);
    value = spi2_struct->DR;
    while(RX_BUFFER_IS_EMPTY);
    value = spi2_struct->DR;

    //disable SPI
    while( spi2_struct->SR & (0x1 << 7) );
    spi2_struct->CR1 &= ~(0x1 << 6);

    while(1);
}

void init_spi(void){
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

}

void spi_send_message(void) {
    //turn on necessary clocks
    volatile RCC_TypeDef* rcc_struct = RCC;
    rcc_struct->AHB1ENR |= (0x1 << 1); //enable rcc clock for GPIOB
    //rcc_struct->AHB1ENR |= (0x1 << 2); //enable rcc clock for GPIOC
    rcc_struct->APB1ENR |= (0x1 << 14); //enable rcc clock for SPI2

    //configure GPIO ports
    volatile GPIO_TypeDef* gpiob_struct = GPIOB;
    //GPIO_TypeDef* gpioc_struct = GPIOC;
    //PIN 12
    gpiob_struct->MODER &= ~(0x3 << 24);//clear mode bits
    gpiob_struct->MODER |= (0x2 << 24);//set pin to alternate function mode
    gpiob_struct->AFR[1] &= ~(0x0F << 16);//clear alternate function mode bits
    gpiob_struct->AFR[1] |= (0x5 << 16);//set alternate function mode to AF5
    //turn on internal pullup 
    // gpiob_struct->PUPDR &= ~(0x3 << 24);//reset pullup register
    // gpiob_struct->PUPDR &= ~(0x1 << 24);//set pullup register

    //PIN 13
    gpiob_struct->MODER &= ~(0x3 << 26);//clear mode bits
    gpiob_struct->MODER |= (0x2 << 26);//set pin to alternate function mode
    gpiob_struct->AFR[1] &= ~(0x0F << 20);//clear alternate function mode bits
    gpiob_struct->AFR[1] |= (0x5 << 20);//set alternate function mode to AF5
    //turn on internal pullup 
    // gpiob_struct->PUPDR &= ~(0x3 << 26);//reset pullup register
    // gpiob_struct->PUPDR &= ~(0x1 << 26);//set pullup register

    //PIN 15
    gpiob_struct->MODER &= ~(0x3 << 30);//clear mode bits
    gpiob_struct->MODER |= (0x2 << 30);//set pin to alternate function mode
    gpiob_struct->AFR[1] &= ~(0x0F << 28);//clear alternate function mode bits
    gpiob_struct->AFR[1] |= (0x5 << 28);//set alternate function mode to AF5
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

    char* data = "This is a message! If all goes well you will be reading it in the arduino serial monitor.";
    //send data size
    uint8_t size = strlen(data);
    while( !(spi2_struct->SR & (0x1 << 1)) );
    spi2_struct->DR = size;

    //send data
    for(uint8_t i = 0; i < size; i++){
        while( !(spi2_struct->SR & (0x1 << 1)) );
        spi2_struct->DR = data[i];
    }

    //disable SPI
    while( spi2_struct->SR & (0x1 << 7) );
    spi2_struct->CR1 &= ~(0x1 << 6);

    while(1);
}

void toggle_led_with_button(void) {
    // HAL_Init();
    // LED_Init();

    // while (1)
    // {
    //     HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    //     HAL_Delay(1000);
    // }
    GPIO_TypeDef* gpio_struct = GPIOC;
    RCC_TypeDef* rcc_struct = RCC;
    rcc_struct->AHB1ENR |= (0x1 << 2);
    //PC8
    gpio_struct->MODER &= ~(0x3 << 16);
    gpio_struct->MODER |= (0x1 << 16);
    //PC9
    gpio_struct->MODER &= ~(0x3 << 18);
    gpio_struct->ODR |= (0x1 << 8);
    while(1){
        if(gpio_struct->IDR & (0x1 << 9)){
            for(uint32_t i = 0; i<500000; i++);
            gpio_struct->ODR ^= (0x1 << 8);
        }
    }
}
