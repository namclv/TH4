#include "stm32f10x.h"
#include "rfid.h"
#include "stm32f10x_tim.h"

#define LED_PIN GPIO_Pin_13
#define LED_PORT GPIOC

static volatile uint32_t counter_ms = 0; 

void GPIO_Debug_Init(void);
void Debug(void);
void On(void);
void Off(void);

void Sys_Init(void);
void Sys_Run(void);


void Timer2_Init(void);
void Delay_us(uint16_t us);
void Delay_ms(uint32_t ms);
uint32_t millis(void);
void TIM2_IRQHandler(void);

//cau hinh UART


void USART1_Init(uint32_t baud_rate)
{
    RCC->APB2ENR |= (1 << 14) | (1 << 2); // Enable USART1 clock and GPIOA clock

    // Configure PA9 as TX
    GPIOA->CRH &= ~(unsigned int)(0xF << 4); // Clear PA9 configuration bits
    GPIOA->CRH |= (0xB << 4);  // Set PA9 as Alternate function push-pull, 50 MHz

    // Configure PA10 as RX
    GPIOA->CRH &= ~(unsigned int)(0xF << 8); // Clear PA10 configuration bits
    GPIOA->CRH |= (0x4 << 8);  // Set PA10 as Input floating

    USART1->BRR = (unsigned short)(72000000 / baud_rate );// Set baud rate
    USART1->CR1 |= ( 1 << 2) | (1 << 3) | (1 << 13) | (1 << 5); // Enable USART1, transmitter, receiver, and RX interrupt
}

void USART1_Send_Char(char chr)
{
    while (!(USART1->SR & ( 1 << 7)));
    USART1->DR = chr;
}

void USART1_Send_String(char* str)
{
    while(*str) {
        while( !(USART1->SR & ( 1 << 7)));
        USART1->DR = *str++;
    }
}

void USART1_Send_Data(uint8_t* data, uint8_t length)
{
    for (int i = 0; i < length; i++) {
        while( !(USART1->SR & ( 1 << 7)));
        USART1->DR = data[i];
    }
}

void USART1_Send_Number(int16_t num)
{
    if (num < 0) {
        USART1_Send_Char('-');
        num = -num;
    }
    uint8_t length = 0;
    uint8_t temp[10];
    if (num == 0) {
        USART1_Send_Char('0');
        return;
    } else {
        while (num != 0) {
            uint8_t value = num % 10;
            temp[length++] = value + '0';
            num /= 10;
        }
        for (int i = length - 1; i >= 0; i--) {
            USART1_Send_Char(temp[i]);
        }
    }
}

void USART1_Send_Float(float num)
{
    if (num < 0) {
        USART1_Send_Char('-');
        num = -num;
    }
    int16_t integer = (int16_t)num;
    float decimal = num - integer;
    USART1_Send_Number(integer);
    USART1_Send_Char('.');
    decimal *= 1000;
    USART1_Send_Number((int16_t)decimal);
}

void USART1_Send_Hex(uint8_t num)
{
    uint8_t temp;
    temp = num >> 4;
    if(temp > 9) {
        temp += 0x37;
    } else {
        temp += 0x30;
    }
    USART1_Send_Char(temp);
    temp = num & 0x0F;
    if(temp > 9) {
        temp += 0x37;
    } else {
        temp += 0x30;
    }
    USART1_Send_Char(temp);
}

//cau hinh GPIO
void GPIO_Debug_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void Debug(void)
{
    LED_PORT->ODR ^= LED_PIN;
    Delay_ms(1000);
}


void Off (void)
{
    LED_PORT->ODR &= ~(unsigned)LED_PIN;
}

void On (void)
{
    LED_PORT->ODR |= LED_PIN;
}







//cau hinh sys
void Sys_Init(void)
{
    // Initialize Timer2
    Timer2_Init();
    // Initialize GPIO
    GPIO_Debug_Init();
    // Initialize USART at 9600 baud
    USART1_Init(9600);
    // Add a startup delay
    Delay_ms(200);
    // Initialize RFID module
    RFID_Init();
    USART1_Send_String("RFID Reader Initialized\r\n");
    Off();
}

void Sys_Run(void)
{
    RFID_Debug_ReadCard();
    On();
		Delay_ms(1000);
    Off();
		Debug();	
}

//cau hinh timer
void Timer2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 72 - 1;       // Prescaler: 1 MHz (1 tick = 1 µs)
    TIM2->ARR = 1000 - 1;     // Auto-reload: 1000 µs = 1 ms

    TIM2->DIER |= TIM_DIER_UIE;

    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 1); 
    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        counter_ms++;           
    }
}

uint32_t millis(void)
{
    return counter_ms; 
}

void Delay_ms(uint32_t ms)
{
    uint32_t start_time = millis();
    while ((millis() - start_time) < ms); 
}

void Delay_us(uint16_t us)
{
    TIM2->CNT = 0; // Reset counter
    while (TIM2->CNT < us);
}






int main(void)
{
	Sys_Init();
  while(1)
  {
    Sys_Run();
  }
}
