#include "TM4C123GH6PM.h"

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 762
#define SYSCTL_RCGCGPIO_R       (*((volatile unsigned long *)0x400FE608))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define SYSCTL_RCGCUART_R       (*((volatile unsigned long *)0x400FE618))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))
#define UART0_IBRD_R            (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R            (*((volatile unsigned long *)0x4000C028))
#define UART0_CC_R              (*((volatile unsigned long *)0x4000CFC8))
#define UART0_LCRH_R            (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))
#define UART0_FR_R              (*((volatile unsigned long *)0x4000C018))
#define UART0_DR_R              (*((volatile unsigned long *)0x4000C000))
#define UART0_BUFFER_SIZE 1

#define UART1_DR_R        (*((volatile unsigned long *)0x4000D000))
#define UART1_FR_R        (*((volatile unsigned long *)0x4000D018))
#define UART1_IBRD_R      (*((volatile unsigned long *)0x4000D024))
#define UART1_FBRD_R      (*((volatile unsigned long *)0x4000D028))
#define UART1_LCRH_R      (*((volatile unsigned long *)0x4000D02C))
#define UART1_CTL_R       (*((volatile unsigned long *)0x4000D030))
#define UART1_IFLS_R      (*((volatile unsigned long *)0x4000D034))
#define UART1_IM_R        (*((volatile unsigned long *)0x4000D038))
#define UART1_RIS_R       (*((volatile unsigned long *)0x4000D03C))
#define UART1_ICR_R       (*((volatile unsigned long *)0x4000D044))
#define UART1_CC_R        (*((volatile unsigned long *)0x4000DFC8))
#define UART_IFLS_RX1_16  0x00000001
#define UART_FIFO_TX1_8 0x00
#define UART_FIFO_RX1_8 0x00



#define UART_IM_RXIM       0x00000010  // Receive Interrupt Mask
#define UART_IM_TXIM       0x00000020  // Transmit Interrupt Mask

#define UART_IFLS_RXIFLSEL_M 0x00000007  // Receive Interrupt FIFO Level Select
#define UART_IFLS_TXIFLSEL_M 0x00000038  // Transmit Interrupt FIFO Level Select

#define UART_IFLS_RX1_8    0x00000000  // Receive Interrupt FIFO Level 1/8 Full
#define UART_IFLS_TX1_8    0x00000000  // Transmit Interrupt FIFO Level 1/8 Full

int uart_start();
char getChar();
char* itoa(unsigned int val, int base);
void delayMs(unsigned int n);
void GPIO_init(void);
void UART0_init(unsigned clk,unsigned BaudRate);
void uart0_putchar(char c);
void print (const char * string);
void print_idec(int n);
void print_ihex(unsigned n);
void print_inbin(unsigned n);

int uart_start()
{	
  GPIO_init();
  UART0_init(1000000,9600 );
  delayMs(3);
}

void GPIO_init(void)
{
  SYSCTL_RCGCGPIO_R |= 0x01;       
  delayMs(1);
  GPIO_PORTA_DEN_R = 0x03;            
  GPIO_PORTA_AFSEL_R = 0x03;	        
  GPIO_PORTA_PCTL_R = 0x11;           
}

void UART0_init(unsigned clk, unsigned BaudRate)
{
  float BaudRateCarrier = (float)((float)clk / (float)(16 * BaudRate));
  int IBRD = BaudRateCarrier;
  int FBRD = (((float)(BaudRateCarrier - IBRD)) * 64 + 0.5);
  SYSCTL_RCGCUART_R |= 1;        
  delayMs(1);
  UART0_CTL_R = 0x00;	        
  UART0_IBRD_R = IBRD;	
  UART0_FBRD_R = FBRD; 	
  UART0_CC_R = 0;	
  UART0_LCRH_R = 0x70;
  
  UART0->IFLS &= ~(UART_IFLS_RXIFLSEL_M | UART_IFLS_TXIFLSEL_M);
  UART0->IFLS |= UART_IFLS_RX1_16;
  UART0->IM &= ~(UART_IM_RXIM | UART_IM_TXIM);
  UART0->IM |= UART_IM_RXIM;


  NVIC_EnableIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, 0);
  UART0_CTL_R = 0x301;
}



void uart0_putchar(char c)
{
  while ((UART0_FR_R & (1<<5)) != 0);				 
  UART0_DR_R = c;						
}

void print (const char * string)
{
  while(*string)
  uart0_putchar(*(string)++);
}

void print_idec(int n)
{
  char * str=itoa(n,10);
  print(str);
}
void print_ihex(unsigned n)
{
  print("0x");
  print(itoa(n,16));

}
void print_inbin(unsigned n)
{
  print("0b");
  print(itoa(n,2));

}

char getChar(){
	 char c;
	 if ((UART0_FR_R & (1<<4)) == 0){
	 c =  UART0->DR;
	 //uart0_putchar(c);
	return c;
	}
}

char* itoa(unsigned int val, int base){
    
    static char buf[32] = {0};
    int i = 31;
    for(; val && i ; --i, val /= base)
    {
        buf[i] = "0123456789abcdef"[val % base];
       
    }
    return &buf[i+1];
}  

void delayMs(unsigned int n)
{
    volatile unsigned int count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}



