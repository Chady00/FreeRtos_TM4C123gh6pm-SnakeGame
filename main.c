//MOTOR
#define SYSCTL_RCGCGPIO_R         (*((volatile unsigned long *)( 0x400FE608)))

#define GPIO_PORTA_DATA_R         (*((volatile unsigned long *)(0x400043FC )))
#define GPIO_PORTA_DIR_R          (*((volatile unsigned long *)( 0x40004400)))
#define GPIO_PORTA_DEN_R          (*((volatile unsigned long *)( 0x4000451C )))


#define PORTA_765432 0xFC 
#define PORTA_CLK_EN 0x01; 
 
 
 #define BACKWARD  0xAC //101011-00 
 #define FORWARD  0xD4  //110101-00
 #define BREAK  0xFC    //111111-00 
 #define LEFT  0xCC   //1-10-01-1-00 
 #define RIGHT  0xB4   //1-01-10-1-00 

 
 void motorDriverConfig(void); 
 void moveForward(void); 
 void moveBackward(void); 
 void moveLeft(void); 
 void moveRight(void); 
 void Stop(void); 
  
 

//SYSTICK
#define ST_CTRL_R (*((volatile unsigned long *)   0xE000E010)) 
 #define ST_RELOAD_R (*((volatile unsigned long *) 0xE000E014)) 
 #define ST_CURRENT_R (*((volatile unsigned long *)0xE000E018)) 
 
 
 #define ST_ENABLE  0x00000001 
 #define ST_INT_EN  0x00000002 
 #define ST_CLK_SRC 0x00000004 
 #define ST_COUNT   0x00010000 


 
 
 #define ST_RELOAD_VALUE 0x000EA600 
 
 
 
 
 void configureSystickTimer(void); 
#define SYSCTL_RCGCGPIO_R  *((volatile unsigned long *) 0x400FE608) 
 
 	 
 #define GPIO_PORTE_DATA_R  *((volatile unsigned long *) 0x400240E0) 
 #define GPIO_PORTE_DIR_R   *((volatile unsigned long *) 0x40024400) 
 #define GPIO_PORTE_DEN_R   *((volatile unsigned long *) 0x4002451C) 
 #define GPIO_PORTE_AFSEL_R *((volatile unsigned long *) 0x40024420) 
 #define GPIO_PORTE_PD_R    *((volatile unsigned long *) 0x40024514) 
 //For DistanceSensor PE2 for Trigger and PF4 for ECHO 
 #define GPIO_PORTE_DATA_TRIG_R *((volatile unsigned long *) 0x40024010) 
 #define GPIO_PORTE_DATA_ECHO_R *((volatile unsigned long *) 0x40024008) 
 	 
 //NVIC Registers for Enabling Interrupts on TRIGGER (E2) Pin 
 #define GPIO_PORTE_IS_R    *((volatile unsigned long *) 0x40024404) 
 #define GPIO_PORTE_IBE_R   *((volatile unsigned long *) 0x40024408) 
 #define GPIO_PORTE_IEV_R   *((volatile unsigned long *) 0x4002440C) 
 #define GPIO_PORTE_IM_R    *((volatile unsigned long *) 0x40024410) 
 #define GPIO_PORTE_ICR_R   *((volatile unsigned long *) 0x4002441C) 
 	 
 #define NVIC_EN0_R         *((volatile unsigned long *) 0xE000E100) 
 #define NVIC_PRI1_R        *((volatile unsigned long *) 0xE000E404) 
 	 
 #define NVIC_INT4_EN 0x00000010 
 #define PRILEVEL_5   0x000000A0 
 int f;
 
 

#define TM_BASE 0x40032000 
 
 
 //Clock enabling for Timer and GPIO 
 #define RCGC_TIMER_R *(volatile unsigned long *) 0x400FE604 
 
 
 #define CLOCK_GPIOF 0x00000020 
 #define CLOCK_GPIOE 0x00000010 
 #define CLOCK_GPIOB 0x00000002 
 #define SYS_CLOCK_FREQUENCY 16000000 
 
 
 //Timer configuration Registers 
 #define TRIG_ECHO_PIN_EN 0x06 
 #define TRIG_PINE2 0x04 
 #define ECHO_PINE1 0x02 
 
 
 #define PORTE_CLK_EN 0x10; 
 
 
 //Timer configuration Registers 
 #define GPTM_CONFIG_R *(volatile unsigned long *)(TM_BASE + 0x000) 
 #define GPTM_TA_MODE_R *(volatile unsigned long *)(TM_BASE + 0x004) 
 #define GPTM_CONTROL_R *(volatile unsigned long *)(TM_BASE + 0x00C) 
 #define GPTM_INT_MASK_R *(volatile unsigned long *)(TM_BASE + 0x018) 
 #define GPTM_INT_CLEAR_R *(volatile unsigned long *)(TM_BASE + 0x024) 
 #define GPTM_TA_IL_R *(volatile unsigned long *)(TM_BASE + 0x028) 
 #define GPTM_TA_MATCH_R *(volatile unsigned long *)(TM_BASE + 0x030) 
 #define GPTM_TA_PRESCALE_R *(volatile unsigned long *)(TM_BASE + 0x038) 
 #define GPTM_TA_COUNT_R *(volatile unsigned long *)(TM_BASE + 0x048) 
 	 
 // IRQ 0 to 31 Enable and Disable Registers 
 	 
 #define NVIC_EN0_R *((volatile unsigned long *) 0xE000E100) 
 #define NVIC_DIS0_R *((volatile unsigned long *) 0xE000E180) 
 
 
 //GPIO alternate function Configuration 
 #define GPIO_PORTF_AFSEL_R *((volatile unsigned long *) 0x40025420) 
 #define GPIO_PORTF_PCTL_R *((volatile unsigned long *) 0x4002552C) 
 #define GPIO_PORTF_DEN_R *((volatile unsigned long *) 0x4002551C) 
 #define GPIO_PORTF_DIR_R *((volatile unsigned long *) 0x40025400) 
 #define GPIO_PORTF_DATA_R *((volatile unsigned long *) 0x40025038) 
 
 //Timer2 A interrupt is assigned to NVIC IRQ23 
 #define NVIC_EN0_INT23 0x00800000 
 
 
 //Timer2 A bit field definitions for mode configuration 
 #define TIM_16_BIT_CONFIG 0x00000004 
 #define TIM_EDGE_TIME_MODE 0x00000004 
 #define TIM_CAPTURE_MODE 0x00000003 
 
 
 //Timer event Type bit field definitions 
 #define TIM_A_EVENT_POS_EDGE 0x00000000 
 #define TIM_A_EVENT_NEG_EDGE 0x00000004 
 #define TIM_A_EVENT_BOTH_EDGES 0x0000000C 
 #define TIM_A_ENABLE 0x00000001 
 int value1,value2;
 int c=0;
 //Timer A capture mode interrupt mask/clear 
 #define TIM_A_CAP_EVENT_IM 0x00000004 
 #define TIM_A_CAP_EVENT_IC 0x00000004 
 
 
 // Reload values for Timer A with prescale 
 #define TIM_A_INTERVAL 0x0000FFFF 
 #define TIM_A_PRESCALE 0x000000FF 
 
 
//The value required to give to the Delay function for 1sec Delay 
 #define SECOND 100000 
 
 
 //Function Prototypes 
 void Timer2A_Init(void); 
 void distanceSensorConfig(void); 
 void triggerSensor (void); 
 int getDistance(void); 
 void Delay(unsigned long value);
void brogetback(void);
void moveforward(void);
void stopmotors(void);
  void brogetback(void);
 int distance2 = 0; 
 
//Servo 
 #define TM_BASE_PWM 0x40031000  
#define RCGC_TIMER_R *(volatile unsigned long *) 0x400FE604 
#define RCGC_GPIO_R *(volatile unsigned long *) 0x400FE608 
#define CLOCK_GPIOF 0x00000020 
#define SYS_CLOCK_FREQUENCY 16000000 

 
void delay (unsigned long value);

 
 #define GPTM_CONFIG_PWM_R *(volatile unsigned long *) (TM_BASE_PWM + 0x000) 
 #define GPTM_TA_MODE_PWM_R *(volatile unsigned long *) (TM_BASE_PWM + 0x004) 
 #define GPTM_CONTROL_PWM_R *(volatile unsigned long *) (TM_BASE_PWM + 0x00C) 
 #define GPTM_INT_MASK_PWM_R *(volatile unsigned long *) (TM_BASE_PWM + 0x018) 
 #define GPTM_INT_CLEAR_PWM_R *(volatile unsigned long *) (TM_BASE_PWM + 0x024) 
 #define GPTM_TA_IL_PWM_R *(volatile unsigned long *) (TM_BASE_PWM + 0x028) 
 #define GPTM_TA_MATCH_PWM_R *(volatile unsigned long *) (TM_BASE_PWM + 0x030) 
 	 
 #define GPIO_PORTF_AFSEL_R *((volatile unsigned long *) 0x40025420) 
 #define GPIO_PORTF_PCTL_R *((volatile unsigned long *) 0x4002552C) 
 #define GPIO_PORTF_DEN_R *((volatile unsigned long *) 0x4002551C) 


#define TIM_16_BIT_CONFIG 0x00000004 
#define TIM_PERIODIC_MODE 0x00000002 
#define TIM_A_ENABLE      0x00000001 

#define TIM_PWM_MODE      0x0000000A 
#define TIM_CAPTURE_PWM_MODE  0x00000004 


#define TIM_A_INTERVAL_PWM 64000 
#define TIM_A_MATCH 32000 


#define TIM_A_PRESCALE_VALUE 0x14 
#define TIM_A_PRESCALE_MATCH_VALUE 0x14 


void Timer1A_Init(void); 
void servoNeutral(void); 
void servoRight( void); 
void servoLeft( void); 
void servoConfig(void); 
void Timer1A_Init(void) 
{ 
	
	volatile unsigned long delay_clk;
	RCGC_GPIO_R |=0x03;
	delay_clk = RCGC_GPIO_R;
	
  RCGC_GPIO_R |= CLOCK_GPIOF; //Enabling the clock to PortF Module 
  RCGC_TIMER_R |= 0x02; //Enabling clock to Timer Module 1 
   
  GPIO_PORTF_AFSEL_R |= 0x00000004; //Alternate functionality for PF2 GPIO Pin 
  GPIO_PORTF_PCTL_R |=  0x00000700; //Timer2 Capture Compare mode Alternate functionality 
  GPIO_PORTF_DEN_R |=   0x00000004; //Digital Enable 
 	 
  GPTM_CONTROL_PWM_R &= ~(TIM_A_ENABLE); //Disable the Timer 
  GPTM_CONFIG_PWM_R |= TIM_16_BIT_CONFIG; //Timer as 16 Bit Basic Timer 
 	 
  GPTM_TA_MODE_PWM_R |= TIM_PWM_MODE; //Timer is configured in PWM mode 
  GPTM_TA_MODE_PWM_R &= ~(TIM_CAPTURE_PWM_MODE);  
 	 
  GPTM_TA_IL_PWM_R = TIM_A_INTERVAL_PWM; //Providing the Load values for 250Hz Frequency 
  GPTM_TA_MATCH_PWM_R = TIM_A_MATCH ; //Match value for 50% duty cycle initially 
   
  //GPTM_TA_PRESCALE_R = 4; 
  //GPTM_TA_PRESCALE_MATCH_R   = 1; 
 	 
 	GPTM_CONTROL_PWM_R |= TIM_A_ENABLE; //Enabling the Timer 
 } 
 //Function for putting Servo in Neutral position by changing PWM duty cycle 
 void servoNeutral( void) 
 { 
  GPTM_TA_MATCH_PWM_R = 44500; //25%
 } 
 //Function for putting Servo in Left position by changing PWM duty cycle 
 void servoLeft( void) 
 { 
  GPTM_TA_MATCH_PWM_R = 31000; //~50%
 } 
 //Function for putting Servo in Right position by changing PWM duty cycle 
 void servoRight( void) 
 { 
  GPTM_TA_MATCH_PWM_R = 56000; //12.5%
 } 
 void servoConfig(void) 
 { 
 Timer1A_Init(); 
 }
 
 void delay (unsigned long value) 
{
	unsigned long i;
	for(i=0;i<value;i++);
}
 
 
 
 void distanceSensorConfig(void) 
 { 
 	//Enabling clock to Timer and PortF 
  RCGC_TIMER_R |= 0x04; 
  SYSCTL_RCGCGPIO_R |= CLOCK_GPIOF + PORTE_CLK_EN; 
 	 
 	//Enabling PortE GPIO and module 
  GPIO_PORTE_DEN_R |= TRIG_ECHO_PIN_EN; 
  GPIO_PORTE_DIR_R |= TRIG_PINE2; 
  GPIO_PORTE_DIR_R &=~ECHO_PINE1; 
  GPIO_PORTE_PD_R  |= ECHO_PINE1; 
 	 
 //Enabling PortF PF4 pin with Input Capture mode 
  GPIO_PORTF_DEN_R   |= 0x00000010; 
  GPIO_PORTF_AFSEL_R |= 0x00000010; 
  GPIO_PORTF_PCTL_R  |= 0x00070000; 
 	 
   //Initialize the Timer Module for Input Capture 
  Timer2A_Init(); 
 	 
 	 
 	//Configuring systick timer for 50ms continous Trigger to the Sensor 
  configureSystickTimer(); 
 } 
 
 
 void moveforward(void)
 {
	 //high low and high low
 }
 
 
 void stopmotors(void)
 {
	 //low low and low low
 }
 
 
 void Timer2A_Init(void) 
 { 
   
 	 
  GPTM_CONFIG_R = TIM_16_BIT_CONFIG; //16Bit Timer 
 	 
 	//Timer mode is input Capture with count down 
  GPTM_TA_MODE_R |= TIM_EDGE_TIME_MODE + TIM_CAPTURE_MODE;  
  GPTM_CONTROL_R |= TIM_A_EVENT_POS_EDGE; 
 	 
  //providing the Interval and Prescale values 
  GPTM_TA_IL_R = TIM_A_INTERVAL; 
  GPTM_TA_PRESCALE_R = TIM_A_PRESCALE; 
   
 	//Timer0 A interrupt configuration 
  GPTM_INT_MASK_R |= TIM_A_CAP_EVENT_IM; 
  NVIC_EN0_R = NVIC_EN0_INT23; 
 	 
 	//Enable the Timer 
  GPTM_CONTROL_R |=TIM_A_ENABLE; 
 	 
 } 
 
 
 
 
 void configureSystickTimer (void) 
 { 
   ST_CTRL_R &= ~ST_ENABLE;  //Disabling systick 
 	ST_RELOAD_R = ST_RELOAD_VALUE; //Writing reload value 
 	ST_CURRENT_R = 0;  //Setting Count Flag value to 0 
   ST_CTRL_R |= (ST_INT_EN); 	//Enabling Interrupt 
 	ST_CTRL_R |= ST_CLK_SRC;   //Clock Source is Main clock 
   ST_CTRL_R |= ST_ENABLE; //Enable Systick Timer 
 } 
 
 
 void Timer2A_Handler (void) 
 { 
 
 
 unsigned int time_period = 0; 
 static unsigned int time_capture = 0; 
 float time = 0; 
 static char flag =0;  
 	 
 	 
 	GPTM_INT_CLEAR_R |= TIM_A_CAP_EVENT_IC; //Clear interrupt Flag 
 	GPTM_CONTROL_R ^= TIM_A_EVENT_NEG_EDGE; //Toggling the Event so Pulse width can be measured  
 	 
 	if (flag ==0) 
 	{ 
 	  time_capture = GPTM_TA_COUNT_R; 
 		flag = 1; 
 	} 
 	 
 	else  
 	{ 
 	 
 	  if (GPTM_TA_COUNT_R > time_capture) 
 			time_period = time_capture - GPTM_TA_COUNT_R; 
 		  else 
 			time_period = time_capture - GPTM_TA_COUNT_R; 
 			 
 		//Getting the pulse width time and converting it to milliseconds 
 		 time = (time_period*1000)/16000000.0; 
 			 
		 //Using S= V(T/2) we get the distance; 4 is a calibration factor to convert reading to inches 
 		 distance2 = (time/2.0) * 3.43 * 4; 
 			 

 			 
 			flag = 0; 
 	} 
 } 
 
 
 
 	 
 //Function for Triggering the Distance sensor with a pulse of 10us 
 void triggerSensor (void) 
 { 
 	GPIO_PORTE_DATA_TRIG_R &= ~TRIG_PINE2; 
   //Providing a pulse of >10us on Trigger Pin 
 	GPIO_PORTE_DATA_TRIG_R |= TRIG_PINE2; 
 	Delay(4); 
 	GPIO_PORTE_DATA_TRIG_R &= ~TRIG_PINE2; 
 
 } 
 
 // Function to return the distance measured by the Sensor 
 int getDistance(void) 
 { 
  return distance2; 
 } 


void Delay ( unsigned long value ) {
unsigned long i = 0 ;
for ( i=0 ; i<value ; i++ ) ;
}
int strDistance=0;


 void GPIO_PortA_Init() 
 { 
 	//Initializing the Ports for Motor Driver 
   GPIO_PORTA_DEN_R |= PORTA_765432; 
   GPIO_PORTA_DIR_R |= PORTA_765432; 
 } 
 
 
 void moveBackward (void) 
 { 
 	//Writing the Backward Data 
   GPIO_PORTA_DATA_R = BACKWARD; 
 } 
 
 
 void moveForward (void) 
 { 
 	//Writing the Forward Data 
   GPIO_PORTA_DATA_R = FORWARD; 
 } 
 
 
 void moveLeft (void) 
	 { 
 	//Writing the Left Data 
   GPIO_PORTA_DATA_R = LEFT; 
 } 
 
 
 void moveRight (void) 
 	 
 { 
 	//Writing the Right Data 
  GPIO_PORTA_DATA_R =RIGHT; 
 } 
 
 
 void brogetback(void){
	 
			Stop();
			for(f=0;f<1000000;++f);
			moveBackward();
			for(f=0;f<250000;++f);
			Stop();
			for(f=0;f<1000000;++f);
   
			GPTM_CONTROL_PWM_R &= ~(TIM_A_ENABLE); //Disable the Timer 
			servoConfig();
			
			servoLeft();
			delay(2000000);
			delay(2000000);
			GPTM_CONTROL_PWM_R &= ~(TIM_A_ENABLE); //Disable the Timer 
			configureSystickTimer();
			Delay(SECOND/2);	 
			Delay(SECOND/2);	 
			Delay(SECOND/2);	 
			value1= getDistance();
	   
			
			
			servoConfig();
			servoRight();
			delay(2000000);
			delay(2000000);
			configureSystickTimer(); 
			Delay(SECOND/2);	 
			Delay(SECOND/2);	 
			Delay(SECOND/2);	 
			value2= getDistance();
	   
	 
			servoNeutral();
			delay(2000000);
			GPTM_CONTROL_PWM_R &= ~(TIM_A_ENABLE); //Disable the Timer 
			
			
			
			if(value1<5 | value2<5)
			
				{
					brogetback();
				}
		else{
			   c=0;
				if(value1>value2)
				{			moveLeft();
							for(f=0;f<250000;++f);
							Stop();
							for(f=0;f<1000000;++f);
							moveForward();
							configureSystickTimer();
			
				}
						else {
							moveRight();
							for(f=0;f<250000;++f);
							Stop();
							for(f=0;f<1000000;++f);
							moveForward();
							configureSystickTimer();
			
						}	
				}
				c=0;
 }
 
 void Stop (void) 
 	 
 { 
 
 
 	//Writing the Stop Data 
  GPIO_PORTA_DATA_R =BREAK; 
 } 
 

 
 void motorDriverConfig(void) 
 { 
 	//Enabling clock to Port A and Initializing ports 
   SYSCTL_RCGCGPIO_R |= PORTA_CLK_EN; 
   GPIO_PortA_Init(); 
 } 

 
 int i;
 int main ()
 {
	
	 //sensorconfigured
	 distanceSensorConfig();
	GPIO_PORTF_DEN_R   |= 0xE;
	GPIO_PORTF_DIR_R   |= 0xE;

	 motorDriverConfig();

	 servoConfig();
		servoNeutral();
	 delay(2000000);
	 
while(1){
	
		strDistance=getDistance();
		if(strDistance<=30)
		{
			
			Delay(SECOND/2);
			strDistance =getDistance();
			if(strDistance<25)
			{
				GPIO_PORTF_DATA_R = 0x2;
				brogetback();
		
		//		ST_CTRL_R |= ST_ENABLE;  //enabling systick 
		
			}
		}
		else 
		{
			GPIO_PORTF_DATA_R = 0x8;
			moveForward();
		}
	}
 }