void SystemInit()
{};

//enabling the clock of the system
#define SYSCTL_RCGCGPIO_R 	(*((volatile unsigned long*) 0x400FE608)) 
	
//Intialization of the portB registers.
//	The Base Address is 0x40005000 of port B register, from the data sheet.

#define GPIO_PORTB_DEN_R		(*((volatile unsigned long*) 0x4000551C))

#define GPIO_PORTB_DIR_R		(*((volatile unsigned long*) 0x40005400))

#define GPIO_PORTB_DATA_R		(*((volatile unsigned long*) 0x400053FC))



//defination of the macros
	
#define PORTB_CLK_EN 		0x02 // port B clock Enable     

#define CLOCK_GPIOF 0x00000020 // clock enable for the pprt F

// Timer initializing....
	
#define TM_BASE 0x40031000
	
// Peripheral clock enabling for timer and GPIO

#define RCGC2_GPIO_R *( volatile unsigned long *)0x400FE108

#define RCGC_TIMER_R *( volatile unsigned long *)0x400FE604

//Clock Frequency settling time.

#define SYS_CLOCK_FREQUENCY 16000000

// General purpose timer register 

#define GPTM_CONFIG_R *( volatile long *)( TM_BASE + 0x000 )//16 bit is selected

#define GPTM_TA_MODE_R *( volatile long *)( TM_BASE + 0x004 )//offset is used 004 for tA1

#define GPTM_TB_MODE_R *( volatile long *)( TM_BASE + 0x008  )//offset is used 008 for tA2

#define GPTM_CONTROL_R *( volatile long *)( TM_BASE + 0x00C )

#define GPTM_INT_MASK_R *( volatile long *)( TM_BASE + 0x018 )

#define GPTM_INT_CLEAR_R *( volatile long *)( TM_BASE + 0x024)

#define GPTM_TA_IL_R *( volatile long *)( TM_BASE + 0x028 )

#define GPTM_TB_IL_R *( volatile long *)( TM_BASE + 0x02C )

#define GPTM_TA_MATCH_R *( volatile long *)( TM_BASE + 0x030 )

#define GPTM_TB_MATCH_R *( volatile long *)( TM_BASE + 0x034 )

// pf2 alternate functionality is configured
#define GPIO_PORTF_AFSEL_R *(( volatile unsigned long *)0x40025420 )

//port Control Registers
#define GPIO_PORTF_PCTL_R *(( volatile unsigned long *)0x4002552C )

#define GPIO_PORTF_DEN_R *(( volatile unsigned long *)0x4002551C )// Timer config and mode bit field definitions
#define TIM_16_BIT_CONFIG 0x00000004 // 16bit congfigurations

#define TIM_PERIODIC_MODE 0x00000002 

#define TIM_AB_ENABLE 0x00000101 // Timer enabling, 101

#define TIM_PWM_MODE_A 0x0000000A

#define TIM_PWM_MODE_B 0x0000000A 

#define TIM_CAPTURE_MODE 0x00000004 

//1 kHz PWM frequency, The reload Value.
#define TIM_A_INTERVAL 16000 

#define TIM_B_INTERVAL 16000
	
//sensor reading values

#define INT_PB0					0x01          // sensor1 value
#define INT_PB1					0x02          // sensor2 value
#define INT_PB2					0x04          // sensor3 value
#define INT_PB3					0x08          // sensor4 value
#define INT_PB4					0x10          // sensor5 value
#define INT_PB5					0x20          // sensor6 value

#define PORTA_DIR 		  0xFF          // for the ports to have the direction same.

#define PORTA_DEN			  0xFF          //for the ports to be digitally enabled.


void Delay( unsigned long duration)
{
    while ( ( duration -- )!= 0);
}


// timer function
void Timer1A_Init ( void )
{
// Enable the clock for Timer 1, timer1 is 16bit timer
GPTM_CONFIG_R |= TIM_16_BIT_CONFIG ;
	
// TIMER1A in periodic,edge, and PWM modes
GPTM_TA_MODE_R |= TIM_PWM_MODE_A ;
GPTM_TA_MODE_R &= ~( TIM_CAPTURE_MODE );
	
// by using the reload vlaue of 16000, the PWM frequency is 1 KHZ
	
GPTM_TA_IL_R = TIM_A_INTERVAL ;
}
//similarly for timerB
void Timer1B_Init ( void )
{
GPTM_CONFIG_R |= TIM_16_BIT_CONFIG ;
GPTM_TB_MODE_R |= TIM_PWM_MODE_B ;
GPTM_TB_MODE_R &= ~( TIM_CAPTURE_MODE );
GPTM_TB_IL_R = TIM_B_INTERVAL;
}


//TheMainFunction
int __main (void)
{
int Kp=2250;
int Kd=650;
int Ki=0;                                                                                                          

//int error
	signed int error, P, I, D, PIDvalue;
	signed int previousError, previousI;
	int leftMotorSpeed;
	int rightMotorSpeed;
	int min=15999 ,max=6500;
	
	
	volatile unsigned delay_clk;
	
	SYSCTL_RCGCGPIO_R |=  CLOCK_GPIOF + PORTB_CLK_EN;  
	Delay(6);

	GPIO_PORTB_DEN_R 	|= 0xFF;         // pf1 and pf2 digital enable
	GPIO_PORTB_DIR_R 	&= ~( 0xFF) ;    // pf1 and pf2 direction settling

	// Enable the clock for port F and Timer1
	RCGC_TIMER_R |= 0x02 ;
	
// PORTF pin 2,3 configuration as Timer1 A ,B output
   
	 GPIO_PORTF_AFSEL_R|= 0x0000000c ;
   GPIO_PORTF_PCTL_R|= 0x00007700 ; 
   GPIO_PORTF_DEN_R|= 0x0000000C ;
	
	GPTM_CONTROL_R &= ~( TIM_AB_ENABLE );

	Timer1A_Init ();
	Timer1B_Init ();

	GPTM_CONTROL_R |= TIM_AB_ENABLE ;
	Delay(8);
	

	while(1)
	{
//THE LOOP is infi	
  if(GPIO_PORTB_DATA_R==0x11)

		{
			error = 0;
previousError = error;}
  if(GPIO_PORTB_DATA_R==0x18)

		{
			error = 1;
previousError = error;} 
	
  if(GPIO_PORTB_DATA_R==0x1C)

		{error = 2;
previousError = error;}
  if(GPIO_PORTB_DATA_R==0x1E)
		{error = 4;
previousError = error;}

	if(GPIO_PORTB_DATA_R==0x03) 

		
		{error = -1;
previousError = error;}
  if(GPIO_PORTB_DATA_R==0x07)

		{error = -2;
previousError = error;}
		
  if(GPIO_PORTB_DATA_R==0x0F)

		{error = -3;
previousError = error;}
			
	P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
					
  rightMotorSpeed = 10000 + PIDvalue;
	leftMotorSpeed = 10000 - PIDvalue;
	
	if(leftMotorSpeed >min)
	{
		leftMotorSpeed=min;
   }
	 if(leftMotorSpeed <max)
	{
		leftMotorSpeed=max;
   }
	 
	 if(rightMotorSpeed >min)
	{
		rightMotorSpeed=min;
   }
	 if(rightMotorSpeed <max)
	{
		rightMotorSpeed=max;
  }
	
	
  GPTM_TB_MATCH_R = rightMotorSpeed;
	GPTM_TA_MATCH_R =leftMotorSpeed;
  
}
}
