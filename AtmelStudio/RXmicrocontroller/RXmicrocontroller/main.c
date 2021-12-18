#define F_CPU 16000000L //CPU Clock Freq
#define BAUD 57600 //Baudrate in bps
#define BRC ((F_CPU/16/BAUD)-1) //Baud Rate Register Value

#define SERVO_PORT  PORTC   //port to which servos are attached
#define SERVO_DDR   DDRC
#define SERVO_PORT2  PORTD   //port to which servos are attached
#define SERVO_DDR2   DDRD
#define N_SERVOS    8      //number of servos
#define SERVO_FRAME 20000 // microseconds (50Hz)
#define SERVO_TIME_DIV (SERVO_FRAME / N_SERVOS)
#define US2TIMER1(us) ((us) * (uint16_t)(F_CPU / 1E6))




//include libraries
#include<avr\io.h>
#include<avr\interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <util/delay.h>


//delcare variables
volatile uint16_t servoTicks[N_SERVOS];
static uint8_t servo=0;
static int outputLow = 1;
uint8_t motoranglesvalues[N_SERVOS];
int idC=0;

const static uint8_t servoOutMask[N_SERVOS] = {
	0b00000001, // PX0
	0b00000010, // PX1
	0b00000100, // PX2
	0b00001000, // PX3
	0b00010000, // PX4
	0b00100000, // PX5
	0b01000000, // PX6
	0b10000000, // PX7
};




unsigned int map(int x, float xmin,float xmax,float ymin,float ymax);
void setupRX();
void initServo();
void writeangle(uint8_t servo, uint8_t angle);
void writeus(uint8_t servo, uint8_t angle );
void writepotval(uint8_t servo, uint8_t angle );
void setmotors();
void setmotorspot();


int main(void)
{
	//DDRB=0x01;//New Line
	
	
	for(int i=0;i<N_SERVOS;i++){
		motoranglesvalues[i]=0; //setting all servos to 0 initially
	}
	//setup Reciever
	setupRX();
	//initiliaze timers and servo port
	initServo();
	sei();
	
	while(1) 
	{
		setmotorspot();//set servo values to values stored in motoranglevalues array
	}
	
}

//Declaration of USART Reciever
void setupRX()
{
	//Baud Rate Register Setup
	UBRR0H=(BRC>>8);
	UBRR0L=BRC;
	// Enable Recieve and Recieve Complete Interrupt
	UCSR0B=(1<<RXEN0)|(1<<RXCIE0);
	// Prescaler to 128 and Even Parity Enable
	UCSR0C=(1<<UCSZ01)|(1<<UCSZ00)|(1<<UPM01);

}




uint16_t map(int x, float xmin,float xmax,float ymin,float ymax) {

	return ((((x - xmin) * (ymax - ymin))/(xmax - xmin)) + ymin); //maps given value from x range to t range 
	
}



void initServo()
{
	// Outputs
	SERVO_DDR |= 0b01111111;
	SERVO_DDR2 |= 0b11000000;
	// Setup a first compare match
	//TCNT1=0;
	OCR1A = TCNT1 + US2TIMER1(1000);
	// Enable interrupt
	TIMSK1 = (1 << OCIE1A);
	// start timer 1 with no prescaler
	TCCR1B = (1 << CS10);
	
}

void writeangle(uint8_t servo, uint8_t angle )
{
	cli();//clear interrupt
	servoTicks[servo] = map(angle,0,180,16000,32000); //maps angle to ticks and assigns to servoTicks for servo number=servo
	// Enable interrupt
	sei();
}

void writeus(uint8_t servo, uint8_t timems )
{
	cli();// clear interrupt
	servoTicks[servo] = map(timems,1000,2000,16000,32000);//maps microsec to ticks and assigns to servoTicks for servo number=servo
	// Enable interrupt
	sei();
}


void writepotval(uint8_t servo, uint8_t angle )
{
	cli();// Clear interrupt
	servoTicks[servo] = map(angle,0,255,16000,32000);//maps 8-bit val to ticks and assigns to servoTicks for servo number=servo
	sei();// Enable interrupt
}


void setmotors()
{
	for(int i=0;i<N_SERVOS;i++)
	{
		writeangle(i, motoranglesvalues[i]);//writes angle value to servos
		_delay_ms(10);
	}
}

void setmotorspot(){
	for(int i=0;i<N_SERVOS;i++)
	{
		writepotval(i, motoranglesvalues[i]);//write 8-bit val to servos
		_delay_ms(10);
	}
}



ISR(TIMER1_COMPA_vect)
{
	static uint16_t nextStart;
	//static uint8_t servo;
	//static int outputHigh = 1;
	uint16_t currentTime = TCNT1;
	uint8_t mask;
	mask = servoOutMask[servo];
	
	if (outputLow) 
	{
		if (servo>=N_SERVOS-2)
		{
			SERVO_PORT2 = mask;	
		}
		else
		{
			SERVO_PORT = mask;	
		}
		
		// Set the end time for the servo pulse
		OCR1A = currentTime + servoTicks[servo];
		nextStart = OCR1A + US2TIMER1(SERVO_TIME_DIV);
	}	 
	else 
	{
		if (servo>=N_SERVOS-2)
		{
			SERVO_PORT2 &= ~mask;	
		}
		else
		{
			SERVO_PORT &= ~mask;	
		}
		servo++;
		if (servo == N_SERVOS) 
			{ 
				servo = 0;
			}
		OCR1A = nextStart;
	}
	outputLow = !outputLow;
}


ISR(USART_RX_vect)
{
	//cli();
	motoranglesvalues[idC]=UDR0;
	if(idC>=(N_SERVOS-1))
	{
		idC=0;
	}
	else
	{
		idC++;
	}
	//sei();
}
