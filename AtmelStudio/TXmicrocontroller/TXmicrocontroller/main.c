
#define F_CPU 16000000L //CPU Clock

#define BAUD 57600 //Baud Rate
#define BRC ((F_CPU/16/BAUD)-1) //Baud Rate Register Value
#define JYCT 8  //Number of potentiometer
//include libraries
#include<avr\io.h>
#include<avr\interrupt.h>

//initialization of functions
void selectADC(uint8_t ADC_pt);
void setupADC();
void startADC();
void setupTX();
void checkeq(void);
//variables
uint8_t CdutyCycle[JYCT];
uint8_t PdutyCycle[JYCT]={255,255,255,255,255,255,255,255}; //Previous DutyCycle
int idC=0;
uint8_t eqflag=1;
int counter=0;
int main(void)
{
	//setting up ADC and USART Transmitter
	setupTX();
	setupADC();
	//Enabling Global Interrupts
	sei();
	
	while(1)
	{
		checkeq();
	}
	
}

void checkeq(void)
{
	//checking if array is same at the beginning of array transmit
	if (idC==0)
	{
		//comparing current and previous dutyCycles
		for(int ti=0;ti<JYCT;ti++)
		{
			if(PdutyCycle[ti]!=CdutyCycle[ti])
			{
				//eqflag=0;
				UCSR0B=(1<<TXEN0)|(1<<UDRIE0);
				break;
			}
			else
			{
				UCSR0B=0;
			}
		}
	}
}

//Setting Up Transmitter
void setupTX()
{
	//Baud Rate Register Setup
	UBRR0H =(BRC >> 8);
	UBRR0L= BRC;
	//Enabling Transmitter and Interrupt for Empty Transmit Buffer
	UCSR0B=(1<<TXEN0)|(1<<UDRIE0);
	//Configuring Asynchronus USART with 8 bit data and a single parity bit
	UCSR0C=(1<<UCSZ01) |(1<<UCSZ00)|(1<<UPM01);
}
//Interrupt in case of empty transmit buffer
ISR(USART_UDRE_vect)
{
	
	UDR0=CdutyCycle[idC];
	PdutyCycle[idC]=CdutyCycle[idC];
	
	if (idC==(JYCT-1))
	{
		idC=0;
		
	}
	else
	{
		idC++;
	}
}


//select ADC Port
void selectADC(uint8_t ADC_pt)
{
	//Modifying first four bits of ADMUX to select ADC Port
	ADMUX=(ADMUX & 0xE0) | (ADC_pt & (0x1F));
}
//Start ADC Converstion
void startADC()
{
	ADCSRA|=(1<<ADSC);
}
//Setup ADC
void setupADC()
{
	//Selecting Reference Voltage Port and Left Adjustment Result
	ADMUX|=(1<<REFS0)|(1<<ADLAR);
	//Enabling ADC, Prescaling by 128, Enabling ADC Interrupt
	ADCSRA=(1<<ADEN)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
	//Disabling Digital Input
	DIDR0=0x3F;
	startADC();
}
//Interrupt for ADC Completion
ISR(ADC_vect)
{
	//Current ADC Port
	uint8_t curr_ADC=ADMUX & 0x0F;
	
	switch(curr_ADC)
	{
		case 0x00:
		{
			CdutyCycle[0]=ADCH;
			break;
		}
		case 0x01:
		{
			CdutyCycle[1]=ADCH;
			break;
		}
		case 0x02:
		{
			CdutyCycle[2]=ADCH;
			break;
		}
		case 0x03:
		{
			CdutyCycle[3]=ADCH;
			break;
		}
		case 0x04:
		{
			CdutyCycle[4]=ADCH;
			break;
		}
		case 0x05:
		{
			CdutyCycle[5]=ADCH;
			break;
		}
		case 0x06:
		{
			CdutyCycle[6]=ADCH;
			break;
		}
		case 0x07:
		{
			CdutyCycle[7]=ADCH;
			break;
		}
		
	}
	if (curr_ADC ==(JYCT-1))
	{
		selectADC(0x00);
	}
	
	else
	{
		selectADC(curr_ADC+1);
	}
	startADC();
}