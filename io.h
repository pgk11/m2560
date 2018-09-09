#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL   //SET CPU CLOCK
#endif
//Function declaration
double pulseIn(volatile uint8_t , uint8_t );
double microsecondsToInches(unsigned long );
double microsecondsToCentimeters(unsigned long );
void analogWrite(uint8_t ,uint8_t );
uint8_t analogRead(uint8_t);
void delay(unsigned long);
void delayMicroseconds(unsigned long);
double map(double,double,double,double,double);
double constrain(double,double,double);
void attachIntterupt(int, void *,int);
void (*cAllisr)(void); //function pointer used in ISR()
//Function:

unsigned long microsecondsToInches(unsigned long mIcroseconds) 
{
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  return (mIcroseconds*0.00669/ 2);
}

unsigned long microsecondsToCentimeters(unsigned long microseconds) 
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (microseconds*0.17/ 2);
}

unsigned long pulseIn(volatile uint8_t pInno, uint8_t vAlue)
{
  TCCR2A = (1 << WGM21) | (1 << COM2A1) | (1 << FOC2A) | (0 << COM2A0) | (0 << WGM20); //initializing in CTC mode
  TCCR2A = (1 << CS20);
  unsigned long mAxloops = 500000;
  unsigned long wIdth = 0;
  // wait for any previous pulse to end
  while (((PIND) && (pInno)) == vAlue)
	  {
		if (--mAxloops == 0)
		return 0;
	  }
  // wait for the pulse to start  
  while (((PIND) && (pInno)) != vAlue)
	  {
		if (--mAxloops == 0)
		return 0;
	  }
  // wait for the pulse to stop
  while (((PIND) && (pInno)) == vAlue)
	  {
		if (++wIdth == mAxloops)
		return 0;
	   }
  return wIdth;
}

void USART_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
}

/* Set frame format: 8data, 2stop bit */
unsigned char USART_Receive0( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}

void USART_Transmit0( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)))
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
	_delay_ms(100);
	
	
}

unsigned char USART_Receive1( void )
{
	/* Wait for data to be received */
	while (!(UCSR1A & (1<<RXC1)));
	/* Get and return received data from buffer */
	return UDR1;
}

void USART_Transmit1( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) )
	;
	/* Put data into buffer, sends the data */
	UDR1 = data;
	_delay_ms(100);
	
	
}

void initADC()
{
	ADMUX=(1<<REFS0);				//Aref=AVcc
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);		//ADC enabled, Prescaler 64
}

int analogRead(int PiNNo)
{
        //prescalar set to default
  	ADMUX=(1<<REFS0)|(0<<REFS1);
  	ADCSRA|=(1<<ADEN);
        ADMUX|=x;//chose value from 0 to 7 to chose adc pin accordingly
        ADCSRA|=(1<<ADEN);
        ADCSRA|=(1<<ADSC);
	while(ADCSRA&(1<<ADSC));
	return (ADC);
}

void analogWrite(uint8_t pInNo,uint8_t dUtycY)
{
  TCCR1B=(1<<CS11)|(1<<CS10);
  TCCR1A=(1<<WGM10)|(1<<WGM12)|(1<<COM1A1)|(1<<COM1B1);
	if(pInNo==1)
	{
	  OCR1A=dUtycY;
	}
	if(pInNo==2)
	{
          OCR1B=dUtycY;
	}
}


void Dmilli(int j)
{ 
	TCCR0A|=(1<<WGM01);
	TCCR0A|=(1<<CS00);
	TIMSK0|=(1<<OCIE0A);
	OCR0A=255;
	TCNT0=0;
        long int x=31.50*j;
        long int i;
        for(i=0;i<x;i++)
        {
	  while(!(TIFR0 & (1 << OCF0A))) 
          {
          }
          TIFR0|=(1<<OCF0A);
	}
}

float x=0;
int millis()
{
	float l;
	l=x*0.16+0.00000625*TCNT0;
        return l;
}

void tinit(void)
{ 
	TCCR0A|=(1<<WGM01);
        TCCR0A|=(1<<CS00);
	TIMSK0|=(1<<TOV0);
        TCNT0=0;
}

int main(void)
{
        tinit();
        DDRB=0b1111111;
	int y;
	sei();
	while (1)
	{
	 y=millis();
	 PORTB=y;
 	} 
}

ISR(TIMER0_OVF_vect)
{
	x++;	
}	

void delay(unsigned long mIllisec)
{
	int i;
	for(i=0;i<mIllisec;i++)
	{
		_delay_ms(1);
	}
	return;
}

void delayMicroseconds(unsigned long mIcrosec)
{
	int i;
	for(i=0;i<mIcrosec;i++)
	{
		_delay_us(1);
	}
	return;
}

double map(double vAlue, double fromLow, double fromHigh, double toLow, double toHigh)
{
	return ((vAlue-fromLow)/abs(fromHigh-fromLow)*abs(toHigh+toLow));
}

double constrain(double nUm,double uPper,double lOwer)
{
	if(nUm<uPper){
		return uPper;}
	else if(nUm>lOwer){
		return lOwer;}
	else 
	return nUm;	
}
void attachIntterupt(int pIn, void (*iSrfunc)(void), int cOmpare)		//cOmpare:LOW=0,HIGH1,RISING=2,FALLING=3
{
	sei();
	cAllisr=iSrfunc;
	switch(pIn)	  //enabling interrupt pin
	{
		case 0:
		EIMSK|=1<<INT0;
		switch(cOmpare){
			case 2:
			EICRA|=(1<<ISC00)|(1<<ISC01);
			break;
			case 3:
			EICRA|=(0<<ISC00)|(1<<ISC01);
			break;
			case 4:
			EICRA|=(1<<ISC00)|(0<<ISC01);
			break;
			default:
			EICRA|=(0<<ISC00)|(0<<ISC01);
		}
		break;

		case 1:
		EIMSK|=1<<INT1;
                switch(cOmpare)
		{
			case 2:
			EICRA|=(1<<ISC10)|(1<<ISC11);
			break;
			case 3:
			EICRA=(0<<ISC10)|(1<<ISC11);
			break;
			case 4:
			EICRA|=(1<<ISC10)|(0<<ISC11);
			break;
			default:
			EICRA|=(0<<ISC10)|(0<<ISC11);
		}
		break;
		
		case 2:
		EIMSK|=1<<INT2;
		switch(cOmpare)
		{
			case 2:
			EICRA|=(1<<ISC20)|(1<<ISC21);
			break;
			case 3:
			EICRA=(0<<ISC20)|(1<<ISC21);
			break;
			case 4:
			EICRA|=(1<<ISC20)|(0<<ISC21);
			break;
			default:
			EICRA|=(0<<ISC20)|(0<<ISC21);
		}
		break;
		
		case 3:
		EIMSK|=1<<INT3;
		switch(cOmpare)
		{
			case 2:
			EICRA|=(1<<ISC30)|(1<<ISC31);
			break;
			case 3:
			EICRA=(0<<ISC30)|(1<<ISC31);
			break;
			case 4:
			EICRA|=(1<<ISC30)|(0<<ISC31);
			break;
			default:
			EICRA|=(0<<ISC30)|(0<<ISC31);
		}
		break;
		
		case 4:
		EIMSK|=1<<INT4;
                switch(cOmpare)
		{
			case 2:
			EICRB|=(1<<ISC40)|(1<<ISC41);
			break;
			case 3:
			EICRB=(0<<ISC40)|(1<<ISC41);
			break;
			case 4:
			EICRB|=(1<<ISC40)|(0<<ISC41);
			break;
			default:
			EICRB|=(0<<ISC40)|(0<<ISC41);
		}
		break;
		
		case 5:
		EIMSK|=1<<INT5;
		switch(cOmpare)
		{
			case 2:
			EICRB|=(1<<ISC50)|(1<<ISC51);
			break;
			case 3:
			EICRB=(0<<ISC50)|(1<<ISC51);
			break;
			case 4:
			EICRB|=(1<<ISC50)|(0<<ISC51);
			break;
			default:
                 	EICRB|=(0<<ISC40)|(0<<ISC41);
			
		}
		break;
		
		case 6:
		EIMSK|=1<<INT6;
	        switch(cOmpare)
		{
			case 2:
			EICRB|=(1<<ISC60)|(1<<ISC61);
			break;
			case 3:
			EICRB=(0<<ISC60)|(1<<ISC61);
			break;
			case 4:
			EICRB|=(1<<ISC60)|(0<<ISC61);
			break;
			default:
			EICRB|=(0<<ISC60)|(0<<ISC61);
			
		}
		break;
		
		case 7:
		EIMSK|=1<<INT7;
		switch(cOmpare)
		{
			case 2:
			EICRB|=(1<<ISC70)|(1<<ISC71);
			break;
			case 3:
			EICRB=(0<<ISC70)|(1<<ISC71);
			break;
			case 4:
			EICRB|=(1<<ISC70)|(0<<ISC71);
			break;
			default:
			EICRB|=(0<<ISC70)|(0<<ISC71);
		}
		break;
		
	        default:EICRA|=(0<<ISC01)|(0<<ISC00);
	}
}
ISR(INT0_vect)
{
   cAllisr();
}
ISR(INT1_vect)
{
   cAllisr();
}
ISR(INT2_vect)
{
   cAllisr();
}
ISR(INT3_vect)
{
   cAllisr();
}
ISR(INT4_vect)
{
    cAllisr();
}
ISR(INT5_vect)
{
    cAllisr();
}
ISR(INT6_vect)
{
    cAllisr();
}
ISR(INT7_vect)
{ 
    cAllisr();
}
