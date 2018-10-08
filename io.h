#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#ifndef F_CPU
#define F_CPU 16000000UL   //SET CPU CLOCK
#endif
#include <util/delay.h>
#include <avr/interrupt.h>



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

void pinMode(uint8_t , uint8_t );
static void turnOffPWM(uint8_t );
void digitalWrite(uint8_t , uint8_t );
int digitalRead(uint8_t );
unsigned long int microsecondsToInches();
unsigned long int microsecondsToCentimeters();
void setup(void);
void loop(void);
//Function:

/*void pinMode(uint8_t pIn, uint8_t mOde)
{
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mOde == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mOde == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}

static void turnOffPWM(uint8_t tImer)
{
	switch (tImer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

void digitalWrite(uint8_t pIn, uint8_t vAl)
{
	uint8_t tImer = digitalPinToTimer(pIn);
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (tImer != NOT_ON_TIMER) turnOffPWM(tImer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (vAl == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
}

int digitalRead(uint8_t pIn)
{
	uint8_t tImer = digitalPinToTimer(pIn);
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (tImer != NOT_ON_TIMER) turnOffPWM(tImer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}
	
	const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PE	, // PE 0 ** 0 ** USART0_RX	
	PE	, // PE 1 ** 1 ** USART0_TX	
	PE	, // PE 4 ** 2 ** PWM2	
	PE	, // PE 5 ** 3 ** PWM3	
	PG	, // PG 5 ** 4 ** PWM4	
	PE	, // PE 3 ** 5 ** PWM5	
	PH	, // PH 3 ** 6 ** PWM6	
	PH	, // PH 4 ** 7 ** PWM7	
	PH	, // PH 5 ** 8 ** PWM8	
	PH	, // PH 6 ** 9 ** PWM9	
	PB	, // PB 4 ** 10 ** PWM10	
	PB	, // PB 5 ** 11 ** PWM11	
	PB	, // PB 6 ** 12 ** PWM12	
	PB	, // PB 7 ** 13 ** PWM13	
	PJ	, // PJ 1 ** 14 ** USART3_TX	
	PJ	, // PJ 0 ** 15 ** USART3_RX	
	PH	, // PH 1 ** 16 ** USART2_TX	
	PH	, // PH 0 ** 17 ** USART2_RX	
	PD	, // PD 3 ** 18 ** USART1_TX	
	PD	, // PD 2 ** 19 ** USART1_RX	
	PD	, // PD 1 ** 20 ** I2C_SDA	
	PD	, // PD 0 ** 21 ** I2C_SCL	
	PA	, // PA 0 ** 22 ** D22	
	PA	, // PA 1 ** 23 ** D23	
	PA	, // PA 2 ** 24 ** D24	
	PA	, // PA 3 ** 25 ** D25	
	PA	, // PA 4 ** 26 ** D26	
	PA	, // PA 5 ** 27 ** D27	
	PA	, // PA 6 ** 28 ** D28	
	PA	, // PA 7 ** 29 ** D29	
	PC	, // PC 7 ** 30 ** D30	
	PC	, // PC 6 ** 31 ** D31	
	PC	, // PC 5 ** 32 ** D32	
	PC	, // PC 4 ** 33 ** D33	
	PC	, // PC 3 ** 34 ** D34	
	PC	, // PC 2 ** 35 ** D35	
	PC	, // PC 1 ** 36 ** D36	
	PC	, // PC 0 ** 37 ** D37	
	PD	, // PD 7 ** 38 ** D38	
	PG	, // PG 2 ** 39 ** D39	
	PG	, // PG 1 ** 40 ** D40	
	PG	, // PG 0 ** 41 ** D41	
	PL	, // PL 7 ** 42 ** D42	
	PL	, // PL 6 ** 43 ** D43	
	PL	, // PL 5 ** 44 ** D44	
	PL	, // PL 4 ** 45 ** D45	
	PL	, // PL 3 ** 46 ** D46	
	PL	, // PL 2 ** 47 ** D47	
	PL	, // PL 1 ** 48 ** D48	
	PL	, // PL 0 ** 49 ** D49	
	PB	, // PB 3 ** 50 ** SPI_MISO	
	PB	, // PB 2 ** 51 ** SPI_MOSI	
	PB	, // PB 1 ** 52 ** SPI_SCK	
	PB	, // PB 0 ** 53 ** SPI_SS	
	PF	, // PF 0 ** 54 ** A0	
	PF	, // PF 1 ** 55 ** A1	
	PF	, // PF 2 ** 56 ** A2	
	PF	, // PF 3 ** 57 ** A3	
	PF	, // PF 4 ** 58 ** A4	
	PF	, // PF 5 ** 59 ** A5	
	PF	, // PF 6 ** 60 ** A6	
	PF	, // PF 7 ** 61 ** A7	
	PK	, // PK 0 ** 62 ** A8	
	PK	, // PK 1 ** 63 ** A9	
	PK	, // PK 2 ** 64 ** A10	
	PK	, // PK 3 ** 65 ** A11	
	PK	, // PK 4 ** 66 ** A12	
	PK	, // PK 5 ** 67 ** A13	
	PK	, // PK 6 ** 68 ** A14	
	PK	, // PK 7 ** 69 ** A15	
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	_BV( 0 )	, // PE 0 ** 0 ** USART0_RX	
	_BV( 1 )	, // PE 1 ** 1 ** USART0_TX	
	_BV( 4 )	, // PE 4 ** 2 ** PWM2	
	_BV( 5 )	, // PE 5 ** 3 ** PWM3	
	_BV( 5 )	, // PG 5 ** 4 ** PWM4	
	_BV( 3 )	, // PE 3 ** 5 ** PWM5	
	_BV( 3 )	, // PH 3 ** 6 ** PWM6	
	_BV( 4 )	, // PH 4 ** 7 ** PWM7	
	_BV( 5 )	, // PH 5 ** 8 ** PWM8	
	_BV( 6 )	, // PH 6 ** 9 ** PWM9	
	_BV( 4 )	, // PB 4 ** 10 ** PWM10	
	_BV( 5 )	, // PB 5 ** 11 ** PWM11	
	_BV( 6 )	, // PB 6 ** 12 ** PWM12	
	_BV( 7 )	, // PB 7 ** 13 ** PWM13	
	_BV( 1 )	, // PJ 1 ** 14 ** USART3_TX	
	_BV( 0 )	, // PJ 0 ** 15 ** USART3_RX	
	_BV( 1 )	, // PH 1 ** 16 ** USART2_TX	
	_BV( 0 )	, // PH 0 ** 17 ** USART2_RX	
	_BV( 3 )	, // PD 3 ** 18 ** USART1_TX	
	_BV( 2 )	, // PD 2 ** 19 ** USART1_RX	
	_BV( 1 )	, // PD 1 ** 20 ** I2C_SDA	
	_BV( 0 )	, // PD 0 ** 21 ** I2C_SCL	
	_BV( 0 )	, // PA 0 ** 22 ** D22	
	_BV( 1 )	, // PA 1 ** 23 ** D23	
	_BV( 2 )	, // PA 2 ** 24 ** D24	
	_BV( 3 )	, // PA 3 ** 25 ** D25	
	_BV( 4 )	, // PA 4 ** 26 ** D26	
	_BV( 5 )	, // PA 5 ** 27 ** D27	
	_BV( 6 )	, // PA 6 ** 28 ** D28	
	_BV( 7 )	, // PA 7 ** 29 ** D29	
	_BV( 7 )	, // PC 7 ** 30 ** D30	
	_BV( 6 )	, // PC 6 ** 31 ** D31	
	_BV( 5 )	, // PC 5 ** 32 ** D32	
	_BV( 4 )	, // PC 4 ** 33 ** D33	
	_BV( 3 )	, // PC 3 ** 34 ** D34	
	_BV( 2 )	, // PC 2 ** 35 ** D35	
	_BV( 1 )	, // PC 1 ** 36 ** D36	
	_BV( 0 )	, // PC 0 ** 37 ** D37	
	_BV( 7 )	, // PD 7 ** 38 ** D38	
	_BV( 2 )	, // PG 2 ** 39 ** D39	
	_BV( 1 )	, // PG 1 ** 40 ** D40	
	_BV( 0 )	, // PG 0 ** 41 ** D41	
	_BV( 7 )	, // PL 7 ** 42 ** D42	
	_BV( 6 )	, // PL 6 ** 43 ** D43	
	_BV( 5 )	, // PL 5 ** 44 ** D44	
	_BV( 4 )	, // PL 4 ** 45 ** D45	
	_BV( 3 )	, // PL 3 ** 46 ** D46	
	_BV( 2 )	, // PL 2 ** 47 ** D47	
	_BV( 1 )	, // PL 1 ** 48 ** D48	
	_BV( 0 )	, // PL 0 ** 49 ** D49	
	_BV( 3 )	, // PB 3 ** 50 ** SPI_MISO	
	_BV( 2 )	, // PB 2 ** 51 ** SPI_MOSI	
	_BV( 1 )	, // PB 1 ** 52 ** SPI_SCK	
	_BV( 0 )	, // PB 0 ** 53 ** SPI_SS	
	_BV( 0 )	, // PF 0 ** 54 ** A0	
	_BV( 1 )	, // PF 1 ** 55 ** A1	
	_BV( 2 )	, // PF 2 ** 56 ** A2	
	_BV( 3 )	, // PF 3 ** 57 ** A3	
	_BV( 4 )	, // PF 4 ** 58 ** A4	
	_BV( 5 )	, // PF 5 ** 59 ** A5	
	_BV( 6 )	, // PF 6 ** 60 ** A6	
	_BV( 7 )	, // PF 7 ** 61 ** A7	
	_BV( 0 )	, // PK 0 ** 62 ** A8	
	_BV( 1 )	, // PK 1 ** 63 ** A9	
	_BV( 2 )	, // PK 2 ** 64 ** A10	
	_BV( 3 )	, // PK 3 ** 65 ** A11	
	_BV( 4 )	, // PK 4 ** 66 ** A12	
	_BV( 5 )	, // PK 5 ** 67 ** A13	
	_BV( 6 )	, // PK 6 ** 68 ** A14	
	_BV( 7 )	, // PK 7 ** 69 ** A15	
};
*/

unsigned long int microsecondsToInches(unsigned long mIcroseconds) 
{
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  return (mIcroseconds*0.00669/ 2);
}

unsigned long int microsecondsToCentimeters(unsigned long microseconds) 
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

class Serial
{
	public:
	void start( unsigned int uBrr){
		/*Set baud rate */
		UBRR0H = (unsigned char)(uBrr>>8);
		UBRR0L = (unsigned char)uBrr;
		/*Enable receiver and transmitter */
		UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	}
	/* Set frame format: 8data, 2stop bit */
	void send( unsigned char data ){
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) )
		;
		/* Put data into buffer, sends the data */
		UDR0 = data;
		_delay_ms(100);
	}
	unsigned char get( void ){
		/* Wait for data to be received */
		while ( !(UCSR0A & (1<<RXC0)) )
		;
		/* Get and return received data from buffer */
		return UDR0;
	}
	void flush(void){
		unsigned char dUmmy;
		while ( UCSR0A & (1<<RXC0) ) dUmmy = UDR0;
	}

	void end(void){
		flush();
		UCSR0B&=0xe7;	//disabling RXEN & TXEN
	}

};

class Serial1
{
	public:
	void start( unsigned int uBrr){
		/*Set baud rate */
		UBRR1H = (unsigned char)(uBrr>>8);
		UBRR1L = (unsigned char)uBrr;
		/*Enable receiver and transmitter */
		UCSR1B = (1<<RXEN0)|(1<<TXEN0);
	}
	/* Set frame format: 8data, 2stop bit */
	void send( unsigned char data ){
		/* Wait for empty transmit buffer */
		while ( !( UCSR1A & (1<<UDRE)) )
		;
		/* Put data into buffer, sends the data */
		UDR1= data;
		_delay_ms(100);
	}
	unsigned char get( void ){
		/* Wait for data to be received */
		while ( !(UCSR1A & (1<<RXC1)) )
		;
		/* Get and return received data from buffer */
		return UDR1;
	}
	void flush(void){
		unsigned char dUmmy;
		while ( UCSR1A & (1<<RXC1) ) dUmmy = UDR1
		;
	}

	void end(void){
		flush();
		UCSR1B&=0xe7;	//disabling RXEN & TXEN
	}

};
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
        ADMUX|=PiNNo;//chose value from 0 to 7 to chose adc pin accordingly
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


int main(){
	void setup();
	while(1){
		void loop();
	}
}
