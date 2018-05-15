/*
 *
 *
 * Created: 4/24/2018 10:08:23 PM
 * Author : Hector Jimenez
 */ 

#include <avr/io.h>
#include <util/delay.h>
#define BAUDRATE 38400
#define USART_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void setupPWM(uint8_t frequency);

void enablePWM();

void disablePWM();

void setupUSART();

void sendUSART(unsigned char dataOut);

unsigned char receiveUSART();

void setupExtInterrupts();

int8_t temp[6];
uint8_t ucurrX = 0;
int8_t currX = 0;
int8_t currY = 0;
int8_t currZ= 0;
int8_t storedX = 0;
int8_t storedY = 0;
int8_t storedZ = 0;
char ucurrXStr[12];
char currXStr[12];
char currYStr[12];
char currZStr[12];
unsigned char recievedStatusReg = 0;
unsigned char configReg = 0;
unsigned char postureStatus = 0;
unsigned char sensitivityLevel = 0x60;
unsigned char pinBStatus = 0;
unsigned char btConnected = 0;
uint8_t counter = 0;
uint8_t encoderPos = 0;

void setupScreen();

int main(void)
{
    /* Replace with your application code */
	setupUSART();
	setupScreen();
	setupExtInterrupts();
	

	
    while (1) 
    {

		
		configReg |= sensitivityLevel;
		
		sendUSART(configReg);
		btConnected = PINC; 
		while(!btConnected || (UCSR0A & (1<<RXC0)))//while usart not available)
		{
			counter++;
			clearLCD();
			cursorPos(0,0);
			writeString("COMM ERROR");
			cursorPos(2,0);
			writeString(itoa(counter, currXStr, 10));
			updateLCD();
		}
		
		recievedStatusReg = receiveUSART();
		clearLCD();
		cursorPos(0,0);
		postureStatus = (recievedStatusReg & 0x0F);
		if (postureStatus == 0x00)//Conditions for received data
		{
			writeString("Posture OK!");
		}
		else if(postureStatus == 0x01)
		{
			writeString("Leaning FW!");
		}
		else if(postureStatus == 0x02)
		{
			writeString("Leaning BW!");
		}
		else if (postureStatus == 0x04)
		{
			writeString("Leaning Left!");
		}
		else if (postureStatus == 0x08)
		{
			writeString("Leaning Right!");
		}
		else if (postureStatus == 0x0F)
		{
			writeString("UPSIDE DOWN!");
		}
		else if (postureStatus == 0x05)
		{
			writeString("Leaning FWD!");
			cursorPos(1,0);
			writeString("Leaning Left!");
		}
		else if (postureStatus == 0x09)
		{
			writeString("Leaning FWD!");
			cursorPos(1,0);
			writeString("Leaning Right!");
		}
		else if (postureStatus == 0x06)
		{
			writeString("Leaning BWD!");
			cursorPos(1,0);
			writeString("Leaning Left!");
		}
		else if (postureStatus == 0x0A)
		{
			writeString("Leaning BWD!");
			cursorPos(1,0);
			writeString("Leaning Right!");
		}
		else
		{
			counter++;
			writeString("INVALID DATA");
			cursorPos(1,0);
			writeString(itoa(postureStatus,ucurrXStr, 10));
			cursorPos(2,0);
			writeString(itoa(counter, currXStr, 10));
		}
		updateLCD();
		
		_delay_ms(125);

    }
}

void setupPWM(uint8_t frequency)
{
	DDRB |= 0xFD; //Enable pins required for 16-bit PWM as output
	ICR1 = 0xFFFF;
	if(frequency == 1)
	{
		OCR1A = 0xFFFF;
		OCR1B = 0xFFFF; //Set registers for 50% duty cycle	
	}
	else
	{
		OCR1A = 0xFFFF;
		OCR1B = 0x0000; //Set registers for 50% duty cycle		
	}

	TCCR1B |= (1 << WGM12)|(1 << WGM13);
	TCCR1B |= (1 << CS11) | (1 << CS10); //Prescaler of 64, yielding 2Hz Freq with 8MHz clock
}

void enablePWM()
{
	TCCR1A |= (1 << COM1A1)|(1 << COM1B1) | (1 << WGM11);
}

void disablePWM()
{
	TCCR1A = 0;
	TCNT1 = 0;
}

void setupUSART()
{
	UBRR0H = (uint8_t)(USART_PRESCALER>>8); //Cast and shift for upper nibble of UBRR
	UBRR0L = (uint8_t)(USART_PRESCALER); //Cast and shift for lower nibble of UBRR
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	UCSR0C |= ((1<<UCSZ00)|(1<<UCSZ01)|(1<<UPM01));//Enable standard USART protocol + Even parity
	//UCSR0C |= ((1<<UCSZ00)|(1<<UCSZ01));//Enable standard USART protocol (No Parity)
}

void sendUSART(unsigned char dataOut)
{
	 while(!(UCSR0A & (1<<UDRE0)));
	 UDR0 = dataOut;
}

unsigned char receiveUSART()
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void setupScreen()
{
	pcd8544Setup();
	cursorPos(0,0);
	writeString("12:00 PM");
	updateLCD();
}

void setupExtInterrupts()
{
	EICRA = 0b00001010; // Falling Edge EXT Interrupt Trigger
    EIMSK = 0b00000011; // Enable EXT Interrupts 0 and 1
}

encoderISR (INT0_vect) 
{
	uint8_t currPinDState = PIND;
	if(1)
	{
		//CCW
	}
	else
	{
		//CCW
	}


}

buttonISR (INT1_vect) 
{
	
//enable encoder EXT Interrupt pin on button press interrupt

}
