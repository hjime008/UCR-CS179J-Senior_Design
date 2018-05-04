/*
 *
 *
 * Created: 4/24/2018 10:08:23 PM
 * Author : Hector Jimenez
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "pcd8544.h"
#define BAUDRATE 38400
#define USART_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void setupPWM();

void enablePWM();

void disablePWM();

void setupUSART();

void sendUSART(unsigned char dataOut);

unsigned char receiveUSART();

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
uint8_t counter = 0;

void setupScreen();

int main(void)
{
    /* Replace with your application code */
	setupUSART();
	setupScreen();
	

	
    while (1) 
    {

		
		configReg |= sensitivityLevel;
		
		sendUSART(configReg);
		
		while((UCSR0A & (1<<RXC0)))//while usart not available)
		{
			counter++;
			ClearScreen();
			SetTextPosition(0,0);
			DrawString("COMM ERROR");
			SetTextPosition(2,0);
			DrawString(itoa(counter, currXStr, 10));
			UpdateScreen();
		}
		
		recievedStatusReg = receiveUSART();
		ClearScreen();
		SetTextPosition(0,0);
		postureStatus = (recievedStatusReg & 0x0F);
		if (postureStatus == 0x00)//Conditions for received data
		{
			DrawString("Posture OK!");
		}
		else if(postureStatus == 0x01)
		{
			DrawString("Leaning FW!");
		}
		else if(postureStatus == 0x02)
		{
			DrawString("Leaning BW!");
		}
		else if (postureStatus == 0x04)
		{
			DrawString("Leaning Left!");
		}
		else if (postureStatus == 0x08)
		{
			DrawString("Leaning Right!");
		}
		else if (postureStatus == 0x0F)
		{
			DrawString("UPSIDE DOWN!");
		}
		else if (postureStatus == 0x05)
		{
			DrawString("Leaning FWD!");
			SetTextPosition(1,0);
			DrawString("Leaning Left!");
		}
		else if (postureStatus == 0x09)
		{
			DrawString("Leaning FWD!");
			SetTextPosition(1,0);
			DrawString("Leaning Right!");
		}
		else if (postureStatus == 0x06)
		{
			DrawString("Leaning BWD!");
			SetTextPosition(1,0);
			DrawString("Leaning Left!");
		}
		else if (postureStatus == 0x0A)
		{
			DrawString("Leaning BWD!");
			SetTextPosition(1,0);
			DrawString("Leaning Right!");
		}
		else
		{
			counter++;
			DrawString("INVALID DATA");
			SetTextPosition(1,0);
			DrawString(itoa(postureStatus,ucurrXStr, 10));
			SetTextPosition(2,0);
			DrawString(itoa(counter, currXStr, 10));
		}
		UpdateScreen();
		
		_delay_ms(125);

    }
}

void setupPWM()
{
	DDRB |= 0xFD; //Enable pins required for 16-bit PWM as output
	ICR1 = 0xFFFF;
	OCR1A = 0xFFFF;
	OCR1B = 0x0000; //Set registers for 50% duty cycle
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
}

void setupUSART()
{
	UBRR0H = (uint8_t)(USART_PRESCALER>>8); //cast and shift for upper nibble of UBRR
	UBRR0L = (uint8_t)(USART_PRESCALER); //cast and shift for lower nibble of UBRR
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	UCSR0C |= ((1<<UCSZ00)|(1<<UCSZ01));
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
	Pcd8544Init();
	SetTextPosition(0,0);
	DrawString("12:00 PM");
	UpdateScreen();
}