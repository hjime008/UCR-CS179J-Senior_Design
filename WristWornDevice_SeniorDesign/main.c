/*
 *
 *
 * Created: 4/24/2018 10:08:23 PM
 * Author : Hector Jimenez
 * Wrist-Worn Device of UCR CS179J Senior Design Project
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#define BAUDRATE 38400
#define USART_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void setupSysTickTimer();

void enableSysTickTimer();

void setupExternalInput();

void setupUSART();

void sendUSART(unsigned char dataOut);

unsigned char receiveUSART();

unsigned char USARTAvailable();

void setupExtInterrupts();

void sys_tick();

void Calculate12Hour();

void updateTime();

void updateBackight();

void enableBT();

void disableBT();

volatile unsigned char sys_tick_flag = 0;

volatile unsigned char tickCount = 0;

int8_t temp[6];
uint8_t ucurrX = 0;
int8_t currX = 0;
int8_t currY = 0;
int8_t currZ= 0;
int8_t storedX = 0;
int8_t storedY = 0;
int8_t storedZ = 0;
char* am_pmString;
char amStr[3] = "AM";
char pmStr[3] = "PM";
char errStr[4] = "ERR";
char currXStr[12];
char currYStr[12];
char currZStr[12];

unsigned char setupCount = 0;
unsigned char recievedStatusReg = 0;
unsigned char configReg = 0;
unsigned char postureStatus = 0;
unsigned char sensitivityLevel = 1;//default to 1, zero is off
unsigned char pinBStatus = 0;
unsigned char pinCStatus = 0;
unsigned char btConnected = 0;
unsigned char dataReceived = 0;
unsigned char backlightStatus = 0;
unsigned char backlightCounter = 0;
unsigned char inactivityCounter = 0;
unsigned char menuMode = 0;// flag to enter menu mode
unsigned char statMode = 0;
unsigned char menuBack = 0;
unsigned char dtMode = 0;
unsigned char hrSet = 0;
unsigned char minSet = 0;
unsigned char secSet = 0;
uint8_t quarterSec = 0;
uint8_t currHour = 0;
uint8_t currMin = 0;
uint8_t currSec = 0;
uint8_t curr12Hour;
uint8_t currentDevice = 1;//default for posture device
uint8_t counter = 0;
uint8_t encoderPos = 1;
uint8_t currPinDState = 0;
uint16_t fwdLeanCount = 0;
uint16_t bwdLeanCount = 0;
uint16_t leftLeanCount = 0;
uint16_t rightLeanCount = 0;

enum system_states {initial, setup, setupWait, home_screen, settings, statistics, date_time} system_state;


void setupScreen();
void updateHomeScreen();
void updateSettingsMenu();
void updateStatisticsMenu();
void updateDateTimeMenu();

void sys_tick()
{
	switch(system_state)
	{
		case initial:
			system_state = setup;
			break;
			
		case setup:
			system_state = setupWait;
			break;
		
		case setupWait:
			if (setupCount < 8)
			{
				system_state = setupWait;
			}
			else
			{
				system_state = home_screen;
			}
			break;
		
		case home_screen:
			if(menuMode)
			{
				system_state = settings;
			}
			else
			{
				system_state = home_screen;
			}
			break;
			
		case settings:
			if (menuMode)
			{
				if(statMode)
				{
					system_state = statistics;
					encoderPos = 0;//clear stat option
				}
				else if (dtMode)
				{
					system_state = date_time;
					encoderPos = 0;
				}
			}
			else
			{
				system_state = home_screen;	
			}
			break;
			
		case statistics:
			if (menuBack)
			{
				menuBack = 0;
				statMode = 0;
				system_state = settings;
			} 
			else
			{
				system_state = statistics;
			}
			break;
			
		case date_time:
			if (menuBack)
			{
				menuBack = 0;
				dtMode = 0;
				system_state = settings;
			}
			else
			{
				system_state = date_time;
			}
			break;
			
		default:
			system_state = initial;
			break;
	}
	
	switch(system_state)
	{
		case initial:
			break;
			
		case setup:
			enableBT();
			setupScreen();
			setupUSART();
			setupExtInterrupts();
			setupExternalInput();
			//_delay_ms(2000);
			setupCount++;
			break;
			
		case setupWait:
			setupCount++;
			break;
			
		case home_screen:
			updateHomeScreen();
			break;
			
		case settings:
			updateSettingsMenu();
			break;
			
		case statistics:
			updateStatisticsMenu();
			break;
			
		case date_time:
			updateDateTimeMenu();
			break;
	}
}


int main(void)
{
	setupSysTickTimer();
	enableSysTickTimer();

	
    while (1) 
    {
		//updateTime();
		sys_tick();
		while(!sys_tick_flag);
		sys_tick_flag = 0;
		updateTime();
		updateBacklight();
		inactivityTimer();
		//_delay_ms(5);

    }
}

void Calculate12Hour()
{
		if (currHour < 0)
		{
			curr12Hour = 99;
			am_pmString = errStr;
		}
		else if (currHour == 0)
		{
			curr12Hour = 12;
			am_pmString = amStr;
		}
		else if (currHour < 12)
		{
			curr12Hour = currHour;
			am_pmString = amStr;
		}
		else if (currHour == 12)
		{
			curr12Hour = 12;
			am_pmString = pmStr;
		}
		else if (currHour > 12)
		{
			curr12Hour = currHour - 12;
			am_pmString = pmStr;
		}
}

void updateTime()
{
	quarterSec++;
	if(quarterSec >= 8)
	{
		quarterSec = 0;
		currSec++;
	}
	
	if (currSec >= 60)
	{
		currSec = 0;
		currMin++;
	}
	
	if (currMin >= 60)
	{
		currMin = 0;
		currHour++;
	}
	
	if (currHour >= 24)
	{
		currHour = 0;
	}
}

void inactivityTimer()
{
	if(menuMode == 1)
	{
		if (inactivityCounter >= 40)
		{
			encoderPos = 1;//reset system to mainmenu
			menuMode = 0;
			statMode = 0;
			dtMode = 0;
			hrSet = 0;
			minSet = 0;
			secSet = 0;
			system_state = home_screen;
		}
		else
		{
			inactivityCounter++;
		}
	}
}

void updateBacklight() 
{
	
	if (backlightStatus == 1)
	{
		if(backlightCounter >= 40)
		{
			backlightStatus = 0;
			backlightCounter = 0;
			PORTB |= 0x40;
		}
		else
		{
			PORTB &= 0xBF;
			backlightCounter++;
		}
	}
}

void setupSysTickTimer()
{
	TCCR1A = 0;
	TCCR1B = 0; 
	TCNT1  = 0; 
	OCR1A = 31249; 
	//TCCR1B |= (1 << WGM12);//Used as toggle for timer, bit set in enableSysTickTimer()
	TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
	TIMSK1 |= (1 << OCIE1A);
	sei();
}

void enableSysTickTimer()
{
	TCCR1B |= (1 << WGM12);
}

void disableSysTickTimer()
{
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
}

void enableBT()
{
	DDRB &= 0x7F;//Input mode to not disturb en pin
}

void disableBT()
{
	DDRB |= 0x80;//Output mode to logical zero to disable bt module 
	PORTB &= 0x7F;
}


void setupUSART()
{
	UBRR0H = (uint8_t)(USART_PRESCALER>>8); //Cast and shift for upper nibble of UBRR
	UBRR0L = (uint8_t)(USART_PRESCALER); //Cast and shift for lower nibble of UBRR
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	//UCSR0C |= ((1<<UCSZ00)|(1<<UCSZ01)|(1<<UPM01));//Enable standard USART protocol + Even parity
	UCSR0C |= ((1<<UCSZ00)|(1<<UCSZ01));//Enable standard USART protocol (No Parity)
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

unsigned char USARTAvailable()
{
	if ((UCSR0A & (1<<RXC0)))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void setupExternalInput()
{
	DDRC = 0x00;
	PORTC = 0xFF;
}

void setupScreen()
{
	DDRB |= 0x40;//Set pin used for , active low	//PORTB &= 0xBF; 
	PORTB |= 0x40;
	backlightStatus = 0;
	backlightCounter = 0;
	pcd8544Setup();
	cursorPos(0,0);
	writeString("System Start");
	updateLCD();
}

void updateHomeScreen()
{
		btConnected = PIND & 0x80;
		//counter++;
		clearLCD();
		cursorPos(0,0);
		
		Calculate12Hour();
		writeString(itoa(curr12Hour, currXStr, 10));
		writeString(":");
		
		if (currMin < 10)
		{
			writeString("0");//pad time with zero
		}
		writeString(itoa(currMin, currXStr, 10));
		writeString(":");
		
		if (currSec < 10)
		{
			writeString("0");//pad time with zero
		}
		writeString(itoa(currSec, currXStr, 10));
		writeString(" ");
		writeString(am_pmString);
		
		cursorPos(1,0);
		if(currentDevice == 0 )
		{
			btConnected = 0;
		}
		if(btConnected)
		{
			configReg &= 0x1F;
			configReg |= (sensitivityLevel << 5); 
			sendUSART(configReg);
			if ((UCSR0A & (1<<RXC0)))//(USARTAvailable())
			{
				recievedStatusReg = receiveUSART();
				dataReceived = 1;
			}
			
			writeString("BT Paired");
		}
		else
		{
			writeString("BT Not Paired");
			if(currentDevice == 0)
			{
				cursorPos(2,0);
				writeString("(BT Off)");
			}
		}
		
		if(btConnected)
		{
			cursorPos(2, 0);
			postureStatus = (recievedStatusReg & 0x0F);
			if (sensitivityLevel == 0)
			{
				writeString("Standby Mode");
				cursorPos(3, 0);
				writeString("Vibration Off");
			}
			else
			{

				if (postureStatus == 0x00) //Conditions for received data
				{
					writeString("Posture OK!");
				}
				else if (postureStatus == 0x01)
				{
					writeString("Leaning FW!");
					fwdLeanCount++;
				}
				else if (postureStatus == 0x02)
				{
					writeString("Leaning BW!");
					bwdLeanCount++;
				}
				else if (postureStatus == 0x04)
				{
					writeString("Leaning Left!");
					leftLeanCount++;
				}
				else if (postureStatus == 0x08)
				{
					writeString("Leaning Right!");
					rightLeanCount++;
				}
				else if (postureStatus == 0x0F)
				{
					writeString("UPSIDE DOWN!");
				}
				else if (postureStatus == 0x05)
				{
					writeString("Leaning FWD!");
					cursorPos(3, 0);
					writeString("Leaning Left!");
					fwdLeanCount++;
					leftLeanCount++;
				}
				else if (postureStatus == 0x09)
				{
					writeString("Leaning FWD!");
					cursorPos(3, 0);
					writeString("Leaning Right!");
					fwdLeanCount++;
					rightLeanCount++;
				}
				else if (postureStatus == 0x06)
				{
					writeString("Leaning BWD!");
					cursorPos(3, 0);
					writeString("Leaning Left!");
					bwdLeanCount++;
					leftLeanCount++;
				}
				else if (postureStatus == 0x0A)
				{
					writeString("Leaning BWD!");
					cursorPos(3, 0);
					writeString("Leaning Right!");
					bwdLeanCount++;
					rightLeanCount++;
				}
				else
				{
					counter++;
					writeString("INVALID DATA");
					cursorPos(3, 0);
					writeString(itoa(postureStatus, currXStr, 10));
					cursorPos(4, 0);
					writeString(itoa(counter, currXStr, 10));
				}
			}
		}
		
		if(currentDevice == 2)
		{
			cursorPos(5,0);
			pinCStatus = PINC;
			if(pinCStatus ==  61)
			{
				writeString("Mvmt!");
			}
			else if(pinCStatus == 59)
			{
				writeString("HI BPM!");
			}
			else if (pinCStatus == 57)
			{
				writeString("Mvmt! HI BPM!");
			}
		}

		updateLCD();
		dataReceived = 0;
}

void updateSettingsMenu()
{
	clearLCD();
	cursorPos(0,1);
	writeString("Back");
	cursorPos(1,1);
	writeString("Light");
	cursorPos(2,1);
	writeString("Sensitiv:");
	writeString(itoa(sensitivityLevel, currXStr, 10));
	cursorPos(3,1);
	writeString("Stats.");
	cursorPos(4,1);
	writeString("Date/Time");
	cursorPos(5,1);
	writeString("Device: ");
	writeString(itoa(currentDevice, currXStr, 10));
	cursorPos(encoderPos, 0);
	writeString("*");
	updateLCD();
	
}

void updateStatisticsMenu()
{
	clearLCD();
	cursorPos(0,0);
	writeString(" Back");
	cursorPos(1,0);
	writeString(" FWD lean:");
	writeString(itoa(fwdLeanCount, currXStr, 10));
	cursorPos(2,0);
	writeString(" BWD lean:");
	writeString(itoa(bwdLeanCount, currXStr, 10));
	cursorPos(3,0);
	writeString(" Left lean:");
	writeString(itoa(leftLeanCount, currXStr, 10));	
	cursorPos(4,0);
	writeString(" Right lean:");
	writeString(itoa(rightLeanCount, currXStr, 10));
	cursorPos(5,0);
	writeString(" Reset Stats");
	cursorPos(encoderPos, 0);
	writeString("*");	
	updateLCD();
}

void updateDateTimeMenu()
{
	clearLCD();
	cursorPos(0,0);
	writeString(" Back");
	
	cursorPos(1,0);
	writeString(" Curr Hr: ");
	Calculate12Hour();
	writeString(itoa(curr12Hour, currXStr, 10));
	writeString(am_pmString);
	
	cursorPos(2,0);
	writeString(" Curr Min: ");
	if (currMin < 10)
	{
		writeString("0");//pad time with zero
	}
	writeString(itoa(currMin, currXStr, 10));
	
	cursorPos(3,0);
	writeString(" Curr Sec: ");
	if (currSec < 10)
	{
		writeString("0");//pad time with zero
	}
	writeString(itoa(currSec, currXStr, 10));
	if (encoderPos >= 3)
	{
		encoderPos = 3;
	}
	
	cursorPos(encoderPos, 0);
	writeString("*");
	updateLCD();
}

void setupExtInterrupts()
{
	DDRD &= 0xE3;
	PORTD |= 0xF3;
	EICRA = 0b00001010; // Falling Edge EXT Interrupt Trigger
    EIMSK = 0b00000001; // Enable EXT Interrupts 0 and 1
	sei();
}

ISR(TIMER1_COMPA_vect)
{	
	sys_tick_flag = 1;
	tickCount++;
	quarterSec++;	
	
}

ISR (INT1_vect) 
{
	currPinDState = PIND;
	inactivityCounter = 0;
	if (dtMode && (hrSet || minSet || secSet))
	{
		if (!(currPinDState & 0x10) && hrSet)
		{
			if(currHour < 23)
			{
				currHour++;//CCW
			}
		}
		else if ((currPinDState & 0x10) && hrSet)
		{
			
			if (currHour > 0)
			{
				currHour--;//CCW
			}
		}
		else if (!(currPinDState & 0x10) && minSet)
		{
			if(currMin < 59)//add encoder direction checks
			{
				currMin++;//CCW
			}	
		}
		else if ((currPinDState & 0x10) && minSet)
		{
			if (currMin > 0)
			{
				currMin--;//CCW
			}
		}
		else if (secSet)
		{
			//Do nothing, seconds are set by clearing to zero with button press
		}
	}
	else if(!(currPinDState & 0x10) && (menuMode == 1))//Fix for increment when spun outside of menuMode
	{
		if(encoderPos < 5)
		{
			encoderPos++;//CCW
		}
		
	}
	else if((currPinDState & 0x10) && (menuMode == 1))
	{
		if (encoderPos > 0)
		{
			encoderPos--;//CCW
		}
		
	}


}

ISR (INT0_vect) 
{
	inactivityCounter = 0;
	if (statMode)
	{
		if(encoderPos == 5)
		{
			fwdLeanCount = 0;
			bwdLeanCount = 0;
			leftLeanCount = 0;
			rightLeanCount = 0;
		}
		else if (encoderPos == 0)
		{
			encoderPos = 1;
			menuBack = 1;			
		}
		
		return;
	}
	else if(dtMode)
	{
		if (encoderPos == 0)
		{
			encoderPos = 1;
			menuBack = 1;
		}
		else if(!hrSet && !minSet && !secSet)
		{
			if(encoderPos == 1)//hr
			{
				hrSet = 1;
			}
			else if(encoderPos == 2)//min
			{
				minSet = 1;
			}		
			else if (encoderPos == 3)
			{
				//secSet = 1;
				currSec = 0;
				quarterSec = 0;
			}
		}
		else
		{
			hrSet = 0;
			minSet = 0;
			secSet = 0;
		}

		
	}
	else if(encoderPos == 0 && menuMode == 1 && !statMode && !dtMode)
	{
		menuMode = 0; //disable menuMode
		EIMSK &= 0xFD; // disable interrupts for rotary knob
		encoderPos = 1;	//reset encoderPos
	}
	else if(encoderPos == 0 && menuMode == 0)
	{
		encoderPos = 1;
		menuBack = 1;
	}
	else if (encoderPos == 1 && menuMode == 0)
	{
		menuMode = 1; //enable encoder EXT Interrupt pin on button press interrupt
		EIFR |= 0x00;//clear Interrupt flags that may retrigger interrupt
		EIMSK |= (1 << INT1); // enable interrupts for rotary knob		
		encoderPos = 1;
		
	}
	else if(encoderPos == 1 && menuMode == 1)
	{
		backlightCounter = 0;
		backlightStatus = 1;
		//set backlight bit
		
	}
	else if(encoderPos == 2)
	{
		if (sensitivityLevel > 2)
		{
			sensitivityLevel = 0;
		}
		else
		{
			sensitivityLevel++;
		}
	}
	else if(encoderPos == 3)
	{
		statMode = 1;//enable statistic view mode
		dtMode = 0;
	}
	else if (encoderPos == 4)
	{
		dtMode = 1;
		statMode = 0;
	}
	else if(encoderPos == 5)
	{
		if (currentDevice > 1)//Current device change from 3 devices to 2 originally 2
		{
			currentDevice = 0;
		}
		else
		{
			currentDevice++;
		}
		
		if (currentDevice == 1 || currentDevice == 2)
		{
			enableBT();
		}
		else
		{
			disableBT();
		}
	}
	EIFR |= 0x00;

}