/*
 * atmega328_I2ctest.c
 *
 * Created: 4/24/2018 10:08:23 PM
 * Author : Hecto
 */ 

#define BAUDRATE 38400
#define USART_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

void setupPWM();//declare PWM functions
void enablePWM();
void disablePWM();

void setupIMU();//declare IMU functions
void getIMUData();

void setupUSART();//declare USART functions
void sendUSART(unsigned char dataOut);
unsigned char receiveUSART();

void i2c_init();//declare 
uint8_t i2c_start(uint8_t address);
void i2c_writeByte(uint8_t data);
uint8_t i2c_read_ack();
uint8_t i2c_readBytes(uint8_t address, uint8_t* data, uint8_t numBytes);
void i2c_stop();

int8_t receiveBuffer[6] = {0};
int8_t currX = 0;
int8_t currY = 0;
int8_t currZ = 0;
int8_t storedX = 0;
int8_t storedY = 0;
int8_t storedZ = 0;
int8_t currDiffY = 0;
int8_t currDiffZ = 0;
char currXStr[12];
char currYStr[12];
char currZStr[12];
unsigned char statusReg = 0;
unsigned char wrongOrientation = 1; //assume incorrect orientaion on startup
unsigned char pinBStatus = 0;
unsigned char sensitivityLevel = 0;
unsigned char receivedData = 0;

enum system_states {initial, setup, orientation_guard, standby, IMU_read, offset_calc, update_status} system_state;

void sys_tick()
{
	switch(system_state)
	{
		case initial:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<2);// For displaying binary value of current state through LEDs for debugging purposes
			system_state = setup;
			break;
			
		case setup:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<2);
			system_state = orientation_guard;
			break;
			
		case orientation_guard:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<2);
			if(wrongOrientation)
			{
				system_state = orientation_guard;
			}
			else
			{
				system_state = standby;
			}
			break;
			
		case standby:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<2);
			if(!(pinBStatus & 0x01))//some button is pressed
			{
				storedY = currY;
				storedZ = currZ;
				system_state = IMU_read;
				getIMUData();
				enablePWM();
				_delay_ms(1000);
				disablePWM();//implement near 100 percent duty cycle
			}
			else
			{
				system_state = standby;
			}
			break;
			
		case IMU_read:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<2);
			system_state = offset_calc;
			break;
			
		case offset_calc:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<2);
			pinBStatus = PINB;
			system_state = update_status;
			break;
			
		case update_status:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<2);
			pinBStatus = PINB;
			system_state = IMU_read;
			break;
			
		default:
		PORTD &= 0xE3;
		PORTD |= (((uint8_t)(system_state + 1))<<1);
			system_state = initial;
			break;
	}
		
	switch(system_state)
	{
		case initial:
		wrongOrientation = 0;
			break;
		
		case setup:
			setupPWM();
			//setupScreen();
			setupUSART();
			setupIMU();
			//getIMUData();//throwaway read
			DDRB &= 0xFE;
			PORTB |= 0x01;//setup B0 for button input
			break;
		
		case orientation_guard:
			getIMUData();
			if(currX > 0)//IMU orientation is reversed
			{
				wrongOrientation = 1;
				enablePWM();
				// _delay_ms(1000);
			}
			else
			{
				wrongOrientation = 0;
				disablePWM();
			}
			break;
		
		case standby:
			pinBStatus = PINB;
			getIMUData();
			
			break;
		
		case IMU_read:
			getIMUData();
			break;
		
		case offset_calc:
			statusReg &= 0xF0;
			
			currDiffY = abs(currY - storedY);
			if(currDiffY > (5 * (sensitivityLevel + 1) ) )
			//if(0)
			{
				if(currY > storedY)
				{
					//leaning left
					statusReg |= 0x04;
				} 
				else
				{
					//leaning right
					statusReg |= 0x08;
				}
			}
			
			currDiffZ = abs(currZ - storedZ);
			if(currDiffZ > ( 5 * (sensitivityLevel + 1) ) )
			//if(0)
			{
				if(currZ > storedZ)
				{
					//leaning forward
					statusReg |= 0x01;
				} 
				else
				{
					//leaning backward
					statusReg |= 0x02;
				}
			} 
			
			if(currX > 0)
			//if(0)
			{
				statusReg |= 0x0F;
			}
			
			if ((statusReg & 0x0F) >= 1)
			{
				enablePWM();//enablePWM();
			}
			else
			{
				disablePWM();
			}
			break;
		
		case update_status:
			if(!(pinBStatus & 0x01))
			{
				storedY = currY;
				storedZ = currZ;
				statusReg |= 0x10;//set calibration event flag
				statusReg &= 0xF0;
				disablePWM();
				PORTD |= 0x1C;
				_delay_ms(1000);
				enablePWM();
				_delay_ms(1000);
				disablePWM();
				
				
			}
			//statusReg = 0xF0;
			//sendUSART(statusReg);
			if(UCSR0A & (1<<RXC0))//usart avalible
			{
				receivedData = receiveUSART();//set recieved data
				statusReg &= (receivedData | 0x1F);//bitmask update statusreg sensitivity
				sendUSART(statusReg);//send local statusreg over usart
			}
			statusReg &= 0xEF;//clear calibration event flag
			break;
		
		default:
		PORTD |= 0x1C;
		break;
	}

}


int main(void)
{
	DDRD |= 0x1C;
	PORTD &= 0xE3;
	//PORTD |= 0x1C;
	system_state = initial;
	//setupPWM();
	//enablePWM();
	
    while (1) 
    {
		
		_delay_ms(250);
		sys_tick();
		

    }
}

void setupPWM()
{
	DDRB |= 0xFD; //Enable pins required for 16-bit PWM as output
	ICR1 = 0xFFFF;
	OCR1A = 0x7FFF;
	OCR1B = 0x7FFF; //Set registers for 50% duty cycle
	TCCR1B |= (1 << WGM12)|(1 << WGM13);
	TCCR1B |= (1 << CS11) | (1 << CS10); //Prescaler of 64, yielding ~2Hz Freq with 8MHz clock
}

void enablePWM()
{
	TCCR1A |= (1 << COM1A1)|(1 << COM1B1) | (1 << WGM11);
}

void disablePWM()
{
	TCCR1A = 0;
}


void setupIMU()
{
	i2c_init();
	i2c_start(0x68<<1);
	i2c_writeByte(0x6B);//IMU Initialization Commands
	i2c_writeByte(0x00);
	i2c_stop();//send stop
	
	i2c_start(0x68<<1);//throwaway read
	i2c_writeByte(0x3B);
	i2c_readBytes(0x68<<1, receiveBuffer, 6);
}

void getIMUData()
{
	i2c_start(0x68<<1);
	i2c_writeByte(0x3B);
	i2c_readBytes(0x68<<1, receiveBuffer, 6);
	//currX = (temp[0]<<8)|(temp[1]);//for use with 16bit variables, increases precision
	//currY = (temp[2]<<8) |(temp[3]);
	//currZ = (temp[4]<<8)|(temp[5]);
	
	currX = (receiveBuffer[0]);
	currY = (receiveBuffer[2]);
	currZ = (receiveBuffer[4]);
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
	return UDR0;
}

void i2c_init()
{
	TWBR = (uint8_t)TWBR_val;
}

uint8_t i2c_start(uint8_t address)
{
	TWCR = 0;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while( !(TWCR & (1<<TWINT)) );
	
	if((TWSR & 0xF8) != TW_START){ return 1; }
	
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while( !(TWCR & (1<<TWINT)) );
	
	uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	
	return 0;
}

void i2c_writeByte(uint8_t data)
{
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while( !(TWCR & (1<<TWINT)) );
}

uint8_t i2c_read_ack()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while( !(TWCR & (1<<TWINT)) );
	return TWDR;
}


uint8_t i2c_readBytes(uint8_t address, uint8_t* data, uint8_t numBytes)
{
	if (i2c_start(address | 0x01)) return 1;
	
	for (uint8_t i = 0; i < numBytes; i++)
	{
		data[i] = i2c_read_ack();
	}
	
	i2c_stop();
	
	return 0;
}


void i2c_stop()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}


































