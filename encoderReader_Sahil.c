/* Includes */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Defines */
#define F_CPU 16000000
#define BAUD_RATE 9600
#define FOSC 16000000
#define TX_Buffer 255

/* Prototype Function declarations */
void initEncoder(void);
void initUSART(void);
void USART_Transmit(unsigned char data);
void StoreSerial(char ToWrite);
void PrintDelay (float varA, float varB);
float Enc_to_Degrees(float raw_count);

/* Variable definitions */
static float duty = 0.0;
static float prev_duty = 0.0;

char SerialBuffer[TX_Buffer];
static uint8_t ReadPosition = 0;
static uint8_t WritePosition = 0;
char StringArray[100];
static int chararray = 0;

volatile static int EncoderCounts = 0;
static int RawCounts = 0;
static float degrees = 0;
static int printcount = 0;

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Main function
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
int main(void) {
/*	Initialize Encoder, Interrupts, and USART */
	initEncoder();
	initUSART();
	
	while(1) {
		Enc_to_Degrees(EncoderCounts);
		if (printcount < 10000) {
			printcount++;
		} else if (printcount >= 10000) {
			PrintDelay(EncoderCounts,degrees);
			printcount = 0;
		}

	}
	return 0;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			ISR(PCINT2_vect)
	Input:			None
	Description:	Interrupt that fires when change is detected in Pin 3
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
ISR(PCINT2_vect) {

	if (PIND & (1<<PD3)) {
	/*	If pin 3 is high and pin 9 is high (CW rotation) add count */
		if (PINB & (1<<PB1)) {
			EncoderCounts++;
	/*	If pin 3 is high and pin 9 is low (CCW rotation) subtract count */
		} else if ((PINB & (1<<PB1)) == 0) {
			EncoderCounts--;
		}
		
	} else if ((PIND & (1<<PD3)) == 0) {
	/*	If pin 3 is low and pin 9 is low (CCW rotation) subtract count */
		if (PINB & (1<<PB1)) {
			EncoderCounts--;
	/*	If pin 3 is low and pin 9 is low (CW rotation) add count */
		} else if ((PINB & (1<<PB1)) == 0) {
			EncoderCounts++;
		}
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			ISR(PCINT0_vect)
	Input:			None
	Description:	Interrupt that fires when change is detected in Pin 9
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
ISR(PCINT0_vect) {
	
	if (PINB & (1<<PB1)) {
	/*	If pin 9 is high and pin 3 is high (CCW rotation) add count */
		if (PIND & (1<<PD3)) {
			EncoderCounts--;
	/*	If pin 9 is high and pin 3 is low (CW rotation) subtract count */
		} else if ((PIND & (1<<PD3)) == 0) {
			EncoderCounts++;
		}
	} else if ((PINB & (1<<PB1)) == 0) {
	/*	If pin 9 is low and pin 3 is high (CW rotation) add count */
		if (PIND & (1<<PD3)) {
			EncoderCounts++;
	/*	If pin 9 is low and pin 3 is low (CCW rotation) add count */
		} else if ((PIND & (1<<PD3)) == 0) {
			EncoderCounts--;
		}
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initEncoder()
	Input:			void
	Description:	Initialize Interrupt pins for Encoder phase A and B
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initEncoder(void) {
	
/*	Set PD3 and PB1 as input pins */
	DDRD &= ~(1<<PD3); 
	DDRB &= ~(1<<PB1);
	
/*	Enable pullup for PD3 and PB1 */
	PORTD |= (1<<PD3);
	PORTB |= (1<<PB1);
	
/*	Enable PCIE2 and PCIE0 interrupts */ 
	PCICR |= (1 << PCIE2)|(1 << PCIE0);
	
/*	Enable interrupt on Pin PCINT1 = PB1 = Digital Pin 9 */
	PCMSK0 |= (1 << PCINT1);
/*	Enable interrupt on Pin PCINT19 = PD3 = Digital Pin 3 */
	PCMSK2 |= (1 << PCINT19);
	
/*	Enable interrupts */
	sei();
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			Enc_to_Degrees(float raw_count)			//	Encoder Count		Degrees
	Input:			Raw encoder count						//--------------------------------
	Description:	Convert encoder count to degrees.		//		475				90
															//		950				180
															//		1425			270
															//		1900			360
															//		5.27778			1
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
float Enc_to_Degrees(float raw_count) {
	const float deg_conversion = 5.27778;
	degrees = EncoderCounts/deg_conversion;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initUSART(void)
	Input:			None
	Description:	Initialize USART registers, set baud rate, and enable serial communication
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initUSART(void) {
/*	Baud Rate = 9600, Clock speed is 16 MHz, Baud = 103 for 0.2% error
	Baud = ((ClockSpeed/BaudRate/16) - 1)
	Set baud */
	UBRR0H = (unsigned char)(103>>8);
	UBRR0L = (unsigned char)(103);
/*	Enable transmitter and receiver */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //|(1<<TXCIE0)
/*	Set frame format: 8 bit data, 1 stop bit */
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			USART_Transmit(unsigned char data)
	Input:			unsigned character to be transmitted	
	Description:	Waits for empty transmission buffer and places data into UDR0 buffer to be transmitted
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void USART_Transmit(unsigned char data) {
	
	if (chararray == 0) {
/*		Do nothing and wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) ) {
		}
/*		Put data into buffer, sends the data */
		UDR0 = data;
		
	} else if (chararray == 1) {
		
		if (WritePosition < sizeof(StringArray)) {
			WritePosition++;
		}
		
		if (ReadPosition != WritePosition) {
			UDR0 = SerialBuffer[ReadPosition];
			ReadPosition++;
				
			if (ReadPosition >= TX_Buffer) {
				ReadPosition = 0;
			}
		} else if (ReadPosition == WritePosition) {
			ReadPosition = 0;
			WritePosition = 0;
		}
		chararray = 0;
	}

}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			StoreSerial(char ToWrite)
	Input:			Character elements to be written
	Description:	Writes bit to buffer array. Resets array position once buffer limit is reached/exceeded
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void StoreSerial(char ToWrite) {
	SerialBuffer[WritePosition] = ToWrite;
	USART_Transmit(SerialBuffer[WritePosition]);
	
	if (WritePosition >= TX_Buffer) {
		WritePosition = 0;
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			SerialWrite(void)
	Input:			None	
	Description:	Sends each element of StringArray to be stored and transmitted
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void SerialWrite(void) {
	chararray = 1;
	/*	Increment through character area so all elements are sent to StoreSerial(); */
	for (uint8_t n = 0; n < strlen(StringArray); n++) {
		StoreSerial(StringArray[n]);
	}

	/*	Send empty bit until bit is set */
	if (UCSR0A & (1<<UDRE0)) {
		UDR0 = 0;
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			PrintDelay (int varA, int varB)
	Input:			Integer value to be printed 
	Description:	Provides float value within sprintf function to assign to StringArray 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void PrintDelay (float varA, float varB) {
	sprintf(StringArray,"Encoder Reading = %0.1f\n\n\rDegrees of Rotation = %0.1f\n\n\r",varA, varB);
	SerialWrite();
}
