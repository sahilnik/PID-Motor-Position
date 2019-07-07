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
void initMotor(void);
void initEncoder(void);
void Motor_Clockwise(float input);
void Motor_CounterClockwise(float input);
void initUSART(void);
void USART_Transmit(unsigned char data);
void StoreSerial(char ToWrite);
void SerialWriteNumber(void);
void PrintDelay (float varA, float varB);
void PIDController(float measured_state);
float Enc_to_Degrees(float raw_count);

/* Variable definitions */
static float duty = 0.0;
static float prev_duty = 0.0;

char SerialBuffer[TX_Buffer];
static uint8_t ReadPosition = 0;
static uint8_t WritePosition = 0;
char StringArray[100];
static int chararray = 0;

static int32_t lockcount = 0;
static int32_t printcount = 0;

volatile static int EncoderCounts = 0;
static int RawCounts = 0;
static float degrees = 0;

static float control_signal = 0.0;
static int StopFlag_A = 0;
static int StopFlag_B = 0;
static int StopFlag_C = 0;

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Main function
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
int main(void) {
/*	Initialize Encoder, Interrupts, Motor Control, and USART */
	initMotor();
	initEncoder();
	initUSART();
	PIDController(EncoderCounts);
	
	while(1) {
		
		if (StopFlag_A == 0) {
			PIDController(EncoderCounts);
			Enc_to_Degrees(EncoderCounts);
		}
		
		if (control_signal < 0.1 && control_signal > -0.1 && StopFlag_A == 0) { // 0.01
			Motor_CounterClockwise(0);
			PrintDelay(degrees,duty);
			StopFlag_A = 1;
		}
		
		if (StopFlag_A == 1) {
			if (lockcount <= 50000) {
				lockcount++;
			}
			if (lockcount >= 50000) {
				lockcount = 0;
				StopFlag_A = 0;
				EncoderCounts = 0;
			}
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
	Name:			initMotor()
	Input:			void
	Description:	Initialize Power Control registers to allow for CTC timer and use of Compare and Match to generate PWM 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initMotor(void) {
	
/*	Set to fast PWM mode with OCR0A or OCR0B as TOP*/
	TCCR0A |= (1<<COM0A1)|(1<<WGM01)|(1<<WGM00);
	
/*	Set prescaler = 0*/
	TCCR0B |= (1<<CS00);
	
/*	Init counter. Start at 0 for Compare and Match */
	TCNT0 = 0;
	OCR0A = 0;

/*	Set PORTD Pin 6 and Pin 7 as output for motor control */
	DDRD |= (1 << PD6);
	DDRD |= (1 << PD7);
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
	Name:			Motor_Clockwise(float input)
	Input:			Integer value of percentage of duty cycle
	Description:	Set PD7 LOW to dictate Clockwise rotation and set OCR0A to input duty cycle
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void Motor_Clockwise(float input) {
/*	Take input from 0% to 100% as integer 
	Calculate double value of duty cycle based on integer input
	and round double down to nearest integer */
	PORTD &= ~(1 << PD7);
	const float maxdutycycle = 255.0;
	duty = floor((double)(maxdutycycle*input*0.01));

/*	Maximum OCR0A is 255
	If duty cycle exceeds hardware limitations, set to 255 or 0 with respect to input */	
	if (duty > 255) {
		duty = 255;
	} else if (duty < 0) {
		duty = 0;
	}
	OCR0A = duty;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			Motor_CounterClockwise(float input)
	Input:			Integer value of percentage of duty cycle
	Description:	Set PD7 HIGH to dictate Counter-Clockwise rotation and set OCR0A to input duty cycle
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void Motor_CounterClockwise(float input) {
/*	Take input from 0% to 100% as integer 
	Calculate double value of duty cycle based on integer input
	and round double down to nearest integer */
	PORTD |= (1 << PD7);
	const float maxdutycycle = 255.0;
	duty = floor((double)(maxdutycycle*input*0.01));

/*	Maximum OCR0A is 255
	If duty cycle exceeds hardware limitations, set to 255 or 0 with respect to input */	
	if (duty > 255) {
		duty = 255;
	} else if (duty < 0) {
		duty = 0;
	}
	OCR0A = duty;
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
	sprintf(StringArray,"Rotation = %0.1f\n\n\rPWM = %0.1f\n\n\r",varA, varB);
	SerialWrite();
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			PIDController(float Encoder_input)
	Input:			Raw Encoder input
	Description:	PD Controller to provide control input to PWM based on initial temperature,
					current temperature, and desired temperature
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void PIDController(float Encoder_Input) {
/*	Variable setup */
	static float Kp;
	static float Kd;	
	static float Ki;	
/*	475 = 90 deg, 950 = 180 deg, 1425 = 270 deg, 1900 = 360 deg */			
	static float Target_Encoder = 475;	
	
/*	Setting Kp, Kd, Ki gains */
	Kp = 0.04; // 0.08
	Kd = 160.0; // 160.0
	Ki = 0.0000000000005; //0.0000000005; // 0.0000005

	static float derivative = 0.0;
	static float integral = 0.0;
	static float error_signal = 0.0; 
	static float prev_error = 0.0;
	
/*	PID Calculations for Proportional, Derivative, and Integral gains */
	error_signal = Target_Encoder - Encoder_Input;	
	derivative = (error_signal - prev_error);
	integral = (integral + error_signal);
	control_signal = (Kp*error_signal) + (Ki*integral) + (Kd*derivative);

/*	If control signal is positive, set PWM to clockwise, if negative set PWM to counter-clockwise */
	if (control_signal > 0) {
		Motor_Clockwise(control_signal);
	} else if (control_signal < 0) {
		Motor_CounterClockwise(-control_signal);
	}	
	prev_error = error_signal;

}

