#ifdef __AVR_ATmega328P__
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define IS_NTH_BIT_ONE(TARGET, NTH) (((TARGET) & (1 << NTH)) == (1 << NTH))
#define IS_NTH_BIT_ZERO(TARGET, NTH) (!(((TARGET) & (1 << NTH)) == (1 << NTH)))

#define SET_NTH_BIT_TO_ZERO(TARGET, NTH) ((TARGET) &= ~(1 << NTH))
#define SET_NTH_BIT_TO_ONE(TARGET, NTH) ((TARGET) |= (1 << NTH))

#define NORMAL_MODE_VALUE(timer_bit, n_seconds, prescaler) ((int)(((1UL) << (timer_bit)) - ((n_seconds) * ((F_CPU) / (prescaler)))))
#define CTC_MODE_VALUE(n_seconds, prescaler) ((int)(((n_seconds) * ((F_CPU) / (prescaler))) - (1UL)))

#define UBRR_VALUE_LOW_SPEED(UART_BAUDRATE) ((unsigned char)(((F_CPU)/((UART_BAUDRATE) * (16UL)))-((double)(1UL))))
#define UBRR_VALUE_DOUBLE_SPEED(UART_BAUDRATE) ((unsigned char)(((F_CPU)/((UART_BAUDRATE) * (8L)))-((double)(1UL))))

char buffer[43]; 

typedef struct {
    char buffer[50];
    int head; // Index where the next character will be written
    int tail; // Index where the next character will be read
    int size;
    int count; // Current number of characters in the buffer
} RingBuffer;

RingBuffer rb;

void ring_buffer_init(RingBuffer *rb) {
    if (rb == NULL) return;
    memset(rb->buffer, 0, sizeof(rb->buffer));
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->size = 50;
}

void enqueue_char(RingBuffer *rb, char data) {
    if (rb->count == rb->size) {
        // Buffer is full, overwrite oldest data (optional, depending on desired behavior)
        rb->tail = (rb->tail + 1) % rb->size; 
        rb->count--; // Decrement count if overwriting
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
    rb->count++;
}

void enqueue_char_array(RingBuffer *rb, char* data, size_t data_length) {
	for (size_t i = 0; i < data_length; i++) {
		enqueue_char(rb, data[i]);
	}
}

char dequeue(RingBuffer *rb) {
    if (rb->count == 0) {
        // Buffer is empty, handle error or return a special value
        return '\0'; // Example: return null character
    }
    char data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    rb->count--;
    return data;
}

void usart_init_interupt_mode()
{
	UCSR0B = (1<<TXEN0) /*enable TX*/ | (1<<RXEN0) /* enable RX */| (1<<UDRIE0) /* Register Empty Interrupt */| (1<<RXCIE0) /* Complete Interrupt Enable */;
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);  // no parity, 1 stop bit, 8-bit data
	// UBRR0 = UBRR_VALUE_LOW_SPEED(9600);

	UCSR0A = (1<<U2X0); //Double speed mode USART0
	UBRR0 = UBRR_VALUE_DOUBLE_SPEED(115200);

	// UBRR0L = (uint8_t)(F_CPU/(115200*16L)-1);
	// UBRR0H = (F_CPU/(115200*16L)-1) >> 8;
} 

// -UBRRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
 
// -  UBRRH = (((F_CPU/BAUD_RATE)/16)-1)>>8; 	// set baud rate
// -  UBRRL = (((F_CPU/BAUD_RATE)/16)-1);

// -https://github.com/search?q=repo%3Aarduino%2FArduinoCore-avr+UBRRL&type=code
// -*/


enum MY_ADC
{
    ZERO=0, ONE
} current = ZERO;

// int const JOYSTICK_AXIS_X = A0;
// int const JOYSTICK_AXIS_Y = A1;


unsigned int x_axis_adc0 = 0;
unsigned int y_axis_adc1 = 0;

int const UP_BTN = 2;
int const DOWN_BTN = 4;
int const LEFT_BTN = 5;
int const RIGHT_BTN = 3;
int const E_BTN = 6;
int const F_BTN = 7;
int const JOYSTICK_BTN = 0; // PB0 == D8

bool is_up_pressed = 0;
bool is_right_pressed = 0;
bool is_down_pressed = 0;
bool is_left_pressed = 0;
bool is_e_pressed = 0;
bool is_f_pressed = 0;
bool is_joystick_pressed = 0;

unsigned int i = 0;


ISR(ADC_vect){
	// up
	// down
	// left
	// right
	// e
	// f
	// btn
	// x
	// y

	if (current == ZERO) {
		x_axis_adc0 = ADCL + (ADCH << 8);
		current = ONE;
		ADMUX |= (1 << MUX0);
	} else if (current == ONE) {
		y_axis_adc1 = ADCL + (ADCH << 8);
		current = ZERO;
		ADMUX &= ~(1 << MUX0);
	}
	
	snprintf(buffer, sizeof(buffer), "u=%d d=%d l=%d r=%d e=%d f=%d j=%d x=%04d y=%04d\n", is_up_pressed, is_down_pressed, is_left_pressed, is_right_pressed, is_e_pressed, is_f_pressed, is_joystick_pressed, x_axis_adc0, y_axis_adc1);
	
	ADCSRA |= (1<<ADSC); //start conversion
}




ISR(USART_UDRE_vect)
{
	UDR0 = buffer[i];
	i = (i + 1) % (sizeof(buffer));
};

void adc_init_interupt_mode()
{
	/*
	| REFS1 | REFS0 | Vref            |                               |
	|-------|-------|-----------------|-------------------------------|
	| 0     | 0     | AREF pin        | Set externally                |
	| 0     | 1     | AVCC pin        | Same as VCC                   |
	| 1     | 0     | Reserved        | -                             |
	| 1     | 1     | Internal 2.56 V | Fixed regardless of VCC value |

	MUX4:0
	00000 ADC0
	00001 ADC1
	00010 ADC2
	00011 ADC3
	00100 ADC4
	00101 ADC5
	00110 ADC6
	00111 ADC7
	*/
	ADMUX =
		(0 << REFS1) | // 7, Reference Selection Bits
		(1 << REFS0) | // 6, Reference Selection Bits
		(0 << ADLAR) | // 5, ADC Left Adjust Results, (usually set 0 for right-adjusting)
		(0 << MUX3) |  // 3, Analog Channel and Gain Selection Bits
		(0 << MUX2) |  // 2, Analog Channel and Gain Selection Bits
		(0 << MUX1) |  // 1, Analog Channel and Gain Selection Bits
		(current << MUX0);   // 0, Analog Channel and Gain Selection Bits
		

	ADCSRA =
		(1 << ADEN) |  // 7, ADC Enable, This bit enables or disables the ADC. Setting this bit to one will enable the ADC, and clearing this bit to zero will disable it even while a conversion is in progress.
		(0 << ADSC) |  // 6, ADC Start Conversion
		(0 << ADATE) | // 5, ADC Auto Trigger Enable
		(0 << ADIF) |  // 4, ADC Interrupt Flag
		(1 << ADIE) |  // 3, ADC Interrupt Enable
		/*
		Prescaler below, use 1 1 1 if not time critcal

		| ADPS2 | ADPS1 | ADPS0 | ADC Clock |
		|-------|-------|-------|-----------|
		| 0     | 0     | 0     | Reserved  |
		| 0     | 0     | 1     | CK/2      |
		| 0     | 1     | 0     | CK/4      |
		| 0     | 1     | 1     | CK/8      |
		| 1     | 0     | 0     | CK/16     |
		| 1     | 0     | 1     | CK/32     |
		| 1     | 1     | 0     | CK/64     |
		| 1     | 1     | 1     | CK/128    |
		*/
		(1 << ADPS2) | // 2, ADC Prescaler Select Bits
		(1 << ADPS1) | // 1, ADC Prescaler Select Bits
		(1 << ADPS0);  // 0, ADC Prescaler Select Bits
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void)
{
    //ring_buffer_init(&rb);


	DDRB = 0;   // make Port B an input
	DDRD = 0b00000011; // make Port B an input

	// pull up
	SET_NTH_BIT_TO_ONE(PORTD, UP_BTN);
	SET_NTH_BIT_TO_ONE(PORTD, DOWN_BTN);
	SET_NTH_BIT_TO_ONE(PORTD, LEFT_BTN);
	SET_NTH_BIT_TO_ONE(PORTD, RIGHT_BTN);
	SET_NTH_BIT_TO_ONE(PORTD, E_BTN);
	SET_NTH_BIT_TO_ONE(PORTD, F_BTN);
	SET_NTH_BIT_TO_ONE(PORTB, JOYSTICK_BTN);

	DDRC = 0;	   // make Port C an input for ADC input
	
	memset(buffer,'\0', sizeof(buffer));
	// memset(tx_buffer,'\0', sizeof(tx_buffer));
	
	adc_init_interupt_mode();
	usart_init_interupt_mode();

	sei(); //enable interrupts
	
	ADCSRA |= (1<<ADSC); //start conversion

	unsigned char current_pd = 0;
	unsigned char current_pb = 0;
	
	while (1)
	{
		current_pd = PIND;
		current_pb = PINB;

		// All buttons have pull-up resistors and pull to ground when pressed.
		is_up_pressed = IS_NTH_BIT_ZERO(current_pd, UP_BTN);
		is_right_pressed = IS_NTH_BIT_ZERO(current_pd, RIGHT_BTN);
		is_down_pressed = IS_NTH_BIT_ZERO(current_pd, DOWN_BTN);
		is_left_pressed = IS_NTH_BIT_ZERO(current_pd, LEFT_BTN);
		is_e_pressed = IS_NTH_BIT_ZERO(current_pd, E_BTN);
		is_f_pressed = IS_NTH_BIT_ZERO(current_pd, F_BTN);
		is_joystick_pressed = IS_NTH_BIT_ZERO(current_pb, JOYSTICK_BTN);

		
		// enqueue(&rb, '');

		// if (is_up_pressed == 1) {
		// 	snprintf(buffer, sizeof(buffer), "u=%d\n", is_up_pressed);	
		// }
		
    // if (i == 0) {
    //   memset(tx_buffer,'\0', sizeof(tx_buffer));
    //   snprintf(tx_buffer, sizeof(tx_buffer), "%s", buffer);
	// 	}
		_delay_ms(10);
	}

	return 0;
}

/*

#include <avr\io.h>

int main (void){
DDRB = 0xFF; //make Port B an output
DDRD = 0xFF; //make Port D an output
DDRA = 0; //make Port A an input for ADC input

ADCSRA= 0x8F; //enable and interrupt select ck/128
ADMUX= 0xC0; //2.56V Vref and ADC0 single-ended
//input right-justified data

while (1); //wait forever
return 0;
}
*/

#else
#include "gtest/gtest.h"
using ::testing::InitGoogleTest;

// Demonstrate some basic assertions.
TEST(MyTest, BasicAssertions)
{
	//   // Expect two strings not to be equal.
	//   EXPECT_STRNE("hello", "world");
	//   // Expect equality.
	//   EXPECT_EQ(7 * 6, 42);
	EXPECT_EQ(UBRR_VALUE(9600), 103);
	EXPECT_EQ(UBRR_VALUE(4800), 207);
}

int main(int argc, char **argv)
{
	InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

#endif
