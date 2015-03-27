	/*
 * PopUPDemo.c
 *
 * Created: 14-Sep-14 01:28:58 PM
 *  Author: Mahmudur
 */ 
 #include <avr/io.h>
 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include <avr/interrupt.h>
 #include <avr/pgmspace.h>
 #include <avr/eeprom.h>
 #include <math.h>
 #ifndef F_CPU
	 #define F_CPU 16000000UL
 #endif
 #include <util/delay.h>

 #include "sbit.h"
 #include "debug.h"
 #include "uart.h"
 #include "circular_queue.h"
 #include "rx_buffer.h"
 
 //LED Pin
 #define LED_BOARD_DDR		DDRB
 #define LED_BOARD_PORT		PORTB
 #define LED_BOARD			PB7		// pin 13
 
 #define LED_EXT_DDR	DDRA
 #define LED_EXT_PORT	PORTA
 #define LED_EXT		PA2			// pin 24
 #define LED_12V		PA3			// pin 25

 // LED Operation
 #define LED_ON(PORT,PIN)		PORT |=  _BV(PIN)
 #define LED_OFF(PORT,PIN)		PORT &= ~_BV(PIN)
 #define LED_TOGGLE(PORT,PIN)	PORT ^= _BV(PIN)
 
 
 #define	BOARD_HIGH_AUTO_DEFAULT TRUE
 /********************************************************************

 Configuration Area.
 Vibration (US) sensor connection.

 in this example it is connected to as follows

 Sensor | MCU
 _____________
 Sig    | PK7

 ********************************************************************/
#define VIBS_DDR		DDRK
#define VIBS_PIN		PINK
#define VIBS_SIG_IN		PK7

/********************************************************************

 Configuration Area.
 Relay Shield connection.

 in this example it is connected to as follows

 Shield| MCU - Arduino
 ____________ _ ____
 D0    | PH4  - PWM7
 D1    | PH3  - PWM6
 D2    | PE3  - PWM5
 D3    | PG5  - PWM4
 
 ********************************************************************/
 #define RELAY_DDR_D0D1		DDRH
 #define RELAY_PORT_D0D1	PORTH
 #define RELAY_D0			PH4
 #define RELAY_D1			PH3
 #define RELAY_DDR_D2		DDRE
 #define RELAY_PORT_D2		PORTE
 #define RELAY_D2			PE3
 #define RELAY_DDR_D3		DDRG
 #define RELAY_PORT_D3		PORTG
 #define RELAY_D3			PG5
 
/********************************************************************
 Motor Operation
 
 Function| D0 | D1 | D2 | D3
 ---------------------------
 Forward | 1  | 0  | 0  | 1
 REvarse | 0  | 1  | 1  | 0

 ********************************************************************/

/********************************************************************

 Configuration Area.
 Encoder connection.

 in this example it is connected to as follows

 Encoder				| MCU         - Arduino
 ____________________________________ _ _______________
 Motor Encoder CLK Pin	| PD0 (INT0)  - Digital PIN 21
 Motor Encoder DT Pin	| PA0		  - Digital PIN 22
 Tuning Encoder SW Pin	| PD0 (INT2)  - Digital PIN 19
 Tuning Encoder CLK Pin	| PA0 (INT1)  - Digital PIN 20
 Tuning Encoder DT Pin	| PA0		  - Digital PIN 23
  
 ********************************************************************/

 #define ENCODER_CLK_SW_DDR		DDRD
 #define ENCODER_DT_DDR			DDRA
 
 #define ENCODER_CLK_SW_PORT	PORTD
 #define ENCODER_DT_PORT		PORTA 
 #define ENCODER_DT_PIN			PINA
 
 #define MOTOR_ENCODER_CLK_PIN	PIND0		// arduino digitalpin 21
 #define MOTOR_ENCODER_DT_PIN	PINA0		// arduino digitalpin 22
 #define MOTOR_ENCODER_CLK		PD0
 #define MOTOR_ENCODER_DT		PA0
 
 #define TUNING_ENCODER_SW_PIN	PIND2		// arduino digitalpin 19
 #define TUNING_ENCODER_CLK_PIN	PIND1		// arduino digitalpin 20
 #define TUNING_ENCODER_DT_PIN	PINA1		// arduino digitalpin 23
 #define TUNING_ENCODER_CLK		PD1
 #define TUNING_ENCODER_SW		PD2
 #define TUNING_ENCODER_DT		PA1
 /************************************************************************/
 /*                                                                      */
 /************************************************************************/
 #define	POSITION_COUNT_LIMIT		3
 #define	HIGH_PWM					12
 #define	LOW_PWM						24
 #define	MOTOR_PWM_CYCLE				2 // 8 * 3 = 24 (200ms period)
 
 #define PWM_LIMIT_FORWARD_ADDRESS			0x01
 #define PWM_LIMIT_FORWARD_SET_FLAG_ADDR	0x00
 #define PWM_LIMIT_FORWARD_SET_FLAG			'f'
 #define PWM_LIMIT_REVERSE_ADDRESS			0x02
 #define PWM_LIMIT_REVERSE_SET_FLAG_ADDR	0x03
 #define PWM_LIMIT_REVERSE_SET_FLAG			'r'
//////////////////////////////////////////////////////////////////////////
#define CURRENT_MEASURE_PIN			0
//////////////////////////////////////////////////////////////////////////
static FILE debug =  FDEV_SETUP_STREAM (debug_stream, NULL, _FDEV_SETUP_WRITE);

enum MotorState{
	MNONE = 0,
	FORWARD = 1,
	REVERSE = 2,
	STOP = 3,
};

volatile enum MotorState motorState = MNONE;
volatile uint16_t motor_move_count = 0;
volatile uint16_t motor_move_sec_count = 0;
volatile uint16_t motor_forward_move_count = 0;
volatile uint16_t motor_reverse_move_count = 0;
volatile uint8_t motor_moving_forward = 0;
volatile uint8_t motor_moving_reverse = 0;
volatile uint8_t motor_forward_moving_state_count = 0;
volatile uint8_t motor_reverse_moving_state_count = 0;
volatile uint8_t motor_duty_cycle_state = 0;
volatile uint8_t limit_switch_high_flag = FALSE;
volatile uint8_t limit_switch_low_flag = FALSE;
volatile uint8_t board_hit_flag = FALSE;
volatile uint8_t raspberry_pi_ready = FALSE;
volatile uint8_t topu = 0;
volatile uint16_t pwm_limit_forward = 0;
volatile uint16_t pwm_limit_reverse = 0;
volatile uint16_t hard_motor_delay = 0;

enum BoardPosition{
	BNONE,
	HIGH,
	LOW
};
volatile enum BoardPosition boardPosition = BNONE;
volatile uint16_t board_low_position_count = 0;
volatile uint16_t board_low_position_sec_count = 0;
volatile uint8_t boardHighAuto = BOARD_HIGH_AUTO_DEFAULT;

// timr0
volatile uint16_t timr0_count = 0;

//timr1
volatile uint16_t timr1_count = 0;

// led
volatile uint8_t led_state = 0;

// encoder push button
volatile uint8_t encoder_push_button_flag = 0;

// motor encoder
volatile uint8_t motor_move_direction = 0;
volatile uint8_t motor_turn_done = 0;
volatile int8_t motor_position_count = 0;
volatile int8_t motor_position_count_temp = 0;

// tuning encoder
volatile uint8_t tuning_move_direction = 0;
volatile uint8_t tuning_turn_done = 0;
volatile uint8_t tuning_position_count = 0;
volatile uint8_t tuning_position_count_temp = 0;

volatile uint8_t motor_position_count_limit = 0;

// serial
uint8_t rx_command_flag = 0;
uint8_t rx_char_count = 0;
#define RX_BUFF	64
char rx_buffer[64];


volatile uint16_t adcValue = 0;
volatile uint16_t sensorValue = 0;
volatile long current = 0;

//////////////////////////////////////////////////////////////////////////

void delay_ms(uint16_t ms);
void delay_us(uint16_t us);
void initLED(void);
void setMotorStop(uint8_t soft);
void motorStopSoft(void);
void motorStop(void);
void setMotorForward(void);
void motorForward(void);
void setMotorReverse(void);
void motorReverse(void);
void initRelaySchield(void);
void init_tmr0(void);
void init_timer1(void);
void initEncoder(void);
void InitDevice(void);
void InitADC(void);
uint16_t adcRead(uint8_t adcx);
void hardMotorStop(void);
void hardMotorForward(void);
void hardMotorReverse(void);
void sendDeviceReady(void);
void sendDone(void);
void sendAck(void);

//////////////////////////////////////////////////////////////////////////
ISR(USART0_RX_vect){ // interrupt service routine for UART0 -> Debug
	unsigned char rx_char = UDR0;
	
	switch(rx_command_flag){
		case 0:
		if(rx_char == '\n'){
			rx_command_flag = 1;
			rx_char_count = 0;
		}
		break;
		
		case 1:
		if(rx_char == '\r'){
			rx_command_flag = 0;
			rx_buffer[rx_char_count] = '\0';
			if (rx_char_count>0)
			{
				if(strncmp("down",rx_buffer,4)== 0){
					LED_ON(LED_EXT_PORT,LED_EXT);
					LED_ON(LED_BOARD_PORT,LED_BOARD);
					sendAck();
					setMotorReverse();
					motorState = REVERSE;
					timr1_count = 0;
					motor_position_count = 0;
				}else if (strncmp("ready",rx_buffer,5)== 0){
					raspberry_pi_ready = TRUE;
				}else if(strncmp("frw:",rx_buffer,4)== 0){
					pwm_limit_forward = atoi(&rx_buffer[4]);
					eeprom_write_byte((uint8_t*)PWM_LIMIT_FORWARD_ADDRESS,(uint8_t)pwm_limit_forward);
					eeprom_busy_wait();
					eeprom_write_byte((uint8_t*)PWM_LIMIT_FORWARD_SET_FLAG_ADDR,PWM_LIMIT_FORWARD_SET_FLAG);
					eeprom_busy_wait();
					printf("frw:%d\n",pwm_limit_forward);
				}else if (strncmp("delay:",rx_buffer,6)== 0){
					hard_motor_delay = atoi(&rx_buffer[6]);
					//if(hard_motor_delay > 244){
						//hard_motor_delay = 244;
					//}
					printf("delay:%d\n",hard_motor_delay);
				}else if(strncmp("tone",rx_buffer,4)== 0){// tuning done
					if(encoder_push_button_flag== 0){
						encoder_push_button_flag = 1;
					}
					motor_position_count = 0;
				}else if(strncmp("m:",rx_buffer,2)== 0){
					switch(rx_buffer[2]){
						case 'F':
						hardMotorForward();
						delay_ms(hard_motor_delay);
						hardMotorStop();
						break;
						
						case 'R':
						hardMotorReverse();
						delay_ms(hard_motor_delay);
						hardMotorStop();
						break;
					}
					printf("%s\n",rx_buffer);
				}else if (strncmp("led:",rx_buffer,4)== 0)
				{
					switch(rx_buffer[4]){
						case '0':
							//12v led off
							LED_OFF(LED_EXT_PORT,LED_12V);
						break;
						
						case '1':
							//12v led on
							LED_ON(LED_EXT_PORT,LED_12V);
						break;
					}
				}else if(strncmp("pos:",rx_buffer,4)== 0){
					motor_position_count_limit = atoi(&rx_buffer[4]);
					printf("pos:%d\n",motor_position_count_limit);
				}else if(strncmp("rev:",rx_buffer,4)== 0){
					pwm_limit_reverse = atoi(&rx_buffer[4]);
					eeprom_write_byte((uint8_t*)PWM_LIMIT_REVERSE_ADDRESS,(uint8_t)pwm_limit_reverse);
					eeprom_busy_wait();
					eeprom_write_byte((uint8_t*)PWM_LIMIT_REVERSE_SET_FLAG_ADDR,PWM_LIMIT_REVERSE_SET_FLAG);
					eeprom_busy_wait();
					printf("rev:%d\n",pwm_limit_reverse);
				}
			}
			rx_char_count = 0;
			
		}else{
			rx_buffer[rx_char_count++] = rx_char;
			if(rx_char_count > RX_BUFF){
				rx_char_count = 0;
				printf("\r\nERR: 001\r\n");
			}
		}
		break;
	}
}

//Interrupt Service Routine for INT0 - Motor CLK
ISR(INT0_vect){
	if(ENCODER_DT_PIN & _BV(MOTOR_ENCODER_DT_PIN))
	{
		motor_position_count++;
	}
	else
	{
		motor_position_count--;
	}
	motor_turn_done = 1;
}

//Interrupt Service Routine for INT1 - Tuning CLK
ISR(INT1_vect){
	if(ENCODER_DT_PIN & _BV(TUNING_ENCODER_DT_PIN))
	{
		tuning_move_direction = 0;
	}
	else
	{
		tuning_move_direction = 1;
	}
	tuning_turn_done = 1;
}

//Interrupt Service Routine for INT2 - Tuning SW
ISR(INT2_vect){
	if(encoder_push_button_flag== 0){
		encoder_push_button_flag = 1;
	}
	motor_position_count = 0;
}


ISR(TIMER0_OVF_vect){	
	if(encoder_push_button_flag == 1)
	{
		
	}
	else
	{
		if(tuning_turn_done == 1){
			tuning_turn_done = 0;
			if(tuning_move_direction == 1)
			{
				hardMotorForward();
				delay_ms(hard_motor_delay);
				hardMotorStop();
				printf("m:F\n");
			}
			else
			{
				hardMotorReverse();
				delay_ms(hard_motor_delay);
				hardMotorStop();
				printf("m:R\n");
			}
		}
	}
	//
	//timr0_count++;
	//if(timr0_count == 244){
		//timr0_count = 0;		
		//// LED blink
		//if(led_state == 0){
			//led_state = 1;
			//LED_ON;
		//}else{
			//led_state = 0;
			//LED_OFF;
		//}
	//}
}

ISR(TIMER1_OVF_vect){//interrupt service routine for TIMER 1 -> GSM Parser
	//if (motor_moving_forward == TRUE){
		//if(motor_move_count >= pwm_limit_forward/2){
			//if(current > 5000){
				//motor_move_count = pwm_limit_forward;
			//}
		//}
	//}
}

// delay for a minimum of <ms>
void delay_ms(uint16_t ms){
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

//delay for a minimum of <us>
void delay_us(uint16_t us){
	while(us){
		_delay_us(0.96);
		us--;
	}
}

void initLED(void){	
	LED_EXT_DDR |= _BV(LED_EXT);
	LED_EXT_DDR |= _BV(LED_EXT);
	LED_BOARD_DDR |= _BV(LED_BOARD);
	LED_OFF(LED_EXT_PORT,LED_EXT);
	LED_OFF(LED_EXT_PORT,LED_12V);
	LED_OFF(LED_BOARD_PORT,LED_BOARD);
}

void motorStopSoft(void){
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D0) & ~_BV(RELAY_D1);
	RELAY_PORT_D2 &= ~_BV(RELAY_D2);
	RELAY_PORT_D3 &= ~_BV(RELAY_D3);
}

void motorStop(void){
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D0) & ~_BV(RELAY_D1);
	RELAY_PORT_D2 |= _BV(RELAY_D2);
	RELAY_PORT_D3 |= _BV(RELAY_D3);
}

void setMotorStop(uint8_t soft){
	motor_moving_forward = FALSE;
	motor_moving_reverse = FALSE;
	motor_move_count = 0;
	motor_forward_moving_state_count = 0;
	motor_reverse_moving_state_count = 0;
	motor_duty_cycle_state = 0;
	motor_forward_move_count = 0;
	motor_reverse_move_count = 0;
	if(soft == 1)
	{
		motorStopSoft();
	}
	else
	{
		motorStop();
	}
	motorState = MNONE;
	topu = 1;
}

void motorForward(void){
	RELAY_PORT_D0D1 |= _BV(RELAY_D0);
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D1);
	RELAY_PORT_D2 &= ~_BV(RELAY_D2);
	RELAY_PORT_D3 |= _BV(RELAY_D3);
}

void setMotorForward(void){
	motor_forward_move_count = 0;
	motor_move_count = 0;
	motor_move_sec_count = 0;
	motor_duty_cycle_state = 0;
	boardPosition = BNONE;
	motorForward();
	motor_forward_moving_state_count = 0;
	motor_moving_forward = TRUE;
	motor_moving_reverse = FALSE;
}

void motorReverse(void){
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D0);
	RELAY_PORT_D0D1 |= _BV(RELAY_D1);
	RELAY_PORT_D2 |= _BV(RELAY_D2);
	RELAY_PORT_D3 &= ~_BV(RELAY_D3);
}

void setMotorReverse(void){
	motor_move_count = 0;
	motor_reverse_move_count = 0;
	motor_move_sec_count = 0;
	boardPosition = BNONE;
	motorReverse();
	motor_reverse_moving_state_count = 0;
	motor_moving_reverse = TRUE;
	motor_moving_forward = FALSE;
}

void initRelaySchield(void){
	// setting pins as output
	RELAY_DDR_D0D1 |= _BV(RELAY_D0) | _BV(RELAY_D1);
	RELAY_DDR_D2 |= _BV(RELAY_D2);
	RELAY_DDR_D3 |= _BV(RELAY_D3);
	// setting all pin low
	setMotorStop(1);
	//printf("Relay Shield Initialized...\n");
}

/* Interrupt at every (1/(FCPU/256))x256 sec
 * So, for 16MHz:
 *		->	16MHz/256 = 62500Hz
 *		->	1/62500 = 1.6 x 10^(-5)
 *		->	(1.6 x 10^(-5)) x 256 = 4.096x10^(-3)
 *		-> 1 sec/4.096x10^(-3) = 244
 *		
 **/
void init_tmr0(void){
	timr0_count = 0;
	motor_move_sec_count = 0;
	TCCR0B |= (1<<CS02) | (0<<CS01) | (0<<CS00); //Prescaler = FCPU/256
	TIMSK0 |= (1<<TOIE0); //Enable Overflow Interrupt Enable
	TCNT0 = 0;
	//printf("Timer 0 Initialized\n");	
}

/*
 * Overflow Interrupt at every 0.004096 sec
 * Count 244.140625 times in the ISR to get 1 second
 */
void init_timer1(void)
{	
	timr1_count = 0;
	TCCR1B |= (0<<CS12) | (0<<CS11) | (1<<CS10); //Prescaler = FCPU/1
	TIMSK1 |= (1<<TOIE1); //Enable Overflow Interrupt Enable
	TCNT1 = 0; //Initialize Counter
	//printf("Timer 1 Initialized\n");
}

void sendDone(void){
	printf("done\n");
	LED_OFF(LED_EXT_PORT,LED_EXT);
	LED_OFF(LED_BOARD_PORT,LED_BOARD);
	board_hit_flag = TRUE;
}

void sendAck(void){
	printf("ack\n");
}

void sendDeviceReady(void){
	printf("Device Ready...\n");
}

void InitADC(void){
	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	// Enable the ADC
	ADCSRA |= _BV(ADEN);
}

uint16_t adcRead(uint8_t adcx) {
	///* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 //* the binary representations of the numbers of the pins so we can
	 //* just 'OR' the pin's number with ADMUX to select that pin.
	 //* We first zero the four bits by setting ADMUX equal to its higher
	 //* four bits. */
	//ADMUX	&=	0xf0;
	//ADMUX	|=	adcx;
	//
	///* This starts the conversion. */
	//ADCSRA |= _BV(ADSC);
	//
	///* This is an idle loop that just wait around until the conversion
	 //* is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 //* set above, to see if it is still set.  This bit is automatically
	 //* reset (zeroed) when the conversion is ready so if we do this in
	 //* a loop the loop will just go until the conversion is ready. */
	//while ( (ADCSRA & _BV(ADSC)) );
	//
	///* Finally, we return the converted value to the calling function. */
	
	// Set ADC channel
	ADMUX=(ADMUX&0xF0)|adcx;

	// Start A2D Conversions
	ADCSRA |= (1 << ADSC);

	while(!(ADCSRA & (1<<ADIF)));

	ADCSRA|=(1<<ADIF);
	
	return ADC;
}

void initEncoder(void){
	ENCODER_CLK_SW_DDR &= ~_BV(MOTOR_ENCODER_CLK);
	ENCODER_DT_DDR &= ~_BV(MOTOR_ENCODER_DT);
	ENCODER_CLK_SW_DDR &= ~_BV(TUNING_ENCODER_CLK);
	ENCODER_DT_DDR &= ~_BV(TUNING_ENCODER_DT);
	ENCODER_CLK_SW_DDR &= ~_BV(TUNING_ENCODER_SW);
	ENCODER_CLK_SW_PORT |= _BV(MOTOR_ENCODER_CLK) | _BV(TUNING_ENCODER_SW);// Enable pulled up
	ENCODER_DT_PORT |= _BV(MOTOR_ENCODER_DT);
	
	EIMSK |= _BV(INT0) |  _BV(INT1) | _BV(INT2);
	 // INT0 - Falling edge
	 // INT1 - Falling edge
	 // INT2 - The low level
	EICRA |= _BV(ISC01) | _BV(ISC11);
	EICRA &= ~_BV(ISC20);
	EICRA &= ~_BV(ISC21);
}

void InitDevice(void){
	cli();
	delay_ms(10);
	initEncoder();
	debug_init(debug_BaudValue,1);
	stdout = &debug;
	initLED();
	InitADC();
	// init_tmr0();
	init_timer1();
	initRelaySchield();	
	delay_ms(10);
	sei();
}

int main(void)
{
	board_hit_flag = FALSE;
	raspberry_pi_ready = FALSE;
	motorState = MNONE;
	boardPosition = BNONE;
	motor_moving_forward = FALSE;
	motor_moving_reverse = FALSE;
	board_low_position_count = 0;
	board_low_position_sec_count = 0;
	boardHighAuto = BOARD_HIGH_AUTO_DEFAULT;
	motor_forward_moving_state_count = 0;
	motor_reverse_moving_state_count = 0;
	motor_forward_move_count = 0;
	motor_reverse_move_count = 0;
	motor_duty_cycle_state = 0;
	motor_position_count = 0;
	motor_move_direction = 0;
	
	motor_turn_done = 0;
	encoder_push_button_flag = 0;
	hard_motor_delay = 40;
	motor_position_count_limit = POSITION_COUNT_LIMIT;
	
	InitDevice();
	
	pwm_limit_forward = 20;
	if (eeprom_read_byte((const uint8_t*)PWM_LIMIT_FORWARD_SET_FLAG_ADDR) == PWM_LIMIT_FORWARD_SET_FLAG){
		eeprom_busy_wait();
		pwm_limit_forward = (uint16_t)eeprom_read_byte((const uint8_t*)PWM_LIMIT_FORWARD_ADDRESS);
	}
	eeprom_busy_wait();
	printf("pwm_limit_forward:%d\n",pwm_limit_forward);
	
	pwm_limit_reverse = 6;
	if (eeprom_read_byte((const uint8_t*)PWM_LIMIT_REVERSE_SET_FLAG_ADDR) == PWM_LIMIT_REVERSE_SET_FLAG){
		eeprom_busy_wait();
		pwm_limit_reverse = (uint16_t)eeprom_read_byte((const uint8_t*)PWM_LIMIT_REVERSE_ADDRESS);
	}
	eeprom_busy_wait();
	printf("pwm_limit_reverse:%d\n",pwm_limit_reverse);
	
	delay_ms(1000);
	
	// waits until tuning button is pressed
	//while(!encoder_push_button_flag){
		//delay_ms(250);
		//LED_TOGGLE(LED_EXT_PORT,LED_EXT);
		//LED_TOGGLE(LED_BOARD_PORT,LED_BOARD);
	//}
	LED_OFF(LED_EXT_PORT,LED_EXT);
	LED_OFF(LED_BOARD_PORT,LED_BOARD);
	
	delay_ms(1000);
	
	sendDeviceReady();
	topu = 0;;
    while(TRUE)
    {
		delay_ms(50);
		adcValue = adcRead(CURRENT_MEASURE_PIN);
		sensorValue = adcValue;
		if( sensorValue < 103 ) sensorValue = 102;
		else if( sensorValue > 921 ) sensorValue = 922;
		sensorValue -= 102;
		// calculate current using integer operation
		current = ( ( sensorValue * 49951 ) >> 10 ) - 20000;
		if((current>1000 || current < -1000) && adcValue != 1023){
			printf("Current: %d mV [%d]\n",(int)current,adcValue);
		}
		//if(currentPickState == 1){
			//currentPickState = 0;
			//current_previous = 0;
			//printf("Motor Blocked\n");
		//}
		//printf("adcValue: %d\n",adcValue);
		if (motor_moving_forward == TRUE){		
			// motor will move only for its move time limit
			motor_move_count++;
			if(motor_move_count >= pwm_limit_forward/2){
				if(current >= 4000){
					motor_move_count = pwm_limit_forward;
				}
			}
			if(motor_move_count == pwm_limit_forward){
				motor_move_count = 0;
				motor_forward_moving_state_count = 0;
				motor_forward_move_count = 0;
				boardPosition = HIGH;
				motorState = STOP;
				setMotorStop(0);
				//delay_ms(1000);// hold few second for stability
				//send done
				sendDone();
				printf("Motor forward Move time limit reached!\n");
			}
			//motor_forward_move_count++;
			//if(motor_forward_move_count == (pwm_limit*3)){
				//motor_forward_move_count = 0;
				//motorForward();
				////if(motor_forward_moving_state_count == 0){
					////motor_duty_cycle_state++;
					////if(motor_duty_cycle_state == 2){
						////motor_duty_cycle_state = 0;
						////motor_forward_moving_state_count = 1;
					////}
					////motorStopSoft();
					////printf("MFMTP\n");
				////}else {
					////motor_forward_moving_state_count = 0;
					////motorForward();
					////printf("MFM\n");
				////}
			//}
		}
		// reverse
		if(motor_moving_reverse == TRUE){
			motor_move_count++;
			if(motor_move_count == pwm_limit_reverse){//(244/4)=61 , (244/3)=81
				motor_move_count = 0;
				motor_reverse_moving_state_count = 0;
				motor_reverse_move_count = 0;
				boardPosition = LOW;
				board_low_position_count = 0;
				motorState = STOP;
				setMotorStop(0);
				printf("Motor reverse Move time limit reached!\n");
			}
					
			//motor_reverse_move_count++;
			//if(motor_reverse_move_count == pwm_limit){
				//motor_reverse_move_count = 0;
				//motorReverse();
				////if(motor_forward_moving_state_count == 0){
					////motor_duty_cycle_state++;
					////if(motor_duty_cycle_state == 2){
						////motor_duty_cycle_state = 0;
						////motor_forward_moving_state_count = 1;
					////}
					////motorStopSoft();
					////printf("MRMTP\n");
				////}else {
					////motor_forward_moving_state_count = 0;
					////motorReverse();
					////printf("MRM\n");
				////}
			//}
		}
		
		// checking if board is at low position and
		// waiting till its stay limit to start for high position
		if (boardPosition==LOW && boardHighAuto == TRUE){
			board_low_position_count++;
			if(board_low_position_count == 122){ // 244(=1s)/1000ms = 0.244(=1ms) , 100 ms = 24.4
				board_low_position_count = 0;
				motorState = FORWARD;
				//motor_position_count = 0;
				setMotorForward();
				printf("BLPSTLR\n");
			}
		}
		if(topu == 1)
		{
			topu = 0;
			printf("Stopped: %d\n",motor_position_count);
		}
		if (motor_turn_done == 1)
		{
			motor_turn_done = 0;
			printf("MPC:%d,d:%d\n",motor_position_count,motor_move_direction);
		}
    }
}

void hardMotorStop(void){
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D0) & ~_BV(RELAY_D1);
	RELAY_PORT_D2 &= ~_BV(RELAY_D2);
	RELAY_PORT_D3 &= ~_BV(RELAY_D3);
}

void hardMotorForward(void){
	RELAY_PORT_D0D1 |= _BV(RELAY_D0);
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D1);
	RELAY_PORT_D2 &= ~_BV(RELAY_D2);
	RELAY_PORT_D3 |= _BV(RELAY_D3);
}

void hardMotorReverse(void){
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D0);
	RELAY_PORT_D0D1 |= _BV(RELAY_D1);
	RELAY_PORT_D2 |= _BV(RELAY_D2);
	RELAY_PORT_D3 &= ~_BV(RELAY_D3);
}