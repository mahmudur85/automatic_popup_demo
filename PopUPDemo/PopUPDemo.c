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
 #define LED_DDR	DDRB
 #define LED_PORT	PORTB
 #define LED		PB7

 // LED Operation
 #define LED_ON		LED_PORT |=  _BV(LED)
 #define LED_OFF	LED_PORT &= ~_BV(LED)
 #define LED_TOGGLE LED_PORT ^= _BV(LED)

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
#define MOTOR_MOVE_TIME_LIMIT	1 // Secs
/********************************************************************

 Configuration Area.
 Limit Switch connection.

 in this example it is connected to as follows

 Switch| MCU         - Arduino
 ___________________ _ _______________
 HIGH  | PD0 (INT0)  - Digital PIN 21
 LOW   | PD1 (INT1)  - Digital PIN 20
 
 ********************************************************************/

#define LIMIT_SWITCH_DDR	DDRD
#define LIMIT_SWITCH_PORT	PORTD
#define LIMIT_SWITCH_PIN	PIND
#define LIMIT_SWITCH_HIGH	PD0
#define LIMIT_SWITCH_LOW	PD1

#define BOARD_LOW_POSITION_STAY_LIMIT 5 // Sec

#define SMS_PAYLOAD_LEN 160

#define BOARD_HIGH_AUTO_DEFAULT TRUE

 /********************************************************************/

 #define ENCODER_DDR	DDRD
 #define ENCODER_PORT	PORTD
 #define ENCODER_PIN	PIND
 #define ENCODER_CLK	PIND0		// arduino digitalpin 21
 #define ENCODER_DT		PIND1		// arduino digitalpin 20
 #define CLK			PD0
 #define DT				PD1
 
//////////////////////////////////////////////////////////////////////////
static FILE debug =  FDEV_SETUP_STREAM (debug_stream, NULL, _FDEV_SETUP_WRITE);
//static FILE uart1 =  FDEV_SETUP_STREAM (uart_stream, NULL, _FDEV_SETUP_WRITE);

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

volatile uint8_t direction = 0;
volatile uint8_t position_count_reset_flag = 0;
volatile uint8_t turn_done = 0;
volatile uint8_t position_count = 0;
volatile uint8_t position_count_temp = 0;

//////////////////////////////////////////////////////////////////////////

void delay_ms(uint16_t ms);
void delay_us(uint16_t us);
void initLED(void);
void setMotorStop(void);
void setMotorForward(void);
void setMotorReverse(void);
void initRelaySchield(void);
void initVibrationSensor(void);
uint8_t boardHighLimitState(void);
uint8_t boardLowLimitState(void);
void init_tmr0(void);
void init_timer1(void);
void InitDevice(void);
uint8_t vibratinSensorState(void);
void hardMotorStop(void);
void hardMotorForward(void);
void hardMotorReverse(void);
void sendDeviceReady(void);
void sendDone(void);
void sendAck(void);

//////////////////////////////////////////////////////////////////////////
ISR(USART0_RX_vect){ // interrupt service routine for UART0 -> Debug
	unsigned char rx_char = UDR0;
	if(rx_char == 'D'){
		LED_ON;
		sendAck();
		setMotorReverse();
		motorState = REVERSE;
		timr1_count = 0;
		position_count = 0;
	}
	
	if(rx_char == 'R'){
		raspberry_pi_ready = TRUE;
	}
}

ISR(USART1_RX_vect){ // interrupt service routine for UART1 -> GSM Module
	unsigned char rx_char = UDR1;
	if(rx_char != 0x00 && rx_char != 0xFF){
		
	}
}

//Interrupt Service Routine for INT0
ISR(INT0_vect){
	
	if(ENCODER_PIN & _BV(ENCODER_DT))
	{
		direction = 0;
	}
	else
	{
		direction = 1;
	}
	turn_done = 1;
}

ISR(TIMER0_OVF_vect){
	// motor will move only for its move time limit
	if (turn_done == 1)
	{
		turn_done = 0;
		position_count++;
		LED_TOGGLE;
	}
	if (motor_moving_forward == TRUE){
		if(	position_count == position_count_temp)
		{
			position_count = 0;
			position_count_temp = 0;
			boardPosition = HIGH;
			motorState = STOP;
			setMotorStop();
		}
		//motor_move_count++;
		//if(motor_move_count == 610){
			//motor_move_count = 0;
			//motor_forward_moving_state_count = 0;
			//motor_forward_move_count = 0;
			//boardPosition = HIGH;
			//motorState = STOP;
			//setMotorStop();
			////delay_ms(1000);// hold few second for stability
			////send done
			//sendDone();
			//printf("Motor forward Move time limit reached!\n");
		//}
		motor_forward_move_count++;
		if(motor_forward_move_count == 12){
			motor_forward_move_count = 0;
			if(motor_forward_moving_state_count == 0){
				motor_duty_cycle_state++;
				if(motor_duty_cycle_state == 2){
					motor_duty_cycle_state = 0;
					motor_forward_moving_state_count = 1;
				}
				RELAY_PORT_D0D1 &= ~_BV(RELAY_D0) & ~_BV(RELAY_D1);
				RELAY_PORT_D2 |= _BV(RELAY_D2);
				RELAY_PORT_D3 |= _BV(RELAY_D3);
				printf("Motor forward Move Temp pause\n");
			}else {
				motor_forward_moving_state_count = 0;
				RELAY_PORT_D0D1 |= _BV(RELAY_D0);
				RELAY_PORT_D0D1 &= ~_BV(RELAY_D1);
				RELAY_PORT_D2 &= ~_BV(RELAY_D2);
				RELAY_PORT_D3 |= _BV(RELAY_D3);
				printf("Motor forward Move phase\n");
			}
		}
	}
	
	if(motor_moving_reverse == TRUE){
		if(position_count == 3){
			position_count_temp = position_count;
			position_count = 0;
			boardPosition = LOW;
			motorState = STOP;
			setMotorStop();
		}
		//motor_move_count++;
		//if(motor_move_count == 184){//(244/4)=61 , (244/3)=81
			//motor_move_count = 0;
			//motor_reverse_moving_state_count = 0;
			//motor_reverse_move_count = 0;
			//boardPosition = LOW;
			//motorState = STOP;
			//setMotorStop();
			//printf("Motor reverse Move time limit reached!\n");
		//}
		motor_reverse_move_count++;
		if(motor_reverse_move_count == 12){
			motor_reverse_move_count = 0;
			if(motor_forward_moving_state_count == 0){
				motor_duty_cycle_state++;
				if(motor_duty_cycle_state == 2){
					motor_duty_cycle_state = 0;
					motor_forward_moving_state_count = 1;
				}
				RELAY_PORT_D0D1 &= ~_BV(RELAY_D0) & ~_BV(RELAY_D1);
				RELAY_PORT_D2 |= _BV(RELAY_D2);
				RELAY_PORT_D3 |= _BV(RELAY_D3);
				printf("Motor reverse Move Temp pause\n");
			}else {
				motor_forward_moving_state_count = 0;
				RELAY_PORT_D0D1 &= ~_BV(RELAY_D0);
				RELAY_PORT_D0D1 |= _BV(RELAY_D1);
				RELAY_PORT_D2 |= _BV(RELAY_D2);
				RELAY_PORT_D3 &= ~_BV(RELAY_D3);
				printf("Motor reverse Move phase\n");
			}
		}
	}
		
	// checking if board is at low position and
	// waiting till its stay limit to start for high position
	if (boardPosition==LOW && boardHighAuto == TRUE){
		board_low_position_count++;
		if(board_low_position_count == 244){
			board_low_position_count = 0;
			board_low_position_sec_count++;
			if (board_low_position_sec_count == BOARD_LOW_POSITION_STAY_LIMIT){
				board_low_position_sec_count = 0;
				motorState = FORWARD;
				setMotorForward();
				printf("Board Low Position Stay time limit reached!\n");
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
	//if(board_hit_flag == TRUE){
		//timr1_count++;
		//if(timr1_count == 488){// 244*2
			//board_hit_flag = FALSE;
			//timr1_count = 0;
			//LED_OFF;
			////// LED blink
			////if(led_state == 0){
				////led_state = 1;
				////LED_ON;
			////}else{
				////led_state = 0;
				////LED_OFF;
			////}
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
	LED_DDR |= _BV(LED);
	LED_OFF;
}

void setINT(uint8_t state){
	if(state == TRUE){
		limit_switch_high_flag = FALSE;
		limit_switch_low_flag = FALSE;
		cli();
		EIMSK |= _BV(INT0) | _BV(INT1);
		EICRA |= _BV(ISC00) | _BV(ISC10);
		sei();
	}else{
		EIMSK &= ~_BV(INT0) & ~_BV(INT1);
	}
}

void setMotorStop(void){
	motor_moving_forward = FALSE;
	motor_moving_reverse = FALSE;
	motor_move_count = 0;
	motor_forward_moving_state_count = 0;
	motor_reverse_moving_state_count = 0;
	motor_duty_cycle_state = 0;
	motor_forward_move_count = 0;
	motor_reverse_move_count = 0;
	//setINT(FALSE);
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D0) & ~_BV(RELAY_D1);
	RELAY_PORT_D2 |= _BV(RELAY_D2);
	RELAY_PORT_D3 |= _BV(RELAY_D3);
	//delay_ms(2000);
	//RELAY_PORT_D2 &= ~_BV(RELAY_D2);
	//RELAY_PORT_D3 &= ~_BV(RELAY_D3);
	motorState = MNONE;
	topu = 1;
}

void setMotorForward(void){
	motor_forward_move_count = 0;
	motor_move_count = 0;
	motor_move_sec_count = 0;
	motor_duty_cycle_state = 0;
	boardPosition = BNONE;
	//setINT(TRUE);
	RELAY_PORT_D0D1 |= _BV(RELAY_D0);
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D1);
	RELAY_PORT_D2 &= ~_BV(RELAY_D2);
	RELAY_PORT_D3 |= _BV(RELAY_D3);
	motor_forward_moving_state_count = 0;
	//motorState = MNONE;
	motor_moving_forward = TRUE;
	motor_moving_reverse = FALSE;
}

void setMotorReverse(void){
	motor_move_count = 0;
	motor_reverse_move_count = 0;
	motor_move_sec_count = 0;
	boardPosition = BNONE;
	//setINT(TRUE);
	RELAY_PORT_D0D1 &= ~_BV(RELAY_D0);
	RELAY_PORT_D0D1 |= _BV(RELAY_D1);
	RELAY_PORT_D2 |= _BV(RELAY_D2);
	RELAY_PORT_D3 &= ~_BV(RELAY_D3);
	motor_reverse_moving_state_count = 0;
	//motorState = MNONE;
	motor_moving_reverse = TRUE;
	motor_moving_forward = FALSE;
}

void initRelaySchield(void){
	// setting pins as output
	RELAY_DDR_D0D1 |= _BV(RELAY_D0) | _BV(RELAY_D1);
	RELAY_DDR_D2 |= _BV(RELAY_D2);
	RELAY_DDR_D3 |= _BV(RELAY_D3);
	// setting all pin low
	setMotorStop();
	//printf("Relay Shield Initialized...\n");
}

void initVibrationSensor(void){
	VIBS_DDR &= ~(1 << VIBS_SIG_IN);
	//printf("Vibration Sensor Initialized...\n");
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
	LED_OFF;
	board_hit_flag = TRUE;
}

void sendAck(void){
	printf("ack\n");
}

void sendDeviceReady(void){
	printf("Device Ready...\n");
}

void InitDevice(void){
	cli();
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
	delay_ms(10);
	ENCODER_DDR &= ~_BV(CLK);
	ENCODER_DDR &= ~_BV(DT);
	debug_init(debug_BaudValue,1);
	//uart_init(uart_BaudValue,1);
	stdout = &debug;
	initLED();
	EIMSK |= _BV(INT0);
	EICRA |= _BV(ISC01);
	init_tmr0();
	init_timer1();
	initVibrationSensor();
	initRelaySchield();
	sei();
}

uint8_t vibratinSensorState(void){
	if (!(VIBS_PIN & (1<<VIBS_SIG_IN))){
		return TRUE;
	}else{
		return FALSE;	
	}
}

uint8_t boardHighLimitState(void){
	if((LIMIT_SWITCH_PIN & (1<<LIMIT_SWITCH_HIGH))){ // check for HIGH state
		return TRUE;
	}else{
		return FALSE;
	}
}

uint8_t boardLowLimitState(void){
	if((LIMIT_SWITCH_PIN & (1<<LIMIT_SWITCH_LOW))){ // check for HIGH state
		return TRUE;
	}else{
		return FALSE;
	}
}

int main(void)
{
	position_count = 0;
	direction = 0;
	position_count = 0;
	topu = 0;
	turn_done = 0;
	InitDevice();
	delay_ms(1000);
	// initially checks if board is at high 
	// or else start motor move to forward
	//if(boardPosition != HIGH){
	
	//printf("Waiting for Rraspberry PI to Ready...\n");
	//while(TRUE){// waits until raspberry pi is ready
		//delay_ms(100);
		//if (raspberry_pi_ready == TRUE){
			//break;
		//}
	//}
	
	// checks if the board is at high when device is starting
	//if(boardHighLimitState() == FALSE){
		//printf("Moving motor Forward to set the board high\n");
		//setMotorForward(); 
	//}else{
		//boardPosition = HIGH;
		//motor_moving_forward = FALSE;
		//motor_moving_reverse = FALSE;
		//printf("Board is initially at high\n");
	//}
	
	sendDeviceReady();
	
    while(TRUE)
    {
		// if board at high position waits for board to hit
		// then start motor move to reverse
		//if(boardHighLimitState() == TRUE){
			//printf("H\n");
		//}else if(boardLowLimitState() == TRUE){
			//printf("L\n");
		//}
		//////////////////////////////////////////////////////////////////////////
		/*
		if(boardHighLimitState() == TRUE && motor_moving_forward == TRUE){
			//if(limit_switch_high_flag == FALSE){
			setMotorStop();
			delay_ms(1000);// hold few second for stability
			//send done
			sendDone();
			motorState = STOP;
			boardPosition = HIGH;
			limit_switch_high_flag = TRUE;
			limit_switch_low_flag = FALSE;
			printf("Board High\n");
			//}
		}else if (boardLowLimitState() == TRUE && motor_moving_reverse== TRUE){
			//if (limit_switch_low_flag == FALSE){
			setMotorStop();
			motorState = STOP;
			boardPosition = LOW;
			limit_switch_low_flag = TRUE;
			limit_switch_high_flag = FALSE;
			board_low_position_count = 0;
			board_low_position_sec_count = 0;
			printf("Board Low\n");
			//}
		}
		*/
		//else if(boardPosition == HIGH && motor_moving_forward == FALSE && motor_moving_reverse == FALSE){
			////delay_ms(10);
			////printf("it is here\n");
			//if (!(VIBS_PIN & (1<<VIBS_SIG_IN))){
				//setMotorReverse();
				//motorState = REVERSE;
				////printf("Board Hit\n");
			//}
		//}
		
		//////////////////////////////////////////////////////////////////////////
		
		// motor state change and move
		//if(motorState == FORWARD){
			//motorState = MNONE;
			//printf("Motor Rotating Forward\n");
			////setMotorForward();
		//}else if(motorState == REVERSE){
			//motorState = MNONE;
			//printf("Motor Rotating Reverse\n");
			////setMotorReverse();
		//}else if(motorState == STOP){
			//motorState = MNONE;
			//printf("Motor Stopped\n");
			////setMotorStop();			
		//}
		
		
		//if(turn_done == 1)
		//{
			//if(direction==1)
			//{
				//position_count++;
			//}
			//else
			//{
				//position_count--;
			//}
			//turn_done = 0;
			//printf("Count: %d\n",position_count);
		//}
		if(topu == 1)
		{
			topu = 0;
			printf("Stopped: %d\n",position_count);
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