/*
 * ps2_16_linetrace.c
 *
 * Created: 05-12-2016 16:36:38
 * Author : HP
 */


/*
PWM LEFT 	OC1B	PD4
PWM RIGHT 	OC1A	PD5
lSA08 Tx	Rx		PD0
J_PULSE		INT0	PD2
UART En				PD6
MOTOR Dir pins		PORTA - 0,1,2,3

*/


#define F_CPU 14745600UL

/*
//for _delay_us(2.5)
#define up 0b11001110
#define righ 0b10011110
#define lef 0b01111110
#define down 0b00111110
#define select 0b11111100
#define start 0b11100110
#define joyr 0b11110010
#define joyl 0b11111000
#define L2 0b11111100
#define L1 0b11110011
#define R2 0b11111001
#define R1 0b11100111
 
#define triangle 0b11001111
#define squar 0b01111111
#define cros 0b00111111
#define circle 0b10011111

#define r_joy_up 0b00000000
#define r_joy_down 0b01111111
#define r_joy_left 0b00000000
#define r_joy_right 0b11111111

#define l_joy_up 0b00000000
#define l_joy_down 0b11111110
#define l_joy_left 0b00000000
#define l_joy_right 0b11111110
*/
/*#define up 0b11101111
#define righ 0b11011111
#define down 0b10111111
#define lef 0b01111111
#define select 0b11111110
#define joyl 0b11111101
#define joyr 0b11111011
#define start 0b11110111
#define L2 0b11111110
#define R2 0b11111101
#define L1 0b11111011
#define R1 0b11110111

#define triangle 0b11101111
#define circle 0b11011111
#define cros 0b10111111
#define squar 0b01111111

#define r_joy_up 0b00000000
#define r_joy_down 0b11111111
#define r_joy_left 0b00000000
#define r_joy_right 0b11111111

#define l_joy_up 0b00000000
#define l_joy_down 0b11111111
#define l_joy_left 0b00000000
#define l_joy_right 0b11111111
*/
#define up 4
#define righ 5
#define down 6
#define lef 7
#define select 0
#define joyl 1
#define joyr 2
#define start 3
#define L2 0
#define R2 1
#define L1 2
#define R1 3

#define triangle 4
#define circle 5
#define cros 6
#define squar 7

#define r_joy_up 0b00000000
#define r_joy_down 0b11111111
#define r_joy_left 0b00000000
#define r_joy_right 0b11111111

#define l_joy_up 0b00000000
#define l_joy_down 0b11111111
#define l_joy_left 0b00000000
#define l_joy_right 0b11111111

#define motors PORTK
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define baudrate 9600
#define BRX ((F_CPU/(16UL*baudrate))-1)	//baudrate prescaler

#define setvalue 35
#define kp 2.25
#define ki 0
#define kd 1
/*
#define fwd 0b00000101
#define rev 0b00001010
#define cw 0b00000110
#define acw 0b00001001
#define stop 0*/

int j_count=0,piston1=0,piston2=0,piston_pushed1=0,piston_pushed2=0,piston=0;
volatile uint8_t data;
	unsigned char data0, data1, data2, data3, data4, data5;
#define ps2_port		   PORTC
#define ps2_port_ddr       DDRC
#define ps2_port_pin       PINC
/*
#define PSdata             6
#define PScommand          5
#define PSclock            7
#define PSattention        4
*/

#define PSdata            1
#define PScommand         2
#define PSclock           7
#define PSattention       6

#define PBdata            	PB1
#define PBcommand         PB2
#define PBclock           PB7
#define PBattention          PB6

#define sbi(x,y)  x|=(1<<y)     //setbit
#define cbi(x,y)  x&=~(1<<y)//clear bit
/*
#define ackw_1		0x01
#define ckw_1		0x02
*/
#define bot_forword		0b10010110
#define bot_backword    0b01101001
#define bot_left		0b10100101
#define bot_right		0b01011010
#define bot_r_rotate	0b10101010
#define bot_l_rotate	0b01010101
#define bot_stop		0b11111111
#define bot_left_up		0b10111011
#define bot_left_down	0b11101110
#define bot_right_up	0b11011101
#define bot_right_down	0b01111011

#define v_slow11 		0x0C	//0
#define v_slow12		0x0D	//36
#define v_slow13		0x0E	//45
#define v_slow14		0x0F	//55


#define slow11 		0x00//109
#define slow12		0x01//131
#define slow13		0x02//153
#define slow14		0x03//175

#define med11		0x04//197
#define med12		0x05//219
#define med13		0x06//241
#define med14		0x07//263

#define	fast11		0x08//285
#define fast12		0x09//307
#define fast13		0x0A//329
#define fast14		0x0B//351

#define v_slow21 	0xC0
#define v_slow22		0xD0
#define v_slow23		0xE0
#define v_slow24		0xF0

#define slow21 		0x00//109
#define slow22		0x10//131
#define slow23		0x20//153
#define slow24		0x30//175

#define med21		0x40//197
#define med22		0x50//219
#define med23		0x60//241
#define med24		0x70//263

#define	fast21		0x80//285
#define fast22		0x90//307
#define fast23		0xA0//329
#define fast24		0xB0//351


/*
uint8_t bot_little_fast		=0b00000000;
uint8_t bot_slow		=0b01010101;
uint8_t bot_fast		=0b10101010;
uint8_t bot_very_fast		=0b11111111;
*/

/*

//AUTOMAT:
#define ackw_1		0x01
#define ckw_1		0x02
#define ackw_2		0x04
#define ckw_2		0x08
#define ackw_3		0x10
#define ckw_3		0x20
#define ackw_4		0x40
#define ckw_4		0x80

uint8_t bot_r_rotate	=0b10101010;
uint8_t bot_l_rotate	=0b01010101;
uint8_t bot_slowest		=0b00000000;
uint8_t bot_slow		=0b01010101;
uint8_t bot_fast		=0b10101010;
uint8_t bot_very_fast	=0b11111111;
uint8_t bot_stop		=0b11111111;
#define bot_forword ckw_4|ackw_2
#define bot_backword ackw_4|ckw_2
#define bot_left ckw_3|ackw_1
#define bot_right ackw_3|ckw_1

*/
unsigned char gameByte(unsigned char command)
{
	unsigned char i ;
	_delay_us(1);
	unsigned char data = 0x00;                             // clear data variable to save setting low bits later.
	for(i=0;i<8;i++)
	{
		if(command & _BV(i))
		{
			sbi(ps2_port, PScommand);       // bit bang "command" out on PScommand wire.
		}
		else
		{
			cbi(ps2_port, PScommand);
		}
		cbi(ps2_port, PSclock);                             // CLOCK LOW
		_delay_us(7);                                    // wait for output to stabilise

		if((ps2_port_pin & _BV(PSdata)))
		{
			sbi(data, i);                               // read PSdata pin and store
		}
		else
		{
			cbi(data, i);
		}
		sbi(ps2_port, PSclock);                             // CLOCK HIGH
	}
	sbi(ps2_port, PScommand);
	_delay_us(20);                                                   // wait for ACK to pass.
	return(data);
}
void init_PS2inanalougemode()
{
	unsigned char chk_ana = 0, cnt = 0;
	// put controller in config mode

	
	while(chk_ana!= 0x73)
	{
	sbi(ps2_port, PScommand);
	sbi(ps2_port, PSclock);
	cbi(ps2_port, PSattention);

	gameByte(0x01);
	gameByte(0x43);
	gameByte(0x00);
	gameByte(0x01);
	gameByte(0x00);

	sbi(ps2_port, PScommand);
	_delay_ms(1);
	sbi(ps2_port, PSattention);
	_delay_ms(10);

	// put controller in analoge mode
	sbi(ps2_port, PScommand);
	sbi(ps2_port, PSclock);
	cbi(ps2_port, PSattention);

	gameByte(0x01);
	gameByte(0x44);
	gameByte(0x00);
	gameByte(0x01);
	gameByte(0x03);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x00);

	sbi(ps2_port, PScommand);
	_delay_ms(1);
	sbi(ps2_port, PSattention);
	_delay_ms(10);

	// exit config mode
	sbi(ps2_port, PScommand);
	sbi(ps2_port, PSclock);
	cbi(ps2_port, PSattention);

	gameByte(0x01);
	gameByte(0x43);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x5A);
	gameByte(0x5A);
	gameByte(0x5A);
	gameByte(0x5A);
	gameByte(0x5A);

	sbi(ps2_port, PScommand);
	_delay_ms(1);
	sbi(ps2_port, PSattention);
	_delay_ms(10);

	// poll controller and check in analouge mode.
	sbi(ps2_port, PScommand);
	sbi(ps2_port, PSclock);
	cbi(ps2_port, PSattention); 
	gameByte(0x01);
	chk_ana = gameByte(0x42);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x00);
	gameByte(0x00);

	sbi(ps2_port, PScommand);
	_delay_ms(1);
	sbi(ps2_port, PSattention);
	_delay_ms(10);
}
}

int chk_analogue(){
		unsigned char chk_ana = 0;
			sbi(ps2_port, PScommand);
			sbi(ps2_port, PSclock);
			cbi(ps2_port, PSattention);
			gameByte(0x01);
			chk_ana = gameByte(0x42);
			gameByte(0x00);
			gameByte(0x00);
			gameByte(0x00);
			gameByte(0x00);
			gameByte(0x00);
			gameByte(0x00);
			gameByte(0x00);
			sbi(ps2_port, PScommand);
			_delay_ms(1);
			sbi(ps2_port, PSattention);
			_delay_ms(10);
	if(chk_ana == 0x73){
		return 1;
	}
	else 
		{
			return 0;
		}
}

void init_PS2()
{
	sbi(ps2_port_ddr, PBclock);
	cbi(ps2_port_ddr, PBdata);
	sbi(ps2_port, PBdata);

	sbi(ps2_port_ddr, PBcommand);
	sbi(ps2_port_ddr, PBattention);
	init_PS2inanalougemode();
}
void PS2_commn()
{
	unsigned char temp,temp1 ;


	sbi(ps2_port, PScommand);		// start communication with PSx controller
	sbi(ps2_port, PSclock);
	cbi(ps2_port, PSattention);	
	
	gameByte(0x01);			// bite 0. header.
	temp = gameByte(0x42);		// bite 1. header. (should possibly put test on this byte to detect unplugging of controller.)
	gameByte(0x00);			// bite 2. header.

	data0 = gameByte(0x00);	// bite 3. first data bite.
	data1 = gameByte(0x00);	// bite 4.
	data2 = gameByte(0x00);		// bite 5.
	data3 = gameByte(0x00);		// bite 6.
	data4 = gameByte(0x00);		// bite 7.
	data5 = gameByte(0x00);		// bite 8.

	_delay_us(1);
	sbi(ps2_port, PScommand);                      // close communication with PSx controller
	_delay_us(1);
	sbi(ps2_port, PSattention);                        // all done.

}
void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB = ((1<< 2 )|(1<< 1) | (1<<0) );
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}
void SPI_MasterTransmit(char cData)
{
	PORTB &= ~(1<<0);
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	PORTB |= (1<<0);
}
void init_pwm(){
	OCR3A=0;
	OCR3B=0;
	OCR3C=0;
	TCCR3A = 0xFF;
	TCCR3B =0x09;
	TCCR3C = 0x00;
		OCR3A = 00;
	OCR3B = 00;
	OCR3C = 00;
}
void loco(uint8_t bot_pwm1,uint8_t bot_pwm2,uint8_t bot_dirr){
	SPI_MasterTransmit(bot_pwm1);
	_delay_us(500);
	SPI_MasterTransmit(bot_pwm2);
	_delay_us(500);
	SPI_MasterTransmit(bot_dirr);
	_delay_us(500);
}
int pressed(uint8_t dat,uint8_t button){
	if(((dat>>button) & 0x01) == 0){
		return 1;
	}
	else
	return 0;
}
void delay(uint16_t time){
	int i,c;
	c = time/10;
	for(i=0;i<c;i++){
					PS2_commn();
				if(pressed(data1,R1)){
					if(piston1 != 1){
						piston1 = 1;
						if(piston_pushed1 == 0){
							//					motors |= (1<<2);
							PORTG |= (1<<4);
							piston_pushed1 = 1;
							//				OCR3B = 650;
						}
						else if(piston_pushed1 == 1){
							//				motors &= ~(1<<2);
							PORTG &= ~(1<<4);
							//			OCR3B = 650;
							piston_pushed1 = 0;
						}
					}
				}
				else if(pressed(data1,R2)){
					if(piston2 != 1){
						piston2 = 1;
						if(piston_pushed2== 0){
							PORTG |= (1<<0);
							//			motors |= (1<<0);
							//		OCR3A=650;
							piston_pushed2 = 1;
						}
						else if(piston_pushed2==1){
							PORTG &= ~(1<<0);
							//	motors &= ~(1<<0);
							//		OCR3A = 650;
							piston_pushed2 = 0;
						}
					}
				}
				else{
					//	motors &= ~(1<<3);
					//				motors &= ~(1<<1);
					//		motors |= (0x03<<2);
					piston2=0;
					piston1=0;
					//OCR3B = 650;
				}
				if(pressed(data1,squar)){
					if(PINH & 0x01){
						OCR3C = 0;
						motors |= (0xFF);
					}else
					{
						motors &= ~(0x10);
						motors |= 0x20;
						OCR3C = 1000;
					}

				}
				else if(pressed(data1,circle)){
					if(PINH & 0x02){
						OCR3C = 00;
						motors |= (0xFF);
					}
					else
					{
						motors &= ~(0x20);
						motors |= 0x10;
						OCR3C = 1000;
					}
					
				}
				else{
					//		for(int i=0;i<10;i++){
					//			OCR3C -= temp;
					//	_delay_us(2);
					//			}
					OCR3C = 0;
					motors |= (0xF0);
				}
		_delay_ms(10);
	}
}

int main(void)
{
	ps2_port_ddr = 0x0f;
	DDRB |= 0x70;
	PORTL = 0xFF;

	DDRA = 0xFF;
	DDRK = 0xFF;
	DDRE = 0xFF;
		
		init_PS2();
		SPI_MasterInit();
		init_pwm();
	
	int flag_rotate = 0;

/*DDrb for spi , ddrC for ps2 , ddrk for direction ,  for pwm-ocr3a,b,c*/
	DDRJ = 0xFF;
	DDRG = 0xFF;
	DDRF =0xFF;
	DDRH = 0x00;
	int flag=0,flag_1=0,temp=50,cnt,speed_flag=0,flag_tracing=0;
	PORTF = 0;
		
		_delay_ms(500);
		loco(5,1,4);//kp,ki,kd  default values in atmega32-0.5 , 0.2 , 0.5
		_delay_ms(10); 
//PORTE = 0xFF;

while(1)
	{
		/*
		data1 for l1,r1,l2,r2 and shapes
		data2 right joy lr
		data3 right joy ud
		data4 left
		data5 left
		*/
//		if()
/*
if(!chk_analogue()){
	if(flag != 0 ){
			loco(slow13|slow23,slow13 | slow23,0x00);
			flag=0;
	}
	init_PS2inanalougemode();
}*/
PORTF = PINH;
			PS2_commn();
		int cnt=0;	
		if(pressed(data1,R1)){
			if(piston1 != 1){
				piston1 = 1;
				if(piston_pushed1 == 0){
					//					motors |= (1<<2);
					PORTA |= (1<<4);
					piston_pushed1 = 1;
					//				OCR3B = 650;
				}
				else if(piston_pushed1 == 1){
					//				motors &= ~(1<<2);
					PORTA &= ~(1<<4);
					//			OCR3B = 650;
					piston_pushed1 = 0;
				}
			}
		}
		else if(pressed(data1,R2)){
			if(piston2 != 1){
				piston2 = 1;
				if(piston_pushed2== 0){
					PORTA |= (1<<0);
					//			motors |= (1<<0);
					//		OCR3A=650;
					piston_pushed2 = 1;
				}
				else if(piston_pushed2==1){
					PORTA &= ~(1<<0);
					//	motors &= ~(1<<0);
					//		OCR3A = 650;
					piston_pushed2 = 0;
				}
			}
		}
		else{
			//	motors &= ~(1<<3);
			//				motors &= ~(1<<1);
			//		motors |= (0x03<<2);
			piston2=0;
			piston1=0;
			//OCR3B = 650;
		}
		if(pressed(data1,squar)){
			if(!(PINH & 0x01)){
				OCR3C = 0;
				motors |= (0xFF);
			}else
			{
				motors &= ~(0x10);
				motors |= 0x20;
				OCR3C = 1000;
				}
		}
		else if(pressed(data1,circle)){
		if(!(PINH & 0x02)){
				OCR3C = 00;
				motors |= (0xFF);
			}
			else
			{
				motors &= ~(0x20);
				motors |= 0x10;
				OCR3C = 1000;
			}
			
		}
		else{

			//		for(int i=0;i<10;i++){
			//			OCR3C -= temp;
			//	_delay_us(2);
			//			}
			OCR3C = 0;
			motors |= (0xF0);
		}

		if(data2==r_joy_left){
		  	//rotate right;
			cnt=0;
			if(flag!=1){
				loco(slow12|slow22,slow12 | slow22,bot_l_rotate);
					flag=1;

				}
		}
		else if(data2== r_joy_right){
			//rotate right;
			cnt=0;
			if(flag!=2){
				loco(slow12|slow22,slow12 | slow22,bot_r_rotate);
				flag=2;
				PORTJ = 0x20;
			}
		}
		else if(pressed(data0,lef)){
				cnt=0;
				if(pressed(data1,L2)){
					if((flag_1 != 1) || (flag !=7)){
					PORTJ = 0x30;
						flag_1=1;
						flag = 7;
						loco(slow13|slow23,slow13 | slow23,bot_left);
						if(pressed(data1,L2)){
							delay(100);
							loco(med21|med11,med21|med11,bot_left);
							if(pressed(data1,L2)){
								delay(60);
								loco(med23|med13,med23|med13,bot_left);
								if(pressed(data1,L2)){
									_delay_ms(60);
									loco(fast21|fast11,fast21|fast11,bot_left);
								}
							}
						}
					}
				}
				else if(pressed(data1,L1)){
						if((flag_1 !=5) || (flag !=7)){
							
							PORTJ=0x55;
		//                        loco(slow23|slow13,slow23|slow13,0b10000100);
 //                       __delay_ms_ms(100);
					        loco(slow22|slow14,slow22|slow14,0b00100001);		
							if(pressed(data1,L1)){
								_delay_ms(120);
								loco(slow23|med12,slow23|med12,bot_left);
								if(pressed(data1,L1)){
									_delay_ms(100);
									loco(med22|fast11,med22|fast11,bot_left);
									if(pressed(data1,L1)){
										_delay_ms(100);
									loco(med24|fast13,med24|fast13,bot_left);
								}
							}
						}
						flag_1 = 5;
						flag = 7;
						}
					}
					else if((flag_1 != 3) || (flag !=7)){
						if((flag_1 == 5 )&& (flag == 7)){
							flag_tracing = 1;
					        loco(v_slow21|slow11,v_slow21|slow11,0b00100001);		
							_delay_ms(50);
							loco(slow21|slow13,slow21|slow13,0b00000000);
							_delay_ms(130);
							loco(v_slow21|slow11,v_slow21|slow11,bot_left);
						} 
						else{
							loco(slow12|slow22,slow12 | slow22,bot_left);	
						}
						speed_flag=0;
						PORTJ=0x50;
						flag_1=3;
						flag=7;
					}
		}
			else if(pressed(data0,righ)){
							if(pressed(data1,L2)){
								if((flag_1 != 1) || (flag !=8)){
									loco(slow13|slow23,slow13 | slow23,bot_right);
									if(pressed(data1,L2)){
											_delay_ms(100);
											loco(med21|med11,med21|med11,bot_right);
											if(pressed(data1,L2)){
												_delay_ms(60);
												loco(med23|med13,med23|med13,bot_right);
												if(pressed(data1,L2)){
													_delay_ms(60);
													loco(fast21|fast11,fast21|fast11,bot_right);
												}
										}
									}
								flag = 8;
								flag_1=1;
							}
  							}
		
							else{
					if(pressed(data1,L1)){
						if((flag_1 !=5) || (flag !=8)){
							 if((flag_1 == 5 )&& (flag == 9) ){
							 //rotate 90 from backword to right
							 	flag_1 = 5;
							 	flag = 8;
							 	PORTJ = 0x66;
							loco(slow21|fast11,med23|slow11,0b00101000);
							delay(700); 			
							loco(slow21|med11,fast21|slow14,0b00101010);  
							delay(150);
							loco(med21|slow11,med23|slow13,0b01001010);
							delay(100);
							loco(med23|med11,med23|med11,0b01001000);
							_delay_ms(100);
							loco(med23|med11,med23|med11,bot_right);
							_delay_ms(100);
							loco(fast21|med13,fast21|med13,bot_right);
							 PORTJ = 0xAA;
							 }
							 else{
				 					PORTJ=0x44;
									flag_1 = 5;
									flag = 8;
									loco(slow24|slow12,slow24|slow12,0b01001000);
			 							if(pressed(data1,L1)){
											_delay_ms(120);
											loco(med22|slow14,med22|slow14,bot_right);
											if(pressed(data1,L1)){
												delay(100);
												loco(fast21|med13,fast21|med13,bot_right);
											}
											if(pressed(data1,L1)){
												speed_flag=1;
												_delay_ms(60);
												loco(fast23|med14,fast23|med14,bot_right);
											}
									 }
							 } 
						}
					}
					else if((flag_1 != 3) || (flag !=8)){

						if((flag_1 ==5) || (flag ==8)){
								PORTJ=0x22;
								flag_tracing = 1;
								loco(slow21|v_slow11,slow21|v_slow11,0b01001000);						
								_delay_ms(50);
								loco(slow21|v_slow11,slow21|v_slow11,0b00000000);
								_delay_ms(130);
								loco(slow22|v_slow11,slow22|v_slow11,bot_right);
								}
								else{
																				PORTJ=0x55;
									loco(slow12|slow22,slow12 | slow22,bot_right);
								}
									flag_1=3;
									flag=8;
									speed_flag=0;
								}
							}
						}
			else if(pressed(data0,down)){
					cnt=0;
					PORTJ= 0xff;
							if(pressed(data1,L2)){
							PORTJ=0x11;
								if((flag_1 != 1) || (flag !=9)){
								flag_1=1;
								flag = 9;
								loco(slow13|slow23,slow13 | slow23,bot_backword);
									if(pressed(data1,L2)){
										_delay_ms(100);
										loco(med21|med11,med21|med11,bot_backword);
										if(pressed(data1,L2)){
											_delay_ms(60);
											loco(med23|med13,med23|med13,bot_backword);
											if(pressed(data1,L2)){
												_delay_ms(60);											
												loco(fast21|fast11,fast21|fast11,bot_backword);
										}
									}
								}
								}
							}
				else{
					if(pressed(data1,L1)){				
						PORTJ=0x30;
						if((flag_1 !=5) || (flag !=9)){
						flag_1 = 5;
						flag = 9;
							loco(slow23|slow14,slow23|slow14,0b00100001);
							if(pressed(data1,L1)){
								_delay_ms(120);
								loco(slow23|med11,slow23|med11,bot_backword);
								if(pressed(data1,L1)){
									_delay_ms(100);
									loco(med11|med13,med11|med13,bot_backword);
									if(pressed(data1,L1)){
										_delay_ms(100);
										loco(med23|fast11,med23|fast11,bot_backword);
										if(pressed(data1,L1)){
											speed_flag=1;
											_delay_ms(60);
											loco(med23|fast13,med23|fast13,bot_backword);
										}
								}
							}
						}
						}
					}
					else if((flag_1 != 3) || (flag !=9)){
							if((flag_1 ==5) || (flag ==9)){
								flag_tracing =1;
								loco(v_slow21|slow11,v_slow21|slow11,0b00100001);
								_delay_ms(50);
								loco(v_slow21|slow11,v_slow21|slow11,0b00000000);
								_delay_ms(130);
								loco(v_slow21|slow13,v_slow21|slow13,bot_backword);
							} 
							else {
								loco(slow12|slow22,slow12 | slow22,bot_backword);
								_delay_ms(100);
//								loco(slow13|slow23,slow13 | slow23,bot_backword); 
							}	
							flag_1=3;
							flag = 9;							
							speed_flag=0;
												
						}
				}
				}
		else if(pressed(data0,up)){
					if(pressed(data1,L2) ){
						PORTJ=0x20;
						if((flag_1 != 1) || (flag !=10)){
							flag_1=1;
							flag = 10;
							loco(slow13|slow23,slow13 | slow23,bot_forword);
							if(pressed(data1,L2)){
								_delay_ms(100);
								loco(med21|med11,med21|med11,bot_forword);
								if(pressed(data1,L2)){
									_delay_ms(60);
									loco(med23|med13,med23|med13,bot_forword);
									if(pressed(data1,L2)){
										speed_flag=1;
										_delay_ms(60);
										loco(fast21|fast11,fast21|fast11,bot_forword);
									}
								}
							}
						}
					}

				else{
					if(pressed(data1,L1)){	
						PORTJ=0x60;
						if((flag_1 !=5) || (flag !=10)){
							if((flag_1 == 5)&& (flag == 7)){
					loco(slow21|slow11,med23|fast11,0b00000101);
					delay(770);//650
					loco(slow23|slow11,fast21|med13,0b10000101);
					delay(100);//100  
					loco(fast21|slow11,fast21|slow13,0b10000100);	
					delay(100);
					 loco(fast21|med11,fast21|med11,bot_forword);
					delay(100); 
					loco(fast23|med13,fast23|med13,bot_forword);
					delay(100);
					loco(fast23|fast11,fast23|fast11,bot_forword);
					} 
 						else{
							 loco(slow24|slow13,slow24|slow13,0b10000100);
							 if(pressed(data1,L1)){
								 _delay_ms(100);
								 loco(med21|slow13,med21|slow13,0b10000100);
								 if(pressed(data1,L1)){ 
									_delay_ms(100);
									loco(med23|med11,med23|med11,bot_forword);
									 if(pressed(data1,L1)){
										 _delay_ms(100);
										 loco(fast21|med13,fast21|med13,bot_forword);
										if(pressed(data1,L1)){
											_delay_ms(100);
											loco(fast23|med13,fast23|med13,bot_forword);
										}
									 }
								 }
								 
							 }
						 }
						 flag_1=5;
 						flag=10;                
					}
					}
					else if((flag_1 != 3) || (flag !=10)){
								PORTJ=0x80;
														if((flag_1 == 5 )&&(flag == 10) ){
							loco(slow24|v_slow12,slow24|v_slow12,0b10000100);
							_delay_ms(60);								
							loco(slow23|slow13,slow23|slow13,0b00000000);
							_delay_ms(150);
							loco(slow23|v_slow11,slow23|v_slow11,bot_forword);
							flag_tracing = 1;
							}else 
							{
								loco(slow12|slow22,slow12 | slow22,bot_forword);
								}
								flag_1=3;
								flag = 10;
								speed_flag=0;				
							}
					} 
			}
				else if(data4==l_joy_right){
					if((flag_1 != 5 )|| (flag != 8)){
						flag_tracing = 1;
						loco(slow23|v_slow12,slow23|v_slow12,bot_right);
						flag_1 = 5;
						flag = 8;
					}
				}
				else if(data4==l_joy_left){
					if((flag_1 != 5 )|| (flag != 7)){
						flag_tracing = 1;
						loco(v_slow22|slow13,v_slow22|slow13,bot_left);
						flag_1 = 5;
						flag = 7;
					}
				}
				else if(data5==l_joy_up){
					if((flag_1 != 5 )|| (flag != 10)){ 
						flag_tracing = 1;
						loco(slow23|v_slow12,slow23|v_slow12,bot_forword);
						flag_1 = 5;
						flag = 10;
					}
				}
				else if(data5==l_joy_down){
					if((flag_1 != 5 ) || (flag != 9)){
						flag_tracing = 1;
						loco(v_slow22|slow13,v_slow22|slow13,bot_backword);
						flag_1 = 5;
						flag = 9;
					}
				}
		else {
			if(flag!=0)
				{
					speed_flag=0;
					
					if(pressed(data1,L1)){										
						flag_rotate = 1; 
						PORTJ = 0x0F;
					}else{	
						if((((flag_1 == 5 )||(flag_tracing==1))&&(flag == 10)) ){
					flag_tracing=0;
					loco(slow24|v_slow12,slow24|v_slow12,0b10000100);
					_delay_ms(60);
												loco(med22|med12,med22|med12,bot_stop);
												_delay_ms(30);

						//forword
				}	/*
						if((((flag_1 == 5 ) || (flag_tracing==1))&& (flag == 7))){
							loco(med21|slow12,med21|slow12,bot_stop);//left
							  flag_tracing=0;						  			
						}
						if((((flag_1 ==5)||(flag_tracing==1)) && (flag ==8))){
							//right
							flag_tracing=0;
							loco(slow22|med11,slow22|med11,bot_stop);						
						}
						else if((((flag_1 ==5)||(flag_tracing==1)) && (flag ==9)) ){
							flag_tracing=0;
							loco(med21|slow12,med21|slow12,bot_stop);
						}
						else if((((flag_1 == 5 )||(flag_tracing==1))&&(flag == 10)) ){
								flag_tracing=0;
								loco(slow22|med11,slow22|med11,bot_stop);
							//forword
						}else*/{
								PORTJ=0xFF;
								loco(slow14|slow24,slow14 | slow24,0x00);
						}
						cnt++;
						flag_rotate = 0;
						flag=0;
						flag_1=0;
					}
				}
			}
		}
		}
