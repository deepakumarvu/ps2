/* rpmmeasurement.c
 *
 * Created: 02-09-2017 18:12:18
 * Author : Sharmaji
 Perfect code till date 2/6
 modification possible in pid constants, can be increased
 */

/*
		check the pwm value for incremental set different for slow and max speed
*/ 
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
/*
#define forword		11
#define backword	12
#define left		13
#define right		14	
#define l_rotate	15
#define r_rotate	16
#define stop		10
*/
#define motor		4
/*
#define	slow_speed		95.31484375
#define fast_speed		146.4844//175.98125
#define little_fast_speed		146.4844//194.39183
#define very_fast_speed	249.1234375
*/
#define clk			0x02
#define anclk		0x01
#define softbreak 	0x00
#define hardstop	0x03

#define v_slow1 	0x0C
#define v_slow2		0x0D
#define v_slow3		0x0E
#define v_slow4		0x0F

#define slow1 		0x00
#define slow2		0x01
#define slow3		0x02
#define slow4		0x03

#define med1		0x04
#define med2		0x05
#define med3		0x06
#define med4		0x07

#define	fast1		0x08
#define fast2		0x09
#define fast3		0x0A
#define fast4		0x0B




//values that are according to application
volatile float kp=0.5;
volatile float ki=0.2;
volatile float kd=0.5;
#define offset 0

int incremental_pwm_V_max=600;
int incremental_pwm_max=400;              //for max speed of bot
int incremental_pwm_min=200;              //for min speed of bot

#define motord PORTD	
 //0x01
 //0x02
#define address 0x01      // 01 | 02 | 03 | 04
#define forwrd motord=(1<<1)|(0<<0);
#define backwrd motord=(0<<1)|(1<<0);
#define minpwm 100
volatile uint16_t count=0,flag_overflow=0,duty1=0;
volatile uint16_t p=3,counter_time=0,total_time=0xFF,distance_count=0;
                     //may be changed
volatile double givenrpm = 0;
volatile int factor=1;
volatile double rpm1 =0;
int slow_flag=0;
volatile uint8_t spi_data=0,direction=0,start=0,dir_data=0,pwm_data1=0,pwm_data2=0;
volatile int flag_spi=-3,flag_spi_updated=0;
void initpwm()
{
	//FAST PWM && INVERTING MODE &&  PRESCALER  /256
	OCR1A=0;
	TCCR1A |= 0xF3;
	TCCR1B |= 0x0C;
	TIMSK |= (1<<TOIE1);
}

void setPWM(uint16_t a){
	OCR1A=a;
}

void SPI_SlaveInit(void)
{
	/* Set MISO output, all others input */
	DDRB |= (1<<6);
	DDRB &= ~(1<<4 )|(1<<5) | (1<<7);
	/* Enable SPI */
	SPCR |= (1<<SPE)|(1<<SPIE);
}
double getpulses(float revol){
	double val = (revol*500);
	return val;
}

ISR(SPI_STC_vect){
   
	spi_data = SPDR;
	if(flag_spi==0){
		pwm_data2 = spi_data;
		flag_spi=1;
	}
	else if(flag_spi==1){
		pwm_data1 = spi_data;
		flag_spi_updated=1;
		flag_spi=2;
	}
	else if(flag_spi==2){
		flag_spi=0;
		dir_data=spi_data;
	}
	else if(flag_spi == -3){
		kp= ((float)spi_data)/10;
		flag_spi = -2;
	}else if(flag_spi == -2){
		ki= ((float)spi_data)/10;
		flag_spi = -1;
	}else if(flag_spi == -1){
		kd= ((float)spi_data)/10;
		flag_spi = 0;
	}
}
//1.check whether the unset of toie1 flag stops the interrupt overflow from working or not.
//2.Calculate the rpm
void port_init(){
	DDRB=0xff;		//for encoder input
	DDRB &= ~(1<<0);
	DDRA = 0xFF;
	DDRD = 0xFF;
	DDRC =0xFF;
}
ISR(TIMER2_OVF_vect){
flag_overflow++;	//0.032768
}
int main(void)
{
	volatile double error=0;
    int flag_1=0,flag_2=0,i=0,f=0,rev=0,flag_dir=0;
    uint16_t c=minpwm;
	uint16_t pid=0,duty=0;
	double eintegral=0,eprev=0;

	port_init();
	sei();
	//DDRB= 0xff;
	SPI_SlaveInit();
	TCCR0 |= (1<< CS00)|(1<<CS01)|(1<<CS02);
	TCCR2 |= (1<<CS20 | 1<<CS21 | (1<<CS22));
	TIMSK |= TOIE2;

	initpwm();
	flag_1=0;//_delay_ms(1000);
	setPWM(0);
//dir_data = 0;
//dir_data = 0x02;
while(1){
//	PORTA = TCNT0;
//	PORTA = givenrpm;
//	PORTC = dir_data;
//PORTC
	if(((dir_data>>(2*(motor-1)))& 0x03)==clk) //1
		{
			PORTD |= (1<<1);
			PORTD  &= ~(1<<0);
			flag_2=1;
			flag_dir = 1;
		}
	else if(((dir_data>>(2*(motor-1)))& 0x03)==anclk)
		{
			PORTD |= (1<<0);
			PORTD  &= ~(1<<1);
			flag_2=1;
			flag_dir = 2;
		}
	else if(((dir_data>>( 2*(motor-1)))& 0x03)==softbreak){
				if(flag_2 == 1){
				c=(duty-minpwm)/10;
				for(i=0;i<10;i++){
		                duty-=c;
		                setPWM(duty);
		                _delay_us(100);
				}
			}
			setPWM(duty1);
			PORTD &= ~(11<<0);
			flag_overflow=0;
			flag_dir = 0;
            //PORTD &= ~(11<<0);
			count=0;
			p=1;
			error=0;
			eintegral=0;
			eprev=0;
			flag_1=0;
			flag_2=0;
			c=minpwm;
			factor=1;
		}
		else if(((dir_data>>(2*(motor-1)))& 0x03)==hardstop){
			if(flag_2 == 1){
				c=(duty-minpwm)/30;
                for(i=0;i<30;i++){
	                if(duty == minpwm);
	                else{
		                duty-=c;
		                setPWM(duty);
		                _delay_us(100);
	                }
				}
			}
			flag_overflow=0;
			PORTD |= (1<<1)|(1<<0);
			setPWM(duty1);
			flag_dir = 0;
            //PORTD &= ~(11<<0);
			count=0;
			p=1;
			error=0;
			eintegral=0;
			eprev=0;
			flag_1=0;
			flag_2=0;
			c=minpwm;
			factor=1;
		}
		rpm_values();
		if(flag_2==1){
//			PORTA = count;
					if(flag_1==0){
						flag_1=1;
								if(givenrpm < 160){
									for(i=0;i<20;i++){              //check this value before
										setPWM(c);
										c+=(incremental_pwm_min-minpwm)/20;
										_delay_us(100);
									}
									c=incremental_pwm_min;
								}
								else if(givenrpm < 200){
									for(i=0;i<20;i++){              //check this value before
										setPWM(c);
										c+=(incremental_pwm_max-minpwm)/20;
										_delay_us(100);
									}
									c=incremental_pwm_max;
								}
								else if(givenrpm > 200){
									for(i=0;i<30;i++){              //check this value before
										//incremental upto 1023 for max pwm ,delay in ms , check if direction is stop if it is make i =30	
										setPWM(c);
										c+=(incremental_pwm_V_max-minpwm)/30;
										_delay_us(100);
									}
									c=incremental_pwm_V_max;
								}
						setPWM(c);
						_delay_ms(40);
						p=3;
						count=0;
						eintegral=(c/ki);
					}
			        if(p==4){
						
		//			if(flag_overflow == 10)

			//		PORTC = dir_data;
									//PORTC = count;
							        rpm1 = ((3.66211)* count)/factor;
                        			error=(givenrpm - rpm1);
									pid=(uint16_t)((kp*error)+(ki*eintegral)+(kd*(error-eprev))); // eprev - error//pid eq
									eintegral += error;//(((kp*error)+(ki*eintegral)+(kd*(error-eprev)))/ ki); //for past errors
									eprev=error; //for future errors. predicts future error behavior
									duty=pid;
									if(pid>=1023)     //max pwm above which bot is not expected to move
									{
						
										PORTA = 0xFF;
										pid=1023;
										duty = 1023;
									} //check so that pwm doesnt go beyond 255
									if(pid < 0 || pid == 0)
									{
										pid=0;
									}
									if(givenrpm < 10){
										setPWM(0);
									}
									else {
										setPWM(pid);										
									}						
									if(error<4&& error>-4){
										p=2;
										factor=2;
									if(error<2 && error>-2){
										p=1;
										factor=3;
										PORTA = 0x66;
										}
									}
									else
									{
											p=3;
											factor=1;
									}
									count=0;
									//PORTA = total_time;
									}
									}
					}
				}
ISR(TIMER1_OVF_vect)
{
	//count += TCNT0;
	count +=  TCNT0;                //CHANGE IS NEEDED
	TCNT0=0;
	p=(p+1)%5;
//	PORTA = TCNT0;
}
void rpm_values(){
			if(flag_spi_updated == 1){

				flag_spi_updated = 0;
				if((motor==1) || (motor == 2)){
					if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==v_slow1)
					{	givenrpm = 0;
						duty1 = 100;
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==v_slow2)
					{
						duty1 = 100;
						givenrpm =36.9211;
					}
					else if(((pwm_data1>> (4*(motor-1))) & 0x0F)==v_slow3)
					{
						duty1 =100;
						givenrpm = 55.33165;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==v_slow4)
					{
						duty1 = 100;
						givenrpm = 66.31798;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==slow1)
					{	givenrpm = 77.30431;
						duty1 = 100;
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==slow2)
					{
						duty1 = 200;
						givenrpm =91.55275;
						
					}
					else if(((pwm_data1>> (4*(motor-1))) & 0x0F)==slow3)
					{
						duty1 =300;
						givenrpm = 109.8633;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==slow4)
					{
						duty1 = 400;
						givenrpm = 131.83596;
						
					}

					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==med1)
					{
						duty1 = 500;
						givenrpm = 153.80862;
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==med2)
					{
						duty1 = 600;
						givenrpm = 175.78128;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==med3)
					{
						duty1 = 700;
						givenrpm = 197.75394;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==med4)
					{
						duty1 = 800;
						givenrpm = 219.7266;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==fast1)
					{
						duty1 = 900;
						givenrpm = 241.69926;
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==fast2)
					{
						duty1 = 1000;
						givenrpm = 263.67192;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==fast3)
					{
						duty1 = 1000;
						givenrpm = 285.64458;
						
					}
					else if(((pwm_data1 >> (4*(motor-1))) & 0x0F)==fast4)
					{
						duty1 = 1000;
						givenrpm = 307.61724;
					}
				}
				else if((motor ==3) ||(motor == 4)){
					if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==v_slow1)
					{	givenrpm = 0;
						duty1 = 100;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==v_slow2)
					{
						duty1 = 100;
						givenrpm =36.9211;
					}
					else if(((pwm_data2>> (4*(motor-3))) & 0x0F)==v_slow3)
					{
						duty1 =100;
						givenrpm =55.33165;
						
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==v_slow4)
					{
						duty1 = 100;
						givenrpm = 66.31798;
						
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==slow1)
					{	givenrpm = 77.30431;
						duty1 = 100;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==slow2)
					{
						givenrpm = 91.55275;
						duty1 = 200;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==slow3)
					{
						duty1 = 300;
						givenrpm = 109.8633;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==slow4)
					{
						duty1 = 400;
						givenrpm = 131.83596;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==med1)
					{
						duty1 = 500;
						givenrpm = 153.80862;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==med2)
					{
						duty1 = 600;
						givenrpm = 175.78128;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==med3)
					{
						duty1 = 700;
						givenrpm = 197.75394;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==med4)
					{
						duty1 = 800;
						givenrpm = 219.7266;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==fast1)
					{
						duty1 = 900;
						givenrpm = 241.69926;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==fast2)
					{
						duty1 = 1000;
						givenrpm = 263.67192;
					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==fast3)
					{
						duty1 = 1000;
						givenrpm = 285.64458;

					}
					else if(((pwm_data2 >> (4*(motor-3))) & 0x0F)==fast4)
					{
						duty1 = 1000;
						givenrpm = 307.61724;
					}
				}
			}

}


//note:if rpm is read in main, count should be cleared
//in isr note count+=tcnt0,
//rpm is always updated after 30 ms
