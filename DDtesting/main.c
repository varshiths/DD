#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD 38400 // baud rate
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1) // ubrr value

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>

#include "mpu6050/mpu6050.h"
#include "mpu6050/mpu6050.c"
#include "mpu6050/mpu6050dmp6.c"
#include "mpu6050/mpu6050registers.h"

#include "i2chw/i2cmaster.h"
#include "i2chw/twimastertimeout.c"

void uartinit (void)
{
	UBRR0H |= (unsigned char) (BAUDRATE>>8);
	UBRR0L |= (unsigned char) BAUDRATE;
	UCSR0B |= (1<<TXEN0) | (1<<RXEN0); //enable receiver and transmitter
	UCSR0C |= (3<<UCSZ00); // frame set
}

void uarttransmit (int data)
{
	while (!( UCSR0A & (1<<UDRE0))); // wait till register is free
	//data = 99;
	UDR0 = data; // load data in the register
	//while (!( UCSR0A & (1<<UDRE0))); // second wait
	
}

void uarttransmits(char * str){
	while (*str){
		uarttransmit(*str++);
	}
}

void adcinit(){
	
	//ADCSRA |= (1<<ADEN); // enabling adc
	ADCSRA |= (1<<ADPS2); // prescaler to 16
	ADMUX |= (1<<ADLAR); // left adjust
	ADMUX &= ~((1<<REFS0)|(1<<REFS1));// reference voltage // refs0 = 0 and refs1 = 0
	
}

void adcread(int pin){
	
	//while(!ADIF){}
	ADMUX &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2)|(1<<MUX3));
	
	switch(pin){
		case 0:	ADMUX |= (0<<MUX0); break; //setting input pin to ADC0
		case 1: ADMUX |= (1<<MUX0); break;
		case 2: ADMUX |= (0<<MUX0)|(1<<MUX1); break;
		case 3: ADMUX |= (1<<MUX0)|(1<<MUX1); break;
		case 4: ADMUX |= (0<<MUX0)|(0<<MUX1)|(1<<MUX2); break;
		default: break;
	}
	ADCSRA |= (1<<ADEN); // enabling adc everytime read is done
	PRR &= ~(1<<PRADC); // disabling power save mode to start //pradc = 0
	ADCSRA |= (1<<ADSC);// starting conversion
	while(ADCSRA & (1<<ADSC)){};
}

void adctransmit(){
	while(ADCSRA & (1<<ADSC)){};
	int p = ADCH;
	char itmp[10];
	itoa(p, itmp, 10); uarttransmits(itmp);
}


void mpu(int mode) {
	mpumode = mode;
	
	#if MPU6050_GETATTITUDE == 0
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    double axg = 0;
    double ayg = 0;
    double azg = 0;
    double gxds = 0;
    double gyds = 0;
    double gzds = 0;
	#endif

	#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
    double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;
	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;
	#endif

    //init uart
	//uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	
	//init interrupt
	sei();

	//init mpu6050
	mpu6050_init();
	//_delay_ms(50);

	//init mpu6050 dmp processor
	#if MPU6050_GETATTITUDE == 2
	//mpu6050_dmpInitialize();
	//mpu6050_dmpEnable();
	//_delay_ms(10);
	#endif

	#if MPU6050_GETATTITUDE == 0
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
	mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
	#endif

	#if MPU6050_GETATTITUDE == 1
	mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
	mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
	//_delay_ms(10);
	#endif

	#if MPU6050_GETATTITUDE == 2
	if(mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
		mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
	}
	//_delay_ms(10);
	#endif

	#if MPU6050_GETATTITUDE == 0
	char itmp[10];
			
	//uarttransmit(ax);
	ltoa(ax, itmp, 10); uarttransmits(itmp);
	uarttransmit('+');
	ltoa(ay, itmp, 10); uarttransmits(itmp);
	uarttransmit('+');
	ltoa(az, itmp, 10); uarttransmits(itmp);

	//_delay_ms(100);
	#endif

	#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
	
	//roll pitch yaw
	char ptr[20];
	dtostrf(roll,50,6,ptr);
	uarttransmits(ptr);
	uarttransmit('+');
	dtostrf(pitch,50,6,ptr);
	uarttransmits(ptr);
	uarttransmit('+');
	dtostrf(yaw,50,6,ptr);
	uarttransmits(ptr);
	
	#endif
}

int main(void){
	uartinit();
	adcinit();
		
	while(1){
		uarttransmit('#');
		for (int i=0; i<5; i++)
		{
			adcread(0); adctransmit();
			uarttransmit('+');
		}
		
		mpu(0);
		uarttransmit('+');
		mpu(1);
		uarttransmit('~');
	}
}