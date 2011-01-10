//#############################################################################
/** Подключение библиотек ****************************************************/
#include <avr/io.h>				/* порты и регистры							*/
#include <inttypes.h>			/* типы данных								*/
#include <util/delay.h>			/* задержки	_delay_us(double __us)			*/
								/*			_delay_ms(double __ms)			*/
#include <avr/interrupt.h>		/* прерывания	ISR()						*/
#include <compat/deprecated.h>	/* функции sbi, cbi							*/
#include <compat/ina90.h>		/* функции	_NOP() _CLI() _SEI() _WDR()		*/
								/*			_SLEEP() _OPC(op)				*/
//#include <avr/sfr_defs.h>		/* функции	_BV(bit)						*/
								/*			bit_is_set(sfr, bit)			*/
								/* 			bit_is_clear(sfr, bit)			*/
								/* loop_until_bit_is_set(sfr, bit)			*/
								/* loop_until_bit_is_clear(sfr, bit)		*/
#include <stdio.h>				/* для функции sprintf();					*/
#include <stdlib.h>				/*											*/
#include <math.h>				/*											*/
#include <string.h>
#include <avr/pgmspace.h>		/* pgm_read_byte();	prog_					*/
#include <avr/power.h>
#include <avr/eeprom.h>

#include <avr/wdt.h>
#include "usbdrv.h"
//#include "oddebug.h"        /* Это также пример использования макроса отладки */
#include "requests.h"       /* номера custom request, используемые нами */

/*****************************************************************************/

/****************** Обьявления препроцессора (МАКРО) *************************/
#define MY_MACRO
#ifdef  MY_MACRO

#ifndef		_BV
	#define _BV(bit)	(1 << (bit))
#endif

#ifndef		sbi
	#define sbi(data, bit)	(data) |= _BV(bit)
#endif

#ifndef		cbi
	#define cbi(data, bit)	(data) &= ~_BV(bit)
#endif

#ifndef		bis
	#define	bis(data, bit)	((data) & _BV(bit))
#endif

#ifndef		bic
	#define	bic(data, bit)	(!((data) & _BV(bit)))
#endif
//#define bit_is_clear(sfr, bit) (!(_SFR_BYTE(sfr) & _BV(bit)))


#ifndef	GLUE
#define	GLUE(a,b)		a##b	/* объеденение для макро функций */
#endif

//=>my

#define	PORT(x)			GLUE( PORT,	x )
#define	PIN(x)			GLUE( PIN,	x )
#define	DDR(x)			GLUE( DDR,	x )

/* изменяет бит в байте на противоположный */
#define	chbi(data,bit)	(data)^=_BV(bit)
#define	ibi(data,bit)	(data)^=_BV(bit)

/* (так покороче) вечный цикл пока бит bit в байте sfr не установиться в 1 */
#define	lubis(sfr, bit)	loop_until_bit_is_set(sfr, bit)
/* (так покороче) вечный цикл пока бит bit в байте sfr не установиться в 0 */
#define	lubic(sfr, bit)	loop_until_bit_is_clear(sfr, bit)

#define INVERT8(x)	(0xff&~(x))
#define INVERT16(x)	(0xffff&~(x))
#define INVERT32(x)	(0xffffffff&~(x))

#define VECHNO	1
#define NIKOGDA	0

#endif
/*****************************************************************************/


/*****************************************************************************/
#ifndef	SOFT_PWM_RGB
#define	SOFT_PWM_RGB

#define	LED_R		PB0
#define	DDR_LED_R	DDRB
#define	PORT_LED_R	PORTB

#define	LED_G		PB4
#define	DDR_LED_G	DDRB
#define	PORT_LED_G	PORTB

#define	LED_B		PB3
#define	DDR_LED_B	DDRB
#define	PORT_LED_B	PORTB


volatile uint8_t counter_pwm=0;
volatile uint8_t pwm_enable=0;

volatile uint8_t pwm_r=255;
volatile uint8_t pwm_g=255;
volatile uint8_t pwm_b=255;

static inline 
void init_soft_pwm_rgb( void )
{
	sbi(DDR_LED_R,LED_R);
	sbi(PORT_LED_R,LED_R);
	sbi(DDR_LED_G,LED_G);
	sbi(PORT_LED_G,LED_G);
	sbi(DDR_LED_B,LED_B);
	sbi(PORT_LED_B,LED_B);
}

static inline 
void led_pwm_poll( void )
{
	counter_pwm++;
	
	if( counter_pwm >= pwm_r )sbi(PORT_LED_R,LED_R);
	else cbi(PORT_LED_R,LED_R);
	
	if( counter_pwm >= pwm_g )sbi(PORT_LED_G,LED_G);
	else cbi(PORT_LED_G,LED_G);

	if( counter_pwm >= pwm_b )sbi(PORT_LED_B,LED_B);
	else cbi(PORT_LED_B,LED_B);
}

#endif	/* SOFT_PWM_RGB */
/*****************************************************************************/

/** Настройка таймера # 0 (8 бит) для AtTiny45 *******************************/
//#define TIMER0
#ifdef  TIMER0

static inline 
void init_timer0( void )
{

	TCCR0A	= (0		// Timer/Counter Control Register A – TCCR0A
//				|_BV(COM0A1)	// Compare Match Output A Mode
//				|_BV(COM0A0)
//				|_BV(COM0B1)	// Compare Match Output B Mode
//				|_BV(COM0B0)
//				|_BV()
//				|_BV()
//				|_BV(WGM01)		// Waveform Generation Mode
//				|_BV(WGM00)
				);
//
	TCCR0B	= (0		// Timer/Counter Control Register B – TCCR0B
//				|_BV(FOC0A)		// Force Output Compare A
//				|_BV(FOC0B)		// Force Output Compare B
//				|_BV()
//				|_BV()
//				|_BV(WGM02)
//				|_BV(CS02)		// Clock Select
//				|_BV(CS01)
				|_BV(CS00)
				);
//
	TCNT0 = 0x00;	//Timer/Counter Register – TCNT0
	OCR0A = 0x00;	//Output Compare Register A – OCR0A
	OCR0B = 0x00;	//Output Compare Register B – OCR0B
//	
	TIMSK = (	TIMSK	// 	Timer/Counter Interrupt Mask Register – TIMSK0
//				|_BV(OCIE0B)
//				|_BV(OCIE0A)
				|_BV(TOIE0)
				);
//
}

ISR(TIM0_OVF_vect)
{
	led_pwm_poll();
}

#endif
/*****************************************************************************/


/** Настройка таймера # 1 (8 бит) для AtTiny45 *******************************/
//#define TIMER1
#ifdef  TIMER1

#define OC1A		PB1
#define PORT_OC1A	PORTB
#define DDR_OC1A	DDRB

#define NOC1A		PB0
#define PORT_NOC1A	PORTB
#define DDR_NOC1A	DDRB

#define OC1B		PB4
#define PORT_OC1B	PORTB
#define DDR_OC1B	DDRB

#define NOC1B		PB3
#define PORT_NOC1B	PORTB
#define DDR_NOC1B	DDRB

// 8 МГц = 0.000000125 сек = 125нс

/*

#define OCR1B   _SFR_IO8(0x2B)

#define GTCCR   _SFR_IO8(0x2C)
#define TSM     7
#define PWM1B   6
#define COM1B1  5
#define COM1B0  4
#define FOC1B   3
#define FOC1A   2
#define PSR1    1
#define PSR0    0

#define OCR1C   _SFR_IO8(0x2D)

#define OCR1A   _SFR_IO8(0x2E)

#define TCNT1   _SFR_IO8(0x2F)

#define TCCR1   _SFR_IO8(0x30)
#define CTC1    7
#define PWM1A   6
#define COM1A1  5
#define COM1A0  4
#define CS13    3
#define CS12    2
#define CS11    1
#define CS10    0

#define TIMSK   _SFR_IO8(0x39)
#define OCIE1A  6
#define OCIE1B  5
#define TOIE1   2

#define PLLCSR  _SFR_IO8(0x27)
#define LSM     7	//Low Speed Mode	по дефолту включен
#define PCKE    2	//PCK Enable
#define PLLE    1	//PLL Enable
#define PLOCK   0	//PLL Lock Detector

#define TIM1_COMPA_vect			_VECTOR(3)
#define TIMER1_COMPA_vect		_VECTOR(3)
#define SIG_OUTPUT_COMPARE1A		_VECTOR(3)

#define TIM1_OVF_vect			_VECTOR(4)
#define TIMER1_OVF_vect			_VECTOR(4)
#define SIG_OVERFLOW1			_VECTOR(4)

#define TIM1_COMPB_vect			_VECTOR(9)
#define TIMER1_COMPB_vect		_VECTOR(9)
#define SIG_OUTPUT_COMPARE1B		_VECTOR(9)

 */

static inline 
void init_timer1( void )
{
//#define	ENABLE_PLL
#ifdef	ENABLE_PLL
	PLLCSR = ( PLLCSR
//			|(1<<LSM)
//			|(1<<PCKE)
			|(1<<PLLE)
//			|(1<<PLOCK)
			);
#endif

//#define	ENABLE_PLL2
#ifdef	ENABLE_PLL2
	sbi( PLLCSR,PLLE );	// Enable PLL.
	_delay_us( 100 );	// Wait 100 µs for PLL to stabilize.
	do{}while(bit_is_clear(PLLCSR,PLOCK)); // Poll the PLOCK bit until it is set.
	sbi( PLLCSR,PCKE );	// Set the PCKE bit in the PLLCSR register which enables the asynchronous mode.
	sbi( PLLCSR,LSM ); 	// Low Speed Mode 32MHz
#endif

	TCCR1 = ( 0
//			|(1<<CTC1)
//			|(1<<PWM1A)
//			|(1<<COM1A1)
//			|(1<<COM1A0)
//			|(1<<CS13)
//			|(1<<CS12)
//			|(1<<CS11)
			|(1<<CS10)
			);
	GTCCR = ( 0
//			|(1<<TSM)
//			|(1<<PWM1B)
//			|(1<<COM1B1)
//			|(1<<COM1B0)
//			|(1<<FOC1B)
//			|(1<<FOC1A)
//			|(1<<PSR1)
//			|(1<<PSR0)
			);
	TIMSK = ( TIMSK
//			|(1<<OCIE1A)
//			|(1<<OCIE1B)
			|(1<<TOIE1)
			);


	TCNT1 = 0x00;
//	OCR1A = 0x70;
//	OCR1B = 0xff;
	OCR1C = 0xff;

//	sbi( DDR_OC1B,	OC1B );
//	sbi( PORT_OC1B,	OC1B );

//	sbi( DDR_OC1A,	OC1A );
//	cbi( PORT_OC1A,	OC1A );
}

/*****************************************************************************/

/** Прерывание по переполнению счетчика таймера #1 ***************************/
ISR(TIM1_OVF_vect)
{
	led_pwm_poll();
}
/*****************************************************************************/

#endif
/*****************************************************************************/


/** Таблица значиний синуса для ЦАП (8 бит) **********************************/
//#define DAC_SIN_8BIT
#ifdef  DAC_SIN_8BIT
//#define M_PI 3.141592653589793238462643

/*	вот так мы ее получили
for(uint16_t i=0;i<=255;++i)
	buf[i]=		255*((sin(((2.*M_PI)/255.)*(double)(i))/2.)+0.5);
*/
//pgm_read_byte(&sin[i]) //чтение из флеш памяти

prog_uint8_t sin_my[256]=
{
	127,130,133,136,140,143,146,149,152,155,158,161,164,167,170,173,
	176,179,182,185,187,190,193,195,198,201,203,206,208,211,213,215,
	217,220,222,224,226,228,230,232,233,235,237,238,240,241,242,244,
	245,246,247,248,249,250,251,252,252,253,253,254,254,254,254,254,
	254,254,254,254,254,253,253,252,252,251,250,250,249,248,247,246,
	244,243,242,240,239,237,236,234,232,231,229,227,225,223,221,219,
	216,214,212,209,207,204,202,199,197,194,191,189,186,183,180,177,
	175,172,169,166,163,160,157,154,150,147,144,141,138,135,132,129,
	125,122,119,116,113,110,107,104,100, 97, 94, 91, 88, 85, 82, 79,
	 77, 74, 71, 68, 65, 63, 60, 57, 55, 52, 50, 47, 45, 42, 40, 38,
	 35, 33, 31, 29, 27, 25, 23, 22, 20, 18, 17, 15, 14, 12, 11, 10,
	  8,  7,  6,  5,  4,  4,  3,  2,  2,  1,  1,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  1,  1,  2,  2,  3,  4,  5,  6,  7,  8,  9,
	 10, 12, 13, 14, 16, 17, 19, 21, 22, 24, 26, 28, 30, 32, 34, 37,
	 39, 41, 43, 46, 48, 51, 53, 56, 59, 61, 64, 67, 69, 72, 75, 78,
	 81, 84, 87, 90, 93, 96, 99,102,105,108,111,114,118,121,124,127
};
#endif
/*****************************************************************************/


/*****************************************************************************/

/* ------------------------------------------------------------------------- */
/* ----------------------------- интерфейс USB ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[22] = {    /* дескриптор репорта USB */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Дескриптор выше - только макет, это заглушает драйверы. Репорт, который его
 *  описывает, состоит из одного байта неопределенных данных. Мы не передаем
 *  наши данные через HID-репорты, вместо этого мы используем custom-запросы.
 */

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (void *)data;
	static uchar dataPWM[1];


    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR)
	{
		switch(rq->bRequest)
		{
			case CUSTOM_RQ_SET_STATUS:
			{
				if(rq->wValue.bytes[0])
				{
					//init_timer1();
					pwm_enable = 1;
				}
				else
				{
					//TCCR1 = 0;
					init_soft_pwm_rgb();
					pwm_enable = 0;
				}
			}
			case CUSTOM_RQ_GET_STATUS:
			{
				static uchar dataBuffer[1];     // буфер должен оставаться валидным привыходе из usbFunctionSetup //
				dataBuffer[0] = pwm_enable;
				usbMsgPtr = dataBuffer;         // говорим драйверу, какие данные вернуть //
				return sizeof(dataBuffer);      // говорим драйверу послать 1 байт //
			}
			case CUSTOM_RQ_SET_R_PWM:
			{
				pwm_r = rq->wValue.bytes[0];
			}
			case CUSTOM_RQ_GET_R_PWM:
			{
				dataPWM[0] = pwm_r;
				usbMsgPtr = dataPWM;
				return sizeof(dataPWM);
			}
			case CUSTOM_RQ_SET_G_PWM:
			{
				pwm_g = rq->wValue.bytes[0];
			}
			case CUSTOM_RQ_GET_G_PWM:
			{
				dataPWM[0] = pwm_g;
				usbMsgPtr = dataPWM;
				return sizeof(dataPWM);
			}
			case CUSTOM_RQ_SET_B_PWM:
			{
				pwm_b = rq->wValue.bytes[0];
			}
			case CUSTOM_RQ_GET_B_PWM:
			{
				dataPWM[0] = pwm_b;
				usbMsgPtr = dataPWM;
				return sizeof(dataPWM);
			}
		}
	
	/*
        if(rq->bRequest == CUSTOM_RQ_SET_STATUS){
            if(rq->wValue.bytes[0] & 1)
			{    // установить LED //
				//cbi(LED_PORT,LED_BIT);
				init_timer1();
				pwm_enable = 1;
            }else{                          // очистить LED //
                //sbi(LED_PORT,LED_BIT);
				pwm_enable = 0;
            }
        }else
		if(rq->bRequest == CUSTOM_RQ_GET_STATUS){
            static uchar dataBuffer[1];     // буфер должен оставаться валидным привыходе из usbFunctionSetup //
            //dataBuffer[0] = ((LED_PIN & _BV(LED_BIT)) == 0);
            dataBuffer[0] = pwm_enable;
            usbMsgPtr = dataBuffer;         // говорим драйверу, какие данные вернуть //
            return sizeof(dataBuffer);      // говорим драйверу послать 1 байт //
        }else
		// PWM SET
		if(rq->bRequest == CUSTOM_RQ_SET_PWM){
			pwm_g = rq->wValue.bytes[0];
			//OCR1B = 0xff - rq->wValue.bytes[0];
			
        }else
		// PWM GET
		if(rq->bRequest == CUSTOM_RQ_GET_PWM){
			static uchar dataPWM[1];
			//dataPWM[0] = 0xff - OCR1B;//OCR0;
			dataPWM[0] = pwm_g;
			usbMsgPtr = dataPWM;
			return sizeof(dataPWM);
        }
	*/
    }else
	{
        /* вызовы запросов USBRQ_HID_GET_REPORT и USBRQ_HID_SET_REPORT не реализованы,
         *  поскольку мы их не вызываем. Операционная система также не будет обращаться к ним,
         *  потому что наш дескриптор не определяет никакого значения.
         */
    }
    return 0;   /* default для нереализованных запросов: не возвращаем назад данные хосту */
}

/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/*****************************************************************************/



/*****************************************************************************/
/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/*****************************************************************************/


/*****************************************************************************/
/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
//	uint8_t i=0,j=85,k=170; 
//	uint16_t t=0;
	init_soft_pwm_rgb();
	
	//===========================================
	uchar   calibrationValue;
    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff)
    {
        OSCCAL = calibrationValue;
    }
	//===========================================
	usbInit();
    wdt_enable(WDTO_1S);
    /* Даже если Вы не используете сторожевой таймер (watchdog), выключите его здесь. На более новых
     *  микроконтроллерах состояние watchdog (вкл\выкл, период) СОХРАНЯЕТСЯ ЧЕРЕЗ СБРОС!
     */
	//===========================================
    usbDeviceDisconnect();
    _delay_ms(300);			/* 300 ms disconnect */
    usbDeviceConnect();
	//===========================================
    //sbi(LED_DDR,LED_BIT);			/* делаем ножку, куда подключен LED, выходом */
    //sbi(LED_PORT,LED_BIT);
	//===========================================
	
	//init_timer0();
	sei();
    for(;;)
	{    /* main event loop */
        wdt_reset();
        usbPoll();
		
		
		if(pwm_enable)
		{
			led_pwm_poll();
			//++t;
			//if(t==500)
			//{
			//	t=0;
			//	pwm_r = pgm_read_byte( &sin_my[++i] );
			//	pwm_g = pgm_read_byte( &sin_my[++j] );
			//	pwm_b = pgm_read_byte( &sin_my[++k] );
			//}
		}
	}
	//===========================================
}
/*****************************************************************************/
