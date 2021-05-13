/* Name: main.c
 * Project: EasyLogger
 * Author: Christian Starkjohann
 * Creation Date: 2006-04-23
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: Proprietary, free under certain conditions. See Documentation.
 * This Revision: $Id$
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/sleep.h>

#include "usbdrv.h"

/*
Pin assignment:
PB1 = key input (active low with external pull-up)
PB3 = MOSFET 
PB4 = LED output (active high)
PB0, PB2 = USB data lines
*/

/*
time which can be set  for wdt
WDTO_15MS
WDTO_30MS
WDTO_60MS
WDTO_120MS
WDTO_250MS
WDTO_500MS
WDTO_1S
WDTO_2S
WDTO_4S
WDTO_8S
*/

#define BIT_LED 4
#define BIT_KEY 1
#define BIT_MOS 3

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

#ifndef NULL
#define NULL    ((void *)0)
#endif

/* ------------------------------------------------------------------------- */
/* The following variables store the status of the current data transfer */

static volatile uchar currentAddress = 0;
static volatile uchar bytesRemaining = 0;

static volatile uchar mosf_enable = 0; //1 - begin generate signal 0 - no signal need
static volatile uchar i = 0;	   //counter for cycles
static volatile uchar bcount = 0; //number of interrupts from (button & D-) together 
static volatile uchar gosleep = 1; //1 - no usb connect, no usb data, no button; 0 - dont sleep usb on
static volatile uchar tryusb = 0; //1 - detected usb interrupt reconnect usb 0 - can go to sleep
static volatile uchar initusb = 0; //1- if need disconnect,connect,init  0 - if no init need
static volatile uchar wdtvect = 0; //1- if was wdt interrupt 0 - if no wdt interrupt
static volatile int usbpoll = 0; //counter for usb poll
static volatile unsigned int adc = 0; //if timer await
static volatile uchar reset_mcu = 0; //when need reset
static volatile uchar led_s = 0; //previous led state
static volatile uchar mosf_count = 0; //previous led state

static volatile uchar calibrationValue = 0;
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */
/* ------------------------------------------------------------------------- */

uchar   usbFunctionWrite(uchar *data, uchar len)
{
	if(bytesRemaining == 0)
        return 1;               /* end of transfer */
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_write_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

/* ------------------------------------------------------------------------- */

uchar   usbFunctionRead(uchar *data, uchar len)
{
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_read_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
	return 0;
}

/* ------------------------------------------------------------------------- */


ISR(TIMER1_OVF_vect){		//4us betwen interrupt	
	if(mosf_enable){
		PORTB ^= (1<<BIT_MOS); //Invert MOSF output 
	}

}

ISR(WDT_vect){
	wdtvect++; 
	WDTCR |= (1 << WDIE); // if set wdt int	

	if(mosf_enable){
		mosf_count++;
	}
	if(2 == mosf_count){
		mosf_enable = 0;
		PORTB &= ~(1 << BIT_LED);   // LED off 
		DDRB &= ~(1 << BIT_LED);   // input for LED
	}

}	

//SOF is sent every 1ms  on a USB full speed
//SOF is sent every 125us on a USB Hi-speed
ISR (PCINT0_vect){
	bcount++;
	if(bcount > 5){
		mosf_enable = 1;
		bcount = 0;		
		gosleep=0;
		DDRB |= 1 << BIT_LED;   // output for LED 		
		PORTB |= 1 << BIT_LED;      // LED on
	}

}

/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 1;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 1;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}

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
    /* Disable interrupts during oscillator calibration since
     * usbMeasureFrameLength() counts CPU cycles.
     */
    cli();
    calibrateOscillator();
    sei();
    eeprom_write_byte((uint8_t*)10, OSCCAL);   /* store the calibrated value in EEPROM */
}


/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int __attribute__((noreturn)) main(void)
{
m_res:	
	MCUSR = 0;		//clean reset source flag
	wdt_disable();
	MCUCR = 0x12;	//SLEEP STANDBY and The falling edge of INT0 generates an interrupt request.
	cli();
//-------------------VARIABLES SETUP----------------------
	mosf_enable = 0;
	i = 0;	   
	bcount = 0; 
	gosleep = 1; 
	tryusb = 0;  
	initusb = 0; 
	wdtvect = 0; 
	usbpoll = 0; 
	adc = 0; 
	reset_mcu = 0;
	led_s = 0;	
	mosf_count = 0;
//-------------------MODULES SETUP----------------------
	USISR = 0x00; //disable USI
	USICR = 0x00; //disable USI
	
	ACSR = 0x80; //disable comparator
//	ADMUX = 0x00;
//	ADCSRA = 0x00;
	ADMUX = UTIL_BIN8(0000, 0010);  /* Vref=Vcc, measure ADC2(PB4) */
	ADCSRA = UTIL_BIN8(0000, 0111); /* enable ADC, not free running, interrupt disable, rate = 1/128 */
	ADCSRB = 0x00; //
	
//	PLLCSR = 0x06; //PCK enable (timer1 from PLL) PLL enable
	PRR = 0x02;  //usi power reduction
//-------------------PORTS SETUP----------------------
	DDRB = 0x00;  //setup all pin as input
	PORTB = 0x00; // Tri-state (Hi-Z)

	//DIDR0 – Digital Input Disable Register 0

//	DDRB |= 1 << BIT_LED;   // output for LED 
//	DDRB &= ~(1 << BIT_LED);   // input for LED
//	PORTB |= 1 << BIT_LED;      // LED on 
//	PORTB &= ~(1 << BIT_LED);   // LED off 

	DDRB |= 1 << BIT_MOS;   // output for MOSFET 
//	PORTB |= 1 << BIT_MOS;      // MOSFET on 
	PORTB &= ~(1 << BIT_MOS);   // MOSF off 
//-------------------INTERRUPTS & TIMERS SETUP----------------------
	PCMSK |= (1 << PCINT1); //pin1 (button) change enable mask	
	TIMSK |= (1 << TOIE1);	//timer1 interrupt enable
	GIMSK |= (1 << PCIE);	//Pin Change Interrupt Enable

	GTCCR = 0x00;	
	TCCR1 = 0x01;           //4us betwen interrupt CK

//	TCCR1 = 0x0C;           //8ms betwen interrupt CK/2048

	sei();
//-------------------USB SETUP----------------------

	wdt_enable(WDTO_8S);	//start wdt 8sec
	WDTCR |= (1 << WDIE); // if set wdt int	
//-------------BEGIN MAIN LOOP----------------------
    for(;;){  // main event loop 
		//----------------------SLEEP AFTER EACH CYCLE    

		if(mosf_count >= 2){
			mosf_enable = 0;
			goto m_res;
		}	


		if(gosleep){ //if power reset		
			MCUCR = 0x32;		// SLEEP_MODE_PWR_DOWN and The falling edge of INT0 generates an interrupt request.
			sei();		// must be enabled for wakeup
			sleep_cpu();
			MCUCR = 0x12;	//SLEEP STANDBY and The falling edge of INT0 generates an interrupt request.
			MCUSR = 0;	
			
		} 

		
        //----------------------WATCHDOG WAKEUP 
		if(wdtvect > 0){ //if 8sec * num time out	
			wdtvect = 0;
			
			DDRB |= 1 << BIT_LED;   // output for LED 		
			PORTB |= 1 << BIT_LED;      // LED on 
			_delay_ms(1);
			PORTB &= ~(1 << BIT_LED);   // LED off 
			DDRB &= ~(1 << BIT_LED);   // input for LED
			

			PORTB &= ~(1 << BIT_LED);   // LED off 
			DDRB &= ~(1 << BIT_LED);   // input for LED
			ADCSRA |= (1 << ADEN); // enable ADC  
			_delay_us(15);
			ADCSRA |= (1 << ADSC); // start ADC conversion 
			while(ADCSRA & (1 << ADSC)){
				_delay_us(15);
				adc=0;
				adc=ADC;
				if(adc > 400){	//if 5V detected tryusb
					led_s = 1;
					if(0 == usbpoll){
					usbpoll = 1;
					}
				}else{			//if 0V detected disconnect
					led_s = 0;
					if(2 == usbpoll){
					goto m_res;
					}
				}
			}
			ADCSRA &= ~(1 << ADEN);

			if(led_s){
				DDRB |= 1 << BIT_LED;   // output for LED 
				PORTB |= 1 << BIT_LED;      // LED on 
			}else{
				PORTB &= ~(1 << BIT_LED);   // LED off 
				DDRB &= ~(1 << BIT_LED);   // input for LED
			}

			if(1 == usbpoll){
				gosleep = 0;
				tryusb = 1;
				initusb = 1;
				usbpoll = 2;
			}

		}	

		//---------------------- WORK WITH USB       
		if(tryusb){ //if probability usb connect
            if(initusb){
				calibrationValue = eeprom_read_byte((uint8_t*)10); // calibration value from last time 
				if(calibrationValue != 0xff){
					OSCCAL = calibrationValue;
				}
                usbDeviceDisconnect();
                for(i=0;i<20;i++){  // 300 ms disconnect 
                    _delay_ms(15);
                }
                usbDeviceConnect();
				cli();
				usbInit();
				sei();
				initusb = 0;	
            }
        	usbPoll();
            if(0xAC == eeprom_read_byte(0)){
		        eeprom_write_byte((uint8_t*)0,0x81);
		        mosf_enable = 1;
				gosleep =0;
				DDRB |= 1 << BIT_LED;   // output for LED 		
				PORTB |= 1 << BIT_LED;      // LED on
		    }  		
			
		}
        //----------------------	




		
	}

//    return 0;
}

