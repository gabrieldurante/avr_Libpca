#include "timer_common.h"
#include <avr/interrupt.h>
#include <avr/power.h>

void _timer_en_compa_int(e_timer a_timer) {
	// which pin
	switch(a_timer) {
		case E_TIMER0:
			TIMSK0 |= _BV(OCIE0A);
			break;

		case E_TIMER1:
			TIMSK1 |= _BV(OCIE1A);
			break;

		case E_TIMER2:
			TIMSK2 |= _BV(OCIE2A);
			break;

		default:
			break;
	} // switch
}


void _timer_dis_compa_int(e_timer a_timer) {
	// which pin
	switch(a_timer) {
		case E_TIMER0:
			TIMSK0 &= ~_BV(OCIE0A);
			break;

		case E_TIMER1:
			TIMSK1 &= ~_BV(OCIE1A);
			break;

		case E_TIMER2:
			TIMSK2 &= ~_BV(OCIE2A);
			break;

		default:
			break;
	} // switch
}


uint32_t _timer_freq_prescale(uint32_t a_freq, uint16_t a_criterion) {
	uint8_t prescalers[] = { 0x00, 0x03, 0x06, 0x08, 0x0a, 0x00 };

	/**
	 * combine both prescaler and ocr value in one 32bit value 
	 */
	uint32_t retval = 0x00;
	uint16_t *ocr = (uint16_t *)&retval;
	uint8_t *presc = ((uint8_t *)&retval) + 3;

	do {
		*ocr = F_CPU / ((a_freq << 1) * (0x01 << prescalers[*presc]));
		++*presc;		
	} while ((*ocr > a_criterion) && (prescalers[*presc]));

	/* --*ocr; */
	if (*ocr > a_criterion) *ocr = a_criterion;
	return retval;
}


void _timer_init_ctc(e_timer a_timer) {

	switch (a_timer) {
		case E_TIMER0:
			power_timer0_enable();
			// clock disabled
			TCCR0A = 0x02;
			TCCR0B = 0x00;
			TCNT0 = 0x00;
			OCR0A = 0x00;
			break;

		case E_TIMER1:
			power_timer1_enable();
			// clock disabled, CTC mode
			TCCR1A = 0x00;
			TCCR1B = 0x08;
			TCNT1H = 0x00;
			TCNT1L = 0x00;
			OCR1AH = 0x00;
			OCR1AL = 0x00;
			break;

		case E_TIMER2:
			power_timer2_enable();
			TCCR2A = 0x02;
			TCCR2B = 0x00;
			TCNT2 = 0x00;
			OCR2A = 0x00;
			break;

		default:
			break;
	} // switch
}


void _timer_setup_ctc(e_timer a_timer, uint32_t a_pocr) {

	switch(a_timer) {
		case E_TIMER0:
			TCCR0B &= 0xf8;
			TCCR0B |= (a_pocr >> 24) & 0x07;			
			OCR0A = a_pocr & 0xff;
			TCNT0 = 0x00;
			break;

		case E_TIMER1:
			TCCR1B &= 0xf8;
			TCCR1B |= ((a_pocr >> 24) & 0x07);
			OCR1AL = a_pocr & 0xff;
			OCR1AH = (a_pocr >> 8) & 0xff;
			TCNT1H = TCNT1L = 0x00;
			break;

		case E_TIMER2:
			TCCR2B &= 0xf8;
			TCCR2B |= (a_pocr >> 24) & 0x07;			
			OCR2A = a_pocr & 0xff;
			TCNT2 = 0x00;
			break;

		default:
			break;
	} // switch

}
