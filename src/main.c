#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <avr/pgmspace.h> // Required by usbdrv.h
#include "usbdrv.h"


// #define SETTINGS_BUTTONS ((PIN? & 0x03) >> ?)
// // Buttons settings selected by jumpers for [rect] [circ][EXC]
// #define BUTTONS_00(v) return v;  // [L] [R][M]
// #define BUTTONS_01(v) return (v & 0x06) >> 1 & (v & 0x01) << 2 // [M] [L][R]
// #define BUTTONS_02(v) return (v & 0x04) & (v & 0x02) >> 1 & (v & 0x01) << 1 // [L] [M][R]


#define BUTTONS ( (PIND & 0xe0) >> 5 )
#define TOGGLED ( (PIND & 0x10) >> 4 )

#define CR4     ( (PINC & 0x01)      )
#define CR2     ( (PINC & 0x02) >> 1 )
#define CR1     ( (PINC & 0x04) >> 2 )
#define CR3     ( (PINC & 0x08) >> 3 )

// Configuration switches: B3 B2 B1 B0 C5 C4 D1 D0
#define CONF0   ( (PIND & 0x01)      )
#define CONF1   ( (PIND & 0x02) >> 1 )
#define CONF2   ( (PINC & 0x10) >> 4 )
#define CONF3   ( (PINC & 0x20) >> 5 )
#define CONF4   ( (PINB & 0x01)      ) // FIXME
#define CONF5   ( (PINB & 0x02) >> 1 ) // FIXME
#define CONF6   ( (PINB & 0x04) >> 2 ) // FIXME
#define CONF7   ( (PINB & 0x08) >> 3 ) // FIXME

// #define CONF7   ( 1 << PINB3 )


// #define SCROLL_STEP_DIVIDER 8
// #define SCROLL_STEP_DIVIDER ( 4 + ( (8 << CONF3) | (4 << CONF2) ) )
#define SCROLL_STEP_DIVIDER ( 2 << ( 1 << (1 << CONF3) | CONF2) )
// 16 12 8 4
// 4 + (8 4)
//
// 00010
// 00100
// 01000
// 10000

// 16 8 4 2


#define abs(a) \
	((a > 0)? a: -a)

#define clamp(a, min, max) \
	((a > max)? max: (a < min)? min: a)

// #define accelerate(x, y) \
// 	_accelerate(x); \
// 	_accelerate(y)

// #define linearity 2
#define LINEARITY ( (1 << CONF3) | CONF2 )
#define INTERSECTION 0.5

#define accelerate(x, y) \
	x = (1/(INTERSECTION+LINEARITY)) * x * (abs(x) + LINEARITY); \
	y = (1/(INTERSECTION+LINEARITY)) * y * (abs(y) + LINEARITY)

// #define _accelerate(v) \
// 	clamp(v * abs(v), -121, 121)
	// if(v > 10) \
	// 	v = 121; \
	// else if(v < -10) \
	// 	v = -121; \
	// else \
	// 	v = v * abs(v) \

#define direction(a, b, pa) \
	  ((a ^ b)? -1: 1) * ((a ^ pa)? 1: -1)


#define move(a, b, pa, pb, report_d, report_wheel, w_operation, \
	counter, enable_button) \
\
	if((a ^ pa) || (b ^ pb)) { \
		if (TOGGLED) { \
			if (scrolling_mode & enable_button \
				&& abs(counter) == SCROLL_STEP_DIVIDER) \
			{ \
				report_wheel += \
					direction(a, b, pa); \
				change = 1; \
				counter = 0; \
			} else { \
				counter += direction(a, b, pa); \
			} \
		} else { \
			report_d w_operation direction(a, b, pa); \
			change = 1; \
		} \
	}


// Report descriptor {{{
PROGMEM const char usbHidReportDescriptor[61] = {
	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	0x09, 0x02, // USAGE (Mouse)
	0xa1, 0x01, // COLLECTION (Application)

	0x09, 0x01, //   USAGE (Pointer)
	0xa1, 0x00, //   COLLECTION (Physical)

	0x05, 0x09, //     USAGE_PAGE (Button)
	0x19, 0x01, //     USAGE_MINIMUM (Button 1)
	0x29, 0x03, //     USAGE_MAXIMUM (Button 3)
	0x15, 0x00, //     LOGICAL_MINIMUM (0)
	0x25, 0x01, //     LOGICAL_MAXIMUM (1)
	0x75, 0x01, //     REPORT_SIZE (1)
	0x95, 0x03, //     REPORT_COUNT (3)
	0x81, 0x02, //     INPUT (Data, Var, Abs)
	0x75, 0x05, //     REPORT_SIZE (5)
	0x95, 0x01, //     REPORT_COUNT (1)
	0x81, 0x03, //     INPUT (Const, Var, Abs)

	0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
	0x09, 0x30, //     USAGE (X)
	0x09, 0x31, //     USAGE (Y)
	0x09, 0x38, //     USAGE (Wheel)
	0x15, 0x81, //     LOGICAL_MINIMUM (-127)
	0x25, 0x7f, //     LOGICAL_MAXIMUM (127)
	0x75, 0x08, //     REPORT_SIZE (8)
	0x95, 0x03, //     REPORT_COUNT (3)
	0x81, 0x06, //     INPUT (Data, Var, Rel)

	0x05, 0x0c, //     USAGE_PAGE (Counsumer Devices)
	0x0a, 0x38, 0x02, // USAGE (AC Pan)
	0x95, 0x01, //     REPORT_COUNT (1)
	0x81, 0x06, //     INPUT (Data, Var, Rel)

	0xc0,       //   END_COLLECTION

	0xc0,       // END_COLLECTION
};


typedef struct {
	uchar buttons;
	char  dx;
	char  dy;
	char  dvwheel;
	char  dhwheel;
} report_t;


static report_t report;
static uchar idleRate;

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;

	// The following requests are never used. But since they are required by
	// the specification, we implement them in this example.

	// class request type
	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
		// wValue: ReportType (highbyte), ReportID (lowbyte)
		if (rq->bRequest == USBRQ_HID_GET_REPORT) {
			// we only have one report type, so don't look at wValue
			usbMsgPtr = (void *)&report;
			return sizeof(report);
		} else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
			usbMsgPtr = &idleRate;
			return 1;
		} else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
			idleRate = rq->wValue.bytes[1];
		}
	} else {
		// no vendor specific requests implemented
	}
	// default for not implemented requests: return no data back to host
	return 0;
}

/// }}}

static volatile uchar key_press; // mouse buttons

// SIGNAL (SIG_OUTPUT_COMPARE1A)
ISR(TIMER1_COMPA_vect)
{
	static uchar key_state;
	static uchar ct0, ct1;

	uchar i = key_state ^ ~BUTTONS;

	ct0 = ~(ct0 & i);
	ct1 =  (ct0 ^ ct1) & i;
	i &= ct0 & ct1;
	key_state ^= i;

	key_press |= key_state & i;
}

#define DEBOUNCE 200L

void timer_init()
{
	TIMSK  = 1 << OCIE1A;
	TCCR1B = (1 << CS10) | (1 << WGM12);
	// TCCR1B = (1 << CS12) | (1 << CS10) | (1 << WGM12);
	// OCR1A  = 15625;
	OCR1A  = F_CPU / DEBOUNCE;
}

// Hardware init {{{
static void hardware_init(void)
{
	wdt_enable(WDTO_1S);

	DDRD  = 0x00;
	PORTD = 0xf3; // with pull-ups

	DDRC  = 0x00; // First 4 pins - sensors; conf sw
	PORTC = 0xff; // with pull-ups

	DDRB  = 0x30; // debug, conf sw
	PORTB = 0xcf;

	usbInit();
	usbDeviceDisconnect();

	uchar i = 0;
	while (--i) {
		wdt_reset();
		_delay_ms(1);
	}

	usbDeviceConnect();

	sei();
}
// }}}



static uchar v1, v2, v3, v4, p1, p2, p3, p4;
static uchar buttons_now, buttons_prev, scrolling_mode;
static uchar change;

static char v_counter = 0;
static char h_counter = 0;

int __attribute__((noreturn)) main(void)
{
	timer_init();
	hardware_init();

	change = 0;
	buttons_prev = 0;
	scrolling_mode = 0x04;

	p1 = CR1;
	p3 = CR3;
	p2 = CR2;
	p4 = CR4;

	for (;;) {
		//PORTB = ~((PIND & 0x03) << 4);
		// PORTB = (BUTTONS & 1) << 4;
#define CS (PIND & 0x03)
		//if (CS)
		//	PORTB |= 0x20;
		//else
		//	PORTB &= ~0x20;

		wdt_reset();
		usbPoll();

		// buttons_now = BUTTONS;
		buttons_now = key_press;

		v1 = CR1;
		v3 = CR3;
		v2 = CR2;
		v4 = CR4;

		if (buttons_prev != buttons_now) {
			key_press = 0; // Breaks click and hold, modification of debouncing algorithm needed
			// I hope no one will press toggle with other buttons
			// pressed
			if(TOGGLED) {
				// change scrolling mode
				// modes: only-horizontal, only-vertical, both
				if(buttons_now & 0x07) {
					scrolling_mode = buttons_now;
					v_counter = h_counter = SCROLL_STEP_DIVIDER;
				}
			} else {
				report.buttons = buttons_now;
				change = 1;
			}
		}

		move(v1, v3, p1, p3, report.dx, report.dhwheel, +=, h_counter, 0x06);
		move(v2, v4, p2, p4, report.dy, report.dvwheel, -= ,v_counter, 0x05);

		PORTB |= change << 4;

		buttons_prev = buttons_now;

		p1 = v1;
		p2 = v2;
		p3 = v3;
		p4 = v4;

		if (change && usbInterruptIsReady()) {
			if (!CONF0) {
				accelerate(report.dx, report.dy);
			}

			usbSetInterrupt((void *)&report, sizeof(report));

			report.dx      = 0;
			report.dy      = 0;
			report.dvwheel = 0;
			report.dhwheel = 0;

			change = 0;
			// PORTB &= ~0x10;
		}
	}
}
