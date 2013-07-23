/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-

lagtestino - Lag Test for arduIno
=================================

This file contains the source code for the firmware that runs on the
Arduino device. There are two notable things that happen here.

Analog to digital conversion
----------------------------

To keep hardware complexity down, we do best-effort software temporal
anti-aliasing. Sampling is done at the maximum chip rate and then
these values are averaged together to a lower rate for transmission to
the host computer.

Clock synchronization support
-----------------------------

See the http://lagtest.org/synchronization for information about the
theory of operation. In this firmware, we need to respond to clock
requests with our current clock value. We use the device's timer1 as
our official clock.

 */

#include "lagtest_datatypes.h"

// Pin numbers -----------------------------------------------------------------
#define analogPin 0
#define LEDPin 13

// Global variables for ADC ----------------------------------------------------
const uint8_t log2_n_samples = 4;
const uint8_t max_n_samples = 0x01 << log2_n_samples; //Fuse 2**4=16 sample values
volatile uint8_t n_samples=0;
volatile uint16_t accum=0;
volatile timed_sample_t adc_sample;
volatile uint8_t new_adc_sample=0;

// Global variable for clock measurement ---------------------------------------
volatile epoch_dtype epoch=0;

#define CLAMP255( val ) (val) > 255 ? 255 : (val);

// Interrupt service routine for new analog sample ready -----------------------
ISR(ADC_vect)
{

    accum += ADCH;
    n_samples++;

    if (n_samples >= max_n_samples) {
        adc_sample.value = CLAMP255(accum);

        // stamp data with current timestamp
        adc_sample.epoch = epoch;
        adc_sample.ticks = TCNT1;

        accum = 0;
        n_samples = 0;

        new_adc_sample=1;

    }

}

// Interrupt service routine for timer1 overflow -------------------------------
ISR(TIMER1_OVF_vect)
{
    epoch++;

}

// Setup timer1 ----------------------------------------------------------------
void setup_timer1() {
    cli();
    TCCR1A = 0;
    TCCR1B = 0; // normal mode

    TCCR1B |= _BV( CS11 ) | _BV( CS10 ); // clock prescaler 64

    TIMSK1 = _BV(TOIE1); // enable interrupt on timer1
    sei();
}

// Setup analog sampling -------------------------------------------------------
void setup_adc() {
    cli();

    ADMUX = 0;                // Use ADC0.
    ADMUX |= (1 << REFS0);    // Use AVcc as the reference.
    ADMUX |= (1 << ADLAR);    // Set right adjust -> reading ADCH after
                              // convertion will read the higher eight
                              // bits only ( i.e dividing the result by 4 ).

    ADCSRA = 0;               //If this is not set, setting Sample rate fails?!

    ADCSRA |= (1 << ADATE);   // Set free running mode
    ADCSRA |= (1 << ADEN);    // Enable the ADC
    ADCSRA |= (1 << ADIE);    // Enable Interrupts

    // Remeber ADC needs about 13 cycles , in the ISR averaging over 16 samples
    //Adc Sample rate = 16Mhz/64 , 1200 samples/sec *** MAX VALUE POSSIBLE ***
    //Because we have a Bautrate of 115200 = 11.5 kBytes per second , 9 bytes per sample = 1300 samples top!
      ADCSRA |= ( 1 << ADPS2 ) | ( 1 << ADPS1 ) ;

//    ADCSRA |= ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) ;  //Adc Sample rate = 16Mhz/128 = 125khz , 600 Samples/Sec

    ADCSRA |= (1 << ADSC);    // Start the ADC conversion
    sei();
}

// Standard arduino setup function ---------------------------------------------
void setup() {

    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, 0);

    pinMode(2, OUTPUT);
    pinMode(7, OUTPUT);

    // start serial port at 115200 bps:
    Serial.begin(115200);

    setup_timer1();
    setup_adc();
}

// Send data with our simple protocol to the host computer ---------------------
static inline void send_data(const timed_sample_t samp, const char header) {
    const uint8_t * buf;
    Serial.write(header);
    buf = (const uint8_t*)&(samp);
    uint8_t chksum=0;
    for (uint8_t i=0; i< sizeof(timed_sample_t); i++) {
        chksum += buf[i];
        Serial.write(buf[i]);
    }
    Serial.write(chksum);
}

// Standard Arduino loop function. This gets repeatedly called at high rate ----
void loop() {
    static timed_sample_t adc_copy;

    if (new_adc_sample) {

        uint8_t SaveSREG = SREG;   // save interrupt flag
        cli(); // disable interrupts

            adc_copy.value = adc_sample.value;
            adc_copy.epoch = adc_sample.epoch;
            adc_copy.ticks = adc_sample.ticks;
            new_adc_sample = 0;

        SREG = SaveSREG; // restore interrupt flags

        send_data(adc_copy,'H');
    }

    if (Serial.available() >= 2) {
        static char cmd;
        static char value;

        cmd = Serial.read();
        value = Serial.read();

        if (cmd=='P') {

            static timed_sample_t timestamp_request;

            timestamp_request.value = value;

            uint8_t SaveSREG_ = SREG;   // save interrupt flag
            cli(); // disable interrupts

                timestamp_request.epoch = epoch;
                timestamp_request.ticks = TCNT1;

            SREG = SaveSREG_; // restore interrupt flags

            send_data(timestamp_request,'P');
        } else if (cmd=='V') {

            static timed_sample_t version_request;

            version_request.value = 5;

            uint8_t SaveSREG_ = SREG;   // save interrupt flag
            cli(); // disable interrupts

                version_request.epoch = epoch;
                version_request.ticks = TCNT1;

            SREG = SaveSREG_; // restore interrupt flags
            send_data(version_request,'V');
        } else if (cmd=='L') {
            // set LED/DOUT (used for debugging)
            if (value) {
                digitalWrite(LEDPin,HIGH); // turn LED on
            } else {
                digitalWrite(LEDPin,LOW); // turn LED off
            }

            static timed_sample_t LED_request;

            LED_request.value = value;

            uint8_t SaveSREG_ = SREG;   // save interrupt flag
            cli(); // disable interrupts

                LED_request.epoch = epoch;
                LED_request.ticks = TCNT1;

            SREG = SaveSREG_; // restore interrupt flags
            send_data(LED_request,'L');

        }

    }
}
