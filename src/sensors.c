#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "config.h"
#include "sensors.h"

void adc_init(void)
{
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

static uint16_t adc_read(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

float ir_front_distance_cm(void)
{
    uint32_t sum = 0;
    adc_read(IR_FRONT_CH);

    for (uint8_t i = 0; i < IR_SAMPLES; i++) {
        sum += adc_read(IR_FRONT_CH);
        _delay_us(500);
    }

    float avg_adc = (float)sum / (float)IR_SAMPLES;
    float volts = avg_adc * (5.0f / 1023.0f);

    if (volts < 0.1f) return IR_MAX_CM;

    float d = 27.86f / (volts - 0.05f);
    if (d < 0) d = IR_MAX_CM;
    if (d > IR_MAX_CM) d = IR_MAX_CM;

    return d;
}

static void trig_pulse(void)
{
    DDRB |= (1 << US_TRIG);
    PORTB &= ~(1 << US_TRIG);
    _delay_us(2);
    PORTB |= (1 << US_TRIG);
    _delay_us(10);
    PORTB &= ~(1 << US_TRIG);
}

static long measure_echo_duration(void)
{
    DDRD &= ~(1 << US_ECHO);

    const long timeout = 30000;
    long t = 0;

    while ((PIND & (1 << US_ECHO)) && t < timeout) {
        _delay_us(1);
        t++;
    }

    t = 0;
    while (!(PIND & (1 << US_ECHO)) && t < timeout) {
        _delay_us(1);
        t++;
    }
    if (t >= timeout) return -1;

    t = 0;
    while ((PIND & (1 << US_ECHO)) && t < timeout) {
        _delay_us(1);
        t++;
    }
    if (t >= timeout) return -1;

    return t;
}

static float upbar_distance_cm(void)
{
    float minD = 9999.0f;

    for (uint8_t i = 0; i < 3; i++) {
        trig_pulse();
        long us = measure_echo_duration();
        if (us < 0) continue;

        float d = (us * 0.0343f) / 2.0f;
        if (d < minD) minD = d;

        _delay_ms(5);
    }

    if (minD == 9999.0f) return -1.0f;
    return minD;
}

bool upbar_detected(void)
{
    static uint8_t hitCount = 0;

    float d = upbar_distance_cm();
    if (d > 0 && d <= UPBAR_THRESHOLD_CM) {
        hitCount++;
        if (hitCount >= UPBAR_CONFIRM_COUNT) {
            hitCount = 0;
            return true;
        }
    } else {
        hitCount = 0;
    }

    return false;
}
