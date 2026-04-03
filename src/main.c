#include <avr/interrupt.h>

#include "config.h"
#include "system.h"
#include "imu.h"
#include "motion.h"
#include "sensors.h"

volatile uint8_t pwmCounter = 0;
volatile uint32_t g_millis = 0;

ISR(TIMER0_COMPA_vect)
{
    g_millis++;
    pwmCounter++;
    if (pwmCounter >= 100) pwmCounter = 0;

    if (pwmCounter < thrustDutyCycle) PORTD |=  (1 << THRUST_FAN_PIN);
    else                              PORTD &= ~(1 << THRUST_FAN_PIN);

    if (pwmCounter < liftDutyCycle)   PORTD |=  (1 << LIFT_FAN_PIN);
    else                              PORTD &= ~(1 << LIFT_FAN_PIN);
}

void systick_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
}

uint32_t millis(void)
{
    uint8_t s = SREG;
    cli();
    uint32_t m = g_millis;
    SREG = s;
    return m;
}

int main(void)
{
    systick_init();
    twi_init();
    adc_init();
    servo_init();
    setup_fans();

    sei();

    mpu_init();
    calibrate_gyro();

    run_fans(0, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);

    while (1) {
        drive_straight_until_wall();
        handle_intersection();
    }
}
