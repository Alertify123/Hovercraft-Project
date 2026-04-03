#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "imu.h"
#include "motion.h"
#include "sensors.h"
#include "pid.h"
#include "system.h"

volatile uint8_t thrustDutyCycle = 0;
volatile uint8_t liftDutyCycle = 0;

void servo_init(void)
{
    DDRB |= (1 << SERVO_PIN);
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 39999;
    OCR1A = SERVO_CENTER_US * 2;
}

void servo_write_us(int us)
{
    if (us < 600) us = 600;
    if (us > 2400) us = 2400;
    OCR1A = us * 2;
}

void set_servo_angle(int16_t angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    int us = 600 + (int)((1800L * angle) / 180);
    servo_write_us(us);
}

void setup_fans(void)
{
    DDRD |= (1 << THRUST_FAN_PIN) | (1 << LIFT_FAN_PIN);
}

void run_fans(uint8_t thrust, uint8_t lift)
{
    thrustDutyCycle = (thrust > 100) ? 100 : thrust;
    liftDutyCycle = (lift > 100) ? 100 : lift;
}

void stop_fans(void)
{
    thrustDutyCycle = 0;
    liftDutyCycle = 0;
    PORTD &= ~((1 << THRUST_FAN_PIN) | (1 << LIFT_FAN_PIN));
}

void finish_stop(void)
{
    run_fans(0, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);

    DDRB |= (1 << LED_DEBUG);
    while (1) {
        PORTB ^= (1 << LED_DEBUG);
        _delay_ms(300);
    }
}

void turn_by_yaw(float delta_deg)
{
    if (fabsf(delta_deg) < 5.0f) return;

    imu_reset_yaw();

    if (delta_deg > 0) set_servo_angle(ANG_LEFT_DEG);
    else               set_servo_angle(ANG_RIGHT_DEG);

    run_fans(THRUST_TURN_DUTY, LIFT_DUTY);

    uint32_t start = millis();
    while (1) {
        imu_update();

        if (delta_deg > 0 && yaw_deg >= delta_deg - 5.0f) break;
        if (delta_deg < 0 && yaw_deg <= delta_deg + 5.0f) break;

        if (millis() - start > TURN_TIMEOUT_MS) break;
    }

    stop_fans();
    set_servo_angle(ANG_MID_DEG);
    _delay_ms(500);
}

static void advance_then_turn(bool goLeft, float advance_cm)
{
    if (advance_cm < 5.0f) {
        turn_by_yaw(goLeft ? TURN_ANGLE_DEG : -TURN_ANGLE_DEG);
        return;
    }

    imu_update();
    pid_reset();
    float yaw_ref = yaw_deg;

    float currentD = ir_front_distance_cm();
    float targetReading = currentD - advance_cm;

    if (targetReading < MIN_FRONT_CM) targetReading = MIN_FRONT_CM;

    run_fans(THRUST_CRUISE_DUTY, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);

    uint32_t start = millis();
    uint32_t lastCheck = 0;

    while (1) {
        imu_update();
        servo_from_yaw_error(yaw_deg - yaw_ref);

        uint32_t now = millis();
        if (now - lastCheck > SONAR_PERIOD_MS) {
            lastCheck = now;

            if (upbar_detected()) finish_stop();

            float d = ir_front_distance_cm();
            if (d > 0 && d <= targetReading) break;
        }

        if (now - start > 3000) break;
    }

    stop_fans();
    _delay_ms(200);

    turn_by_yaw(goLeft ? TURN_ANGLE_DEG : -TURN_ANGLE_DEG);
}

void handle_intersection(void)
{
    run_fans(THRUST_SCAN_DUTY, LIFT_DUTY);

    set_servo_angle(LEFT_SCAN_ANGLE_DEG);
    _delay_ms(SCAN_SETTLE_MS);
    float dLeft = ir_front_distance_cm();

    set_servo_angle(ANG_MID_DEG);
    _delay_ms(300);

    set_servo_angle(RIGHT_SCAN_ANGLE_DEG);
    _delay_ms(SCAN_SETTLE_MS);
    float dRight = ir_front_distance_cm();

    set_servo_angle(ANG_MID_DEG);
    _delay_ms(SCAN_SETTLE_MS);
    float dFront = ir_front_distance_cm();

    if (dLeft < DEADEND_MAX_CM && dRight < DEADEND_MAX_CM) {
        turn_by_yaw(180.0f);
        return;
    }

    bool goLeft = !(dRight > dLeft);

    float distToCenter = PATH_WIDTH_CM / 2.0f;
    float advanceNeeded = dFront - distToCenter;
    if (advanceNeeded < 0) advanceNeeded = 0;

    advance_then_turn(goLeft, advanceNeeded);
}

void drive_straight_until_wall(void)
{
    imu_update();
    pid_reset();
    float yaw_ref = yaw_deg;

    run_fans(THRUST_CRUISE_DUTY, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);

    uint32_t lastCheck = 0;

    while (1) {
        imu_update();
        servo_from_yaw_error(yaw_deg - yaw_ref);

        uint32_t now = millis();
        if (now - lastCheck > SONAR_PERIOD_MS) {
            lastCheck = now;

            if (upbar_detected()) finish_stop();

            float d = ir_front_distance_cm();
            if (d > 0 && d <= OBSTACLE_THRESHOLD_CM) {
                stop_fans();
                _delay_ms(200);
                break;
            }
        }
    }
}
