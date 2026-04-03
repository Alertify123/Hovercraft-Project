#ifndef HOVERCRAFT_MOTION_H
#define HOVERCRAFT_MOTION_H

#include <stdint.h>

extern volatile uint8_t thrustDutyCycle;
extern volatile uint8_t liftDutyCycle;

void servo_init(void);
void setup_fans(void);
void run_fans(uint8_t thrust, uint8_t lift);
void stop_fans(void);

void servo_write_us(int us);
void set_servo_angle(int16_t angle);

void finish_stop(void);
void turn_by_yaw(float delta_deg);
void drive_straight_until_wall(void);
void handle_intersection(void);

#endif
