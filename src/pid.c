#include <stdint.h>

#include "config.h"
#include "motion.h"
#include "pid.h"
#include "system.h"

static float yaw_int = 0.0f;
static float yaw_prev_err = 0.0f;
static float yaw_d_filt = 0.0f;
static uint32_t yaw_pid_last_ms = 0;

void pid_reset(void)
{
    yaw_int = 0.0f;
    yaw_prev_err = 0.0f;
    yaw_d_filt = 0.0f;
    yaw_pid_last_ms = 0;
}

void servo_from_yaw_error(float err_deg)
{
    uint32_t now = millis();
    float dt = 0.0f;

    if (yaw_pid_last_ms == 0) {
        yaw_pid_last_ms = now;
    } else {
        uint32_t diff = now - yaw_pid_last_ms;
        yaw_pid_last_ms = now;
        dt = diff * 0.001f;
    }

    if (dt < 0.0f || dt > 0.5f) dt = 0.0f;

    float P = YAW_KP * err_deg;

    if (dt > 0.0f && YAW_KI > 0.0f) {
        yaw_int += err_deg * dt;
        if (yaw_int > YAW_I_MAX) yaw_int = YAW_I_MAX;
        if (yaw_int < -YAW_I_MAX) yaw_int = -YAW_I_MAX;
    }

    float I = YAW_KI * yaw_int;

    float D = 0.0f;
    if (dt > 0.0f && YAW_KD > 0.0f) {
        float derr = (err_deg - yaw_prev_err) / dt;
        yaw_d_filt = YAW_D_LPF_ALPHA * yaw_d_filt + (1.0f - YAW_D_LPF_ALPHA) * derr;
        D = YAW_KD * yaw_d_filt;
    }

    yaw_prev_err = err_deg;

    float u_deg = P + I + D;
    float e = (u_deg < -60.0f) ? -60.0f : ((u_deg > 60.0f) ? 60.0f : u_deg);
    int offset = (int)(e * (SERVO_MAX_OFFSET_US / 60.0f));

    servo_write_us(SERVO_CENTER_US + offset);
}
