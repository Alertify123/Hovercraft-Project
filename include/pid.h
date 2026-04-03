#ifndef HOVERCRAFT_PID_H
#define HOVERCRAFT_PID_H

void pid_reset(void);
void servo_from_yaw_error(float err_deg);

#endif
