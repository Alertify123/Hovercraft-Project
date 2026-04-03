#ifndef HOVERCRAFT_IMU_H
#define HOVERCRAFT_IMU_H

void twi_init(void);
void mpu_init(void);
void calibrate_gyro(void);
void imu_update(void);
void imu_reset_yaw(void);

extern float yaw_deg;

#endif
