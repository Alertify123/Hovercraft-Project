#include <util/delay.h>
#include <math.h>
#include <stdint.h>

#include "config.h"
#include "imu.h"
#include "system.h"

float yaw_deg = 0.0f;

static float gyroZ_offset = 0.0f;
static uint32_t imu_last_ms = 0;

void twi_init(void)
{
    TWSR = 0;
    TWBR = 72;
    TWCR = (1 << TWEN);
    PORTC |= (1 << PC4) | (1 << PC5);
}

static void twi_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while (TWCR & (1 << TWSTO));
}

static void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

static uint8_t twi_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

static uint8_t twi_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

static void mpu_write8(uint8_t reg, uint8_t val)
{
    twi_start();
    twi_write((MPU_ADDR << 1) | 0);
    twi_write(reg);
    twi_write(val);
    twi_stop();
}

static void mpu_readN(uint8_t reg, uint8_t *buf, uint8_t n)
{
    twi_start();
    twi_write((MPU_ADDR << 1) | 0);
    twi_write(reg);
    twi_start();
    twi_write((MPU_ADDR << 1) | 1);

    for (uint8_t i = 0; i < n; i++) {
        buf[i] = (i == n - 1) ? twi_read_nack() : twi_read_ack();
    }

    twi_stop();
}

void mpu_init(void)
{
    _delay_ms(50);
    mpu_write8(0x6B, 0x01);
    _delay_ms(10);
    mpu_write8(0x1A, 0x03);
    mpu_write8(0x1B, 0x00);
}

void calibrate_gyro(void)
{
    int32_t sum = 0;

    for (int i = 0; i < 500; i++) {
        uint8_t b[2];
        mpu_readN(0x47, b, 2);
        int16_t gz = (b[0] << 8) | b[1];
        sum += gz;
        _delay_ms(1);
    }

    gyroZ_offset = (float)sum / 500.0f;
}

void imu_update(void)
{
    uint32_t now = millis();

    if (imu_last_ms == 0) {
        imu_last_ms = now;
        return;
    }

    uint32_t diff = now - imu_last_ms;
    if (diff < 4) return;

    float dt = diff * 0.001f;
    imu_last_ms = now;

    uint8_t b[2];
    mpu_readN(0x47, b, 2);
    int16_t gz_raw = (b[0] << 8) | b[1];
    float gz_dps = ((float)gz_raw - gyroZ_offset) / 131.0f;

    if (fabsf(gz_dps) < YAW_DEADBAND_DPS) gz_dps = 0.0f;

    yaw_deg += gz_dps * dt;
}

void imu_reset_yaw(void)
{
    yaw_deg = 0.0f;
    imu_last_ms = millis();
}
