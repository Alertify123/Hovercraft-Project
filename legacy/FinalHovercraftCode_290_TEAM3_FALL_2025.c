#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define SERVO_PIN        PB1
#define THRUST_FAN_PIN   PD4
#define LIFT_FAN_PIN     PD7
#define LED_DEBUG        PB5
#define US_TRIG          PB3 
#define US_ECHO          PD2 
#define IR_FRONT_CH      0    

#define PATH_WIDTH_CM          61.0f   
#define OBSTACLE_THRESHOLD_CM  60.0f   
#define DEADEND_MAX_CM         15.0f   
#define MIN_FRONT_CM           10.0f   

#define UPBAR_THRESHOLD_CM     25.0f   
#define UPBAR_CONFIRM_COUNT    3       

#define TURN_ANGLE_DEG         90.0f
#define TURN_TIMEOUT_MS        3000UL

#define THRUST_CRUISE_DUTY     80      
#define THRUST_TURN_DUTY       65      
#define THRUST_SCAN_DUTY       0       
#define LIFT_DUTY              100     

#define ANG_LEFT_DEG           0
#define ANG_MID_DEG            90
#define ANG_RIGHT_DEG          180

#define LEFT_SCAN_ANGLE_DEG    20      
#define RIGHT_SCAN_ANGLE_DEG   160     

#define SCAN_SETTLE_MS         600     
#define SONAR_PERIOD_MS        60      

#define IR_MAX_CM              80.0f
#define IR_SAMPLES             10

#define CF_ALPHA               0.98f
#define YAW_DEADBAND_DPS       0.05f
#define SERVO_CENTER_US        1500
#define SERVO_MAX_OFFSET_US    400

#define YAW_KP                 3.0f    
#define YAW_KI                 0.1f    
#define YAW_KD                 0.8f    
#define YAW_I_MAX              30.0f   
#define YAW_D_LPF_ALPHA        0.7f    

volatile uint8_t thrustDutyCycle = 0;
volatile uint8_t liftDutyCycle   = 0;
volatile uint8_t pwmCounter      = 0;
volatile uint32_t g_millis       = 0;

static float yaw_int       = 0.0f;
static float yaw_prev_err  = 0.0f;
static float yaw_d_filt    = 0.0f;
static uint32_t yaw_pid_last_ms = 0;

static float yaw_deg       = 0.0f;
static float gyroZ_offset  = 0.0f;
static uint32_t imu_last_ms = 0;

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

static void systick_init(void)
{
    TCCR0A = (1 << WGM01);              
    TCCR0B = (1 << CS01) | (1 << CS00); 
    OCR0A  = 249;                       
    TIMSK0 = (1 << OCIE0A);
}

static inline uint32_t millis(void)
{
    uint8_t s = SREG;
    cli();
    uint32_t m = g_millis;
    SREG = s;
    return m;
}

static void adc_init(void)
{
    ADMUX  = (1 << REFS0); 
    ADCSRA = (1 << ADEN) | (1 << ADPS2)|(1 << ADPS1)|(1<<ADPS0); 
}

static uint16_t adc_read(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

static float ir_front_distance_cm(void)
{
    uint32_t sum = 0;
    adc_read(IR_FRONT_CH);
    
    for (uint8_t i=0; i<IR_SAMPLES; i++) {
        sum += adc_read(IR_FRONT_CH);
        _delay_us(500);
    }

    float avg_adc = (float)sum / (float)IR_SAMPLES;
    float volts   = avg_adc * (5.0f / 1023.0f);

    if (volts < 0.1f) return IR_MAX_CM;
    
    float d = (27.86f / (volts - 0.05f));

    if (d < 0) d = IR_MAX_CM;
    if (d > IR_MAX_CM) d = IR_MAX_CM;

    return d;
}

static void trig_pulse(void)
{
    DDRB |= (1 << US_TRIG);
    PORTB &= ~(1 << US_TRIG);
    _delay_us(2);
    PORTB |=  (1 << US_TRIG);
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

    for (uint8_t i=0; i<3; i++) {
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

static bool upbar_detected(void)
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

static void servo_init(void)
{
    DDRB |= (1<<SERVO_PIN);
    TCCR1A = (1<<COM1A1)|(1<<WGM11);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); 
    ICR1   = 39999; 
    OCR1A  = SERVO_CENTER_US * 2;
}

static void servo_write_us(int us)
{
    if (us < 600) us = 600;
    if (us > 2400) us = 2400;
    OCR1A = us * 2;
}

static void set_servo_angle(int16_t angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    int us = 600 + (int)((1800L * angle) / 180);
    servo_write_us(us);
}

static void setupFans(void) { DDRD |= (1<<THRUST_FAN_PIN) | (1<<LIFT_FAN_PIN); }

static void runFans(uint8_t thrust, uint8_t lift) {
    thrustDutyCycle = (thrust > 100) ? 100 : thrust;
    liftDutyCycle   = (lift > 100)  ? 100 : lift;
}

static void finish_stop(void)
{
    runFans(0, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);

    DDRB |= (1 << LED_DEBUG);
    while (1) {
        PORTB ^= (1 << LED_DEBUG);
        _delay_ms(300);
    }
}

#define MPU_ADDR 0x68

static void twi_init(void) {
    TWSR = 0; TWBR = 72; TWCR = (1 << TWEN);
    PORTC |= (1 << PC4) | (1 << PC5);
}
static void twi_start(void) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}
static void twi_stop(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    while (TWCR & (1<<TWSTO));
}
static void twi_write(uint8_t d) {
    TWDR = d; TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}
static uint8_t twi_read_ack(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT))); return TWDR;
}
static uint8_t twi_read_nack(void) {
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT))); return TWDR;
}
static void mpu_write8(uint8_t reg, uint8_t val) {
    twi_start(); twi_write((MPU_ADDR<<1)|0); twi_write(reg); twi_write(val); twi_stop();
}
static void mpu_readN(uint8_t reg, uint8_t *buf, uint8_t n) {
    twi_start(); twi_write((MPU_ADDR<<1)|0); twi_write(reg);
    twi_start(); twi_write((MPU_ADDR<<1)|1);
    for(uint8_t i=0;i<n;i++) buf[i]=(i==n-1)?twi_read_nack():twi_read_ack();
    twi_stop();
}

static void mpu_init(void) {
    _delay_ms(50);
    mpu_write8(0x6B, 0x01); 
    _delay_ms(10);
    mpu_write8(0x1A, 0x03); 
    mpu_write8(0x1B, 0x00); 
}

static void calibrate_gyro(void) {
    int32_t sum = 0;
    for(int i=0; i<500; i++) {
        uint8_t b[2];
        mpu_readN(0x47, b, 2); 
        int16_t gz = (b[0]<<8)|b[1];
        sum += gz;
        _delay_ms(1);
    }
    gyroZ_offset = (float)sum / 500.0f;
}

static void imu_update(void) {
    uint32_t now = millis();
    if (imu_last_ms == 0) { imu_last_ms = now; return; }
    uint32_t diff = now - imu_last_ms;
    if (diff < 4) return;
    
    float dt = diff * 0.001f;
    imu_last_ms = now;

    uint8_t b[2];
    mpu_readN(0x47, b, 2);
    int16_t gz_raw = (b[0]<<8)|b[1];
    float gz_dps = ((float)gz_raw - gyroZ_offset) / 131.0f;

    if (fabsf(gz_dps) < YAW_DEADBAND_DPS) gz_dps = 0.0f;
    yaw_deg += gz_dps * dt;
}

static void servo_from_yaw_error(float err_deg)
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

    if (dt < 0.0f || dt > 0.5f) {
        dt = 0.0f;
    }

    float P = YAW_KP * err_deg;

    if (dt > 0.0f && YAW_KI > 0.0f) {
        yaw_int += err_deg * dt;
        if (yaw_int >  YAW_I_MAX) yaw_int =  YAW_I_MAX;
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

    float e = (u_deg < -60.0f) ? -60.0f : (u_deg > 60.0f) ? 60.0f : u_deg;
    int offset = (int)(e * (SERVO_MAX_OFFSET_US / 60.0f));
    servo_write_us(SERVO_CENTER_US + offset);
}

static void stopFans(void) {
    thrustDutyCycle = 0; 
    liftDutyCycle   = 0;
    PORTD &= ~((1<<THRUST_FAN_PIN)|(1<<LIFT_FAN_PIN));
}

static void turn_by_yaw(float delta_deg)
{
    if (fabsf(delta_deg) < 5.0f) return;

    yaw_deg = 0.0f;
    imu_last_ms = millis();

    if (delta_deg > 0) set_servo_angle(ANG_LEFT_DEG);
    else               set_servo_angle(ANG_RIGHT_DEG);

    runFans(THRUST_TURN_DUTY, LIFT_DUTY);

    uint32_t start = millis();
    while (1) {
        imu_update();
        float y = yaw_deg;

        if (delta_deg > 0 && y >= delta_deg - 5.0f) break;
        if (delta_deg < 0 && y <= delta_deg + 5.0f) break;

        if (millis() - start > TURN_TIMEOUT_MS) {
            break;
        }
    }

    stopFans();
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
    float yaw_ref = yaw_deg;

    float currentD = ir_front_distance_cm();
    float targetReading = currentD - advance_cm;

    if (targetReading < MIN_FRONT_CM) targetReading = MIN_FRONT_CM;

    runFans(THRUST_CRUISE_DUTY, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);

    uint32_t start     = millis();
    uint32_t lastCheck = 0;

    while (1) {
        imu_update();
        
        float err = yaw_deg - yaw_ref;
        servo_from_yaw_error(err);

        uint32_t now = millis();
        if (now - lastCheck > SONAR_PERIOD_MS) {
            lastCheck = now;

            if (upbar_detected()) {
                finish_stop(); 
            }

            float d = ir_front_distance_cm();
            if (d > 0 && d <= targetReading) {
                break;
            }
        }

        if (now - start > 3000) { 
            break; 
        }
    }

    stopFans();
    _delay_ms(200);

    turn_by_yaw(goLeft ? TURN_ANGLE_DEG : -TURN_ANGLE_DEG);
}

static void handle_intersection(void)
{
    runFans(THRUST_SCAN_DUTY, LIFT_DUTY);

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

    bool goLeft = true; 
    
    if (dRight > dLeft) {
        goLeft = false;
    } else {
        goLeft = true;
    }

    float distToCenter   = PATH_WIDTH_CM / 2.0f;
    float advanceNeeded  = dFront - distToCenter;
    if (advanceNeeded < 0) advanceNeeded = 0;

    advance_then_turn(goLeft, advanceNeeded);
}

static void drive_straight_until_wall(void)
{
    imu_update();
    float yaw_ref = yaw_deg;

    runFans(THRUST_CRUISE_DUTY, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);

    uint32_t lastCheck = 0;

    while(1) {
        imu_update();
        
        float err = yaw_deg - yaw_ref;
        servo_from_yaw_error(err);

        uint32_t now = millis();
        if (now - lastCheck > SONAR_PERIOD_MS) {
            lastCheck = now;

            if (upbar_detected()) {
                finish_stop(); 
            }

            float d = ir_front_distance_cm();
            if (d > 0 && d <= OBSTACLE_THRESHOLD_CM) {
                stopFans();
                _delay_ms(200); 
                break;
            }
        }
    }
}

int main(void)
{
    systick_init();
    twi_init();
    adc_init();
    servo_init();
    setupFans();
    
    sei();

    mpu_init();
    calibrate_gyro();

    runFans(0, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);
    _delay_ms(1500);

    while (1) {
        drive_straight_until_wall();
        handle_intersection();
    }
}
