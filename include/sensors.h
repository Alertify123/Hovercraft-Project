#ifndef HOVERCRAFT_SENSORS_H
#define HOVERCRAFT_SENSORS_H

#include <stdbool.h>

void adc_init(void);
float ir_front_distance_cm(void);
bool upbar_detected(void);

#endif
