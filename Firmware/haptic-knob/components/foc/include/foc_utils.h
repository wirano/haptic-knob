//
// Created by wirano on 23-4-18.
//

#ifndef FOC_UTILS_H
#define FOC_UTILS_H

#include <stdint.h>


#define _limit(a, low, high) (a) < (low) ? (low) : ((a) > (high) ? (high) : (a))


typedef struct {
    uint32_t (*micros)(void);

    uint32_t last_time;
    float Tf;
    float prev;
} foc_lpf_t;


float angle_normalize(float angle);

float lpf(foc_lpf_t *handle, float x);


#endif //FOC_UTILS_H
