//
// Created by wirano on 23-4-18.
//

#include "foc_utils.h"
#include <math.h>
#include "foc.h"
#include "math_table.h"


float angle_normalize(float angle) {
    float normalized = fmodf(angle, _2PI);
    return normalized >= 0 ? normalized : (normalized + _2PI);
}

float lpf(foc_lpf_t *handle, float x) {
    uint32_t timestamp = handle->micros();
    float dt = (timestamp - handle->last_time) * 1e-6f;

    if (dt < 0.0f) dt = 1e-3f;
    else if (dt > 0.3f) {
        handle->prev = x;
        handle->last_time = timestamp;
        return x;
    }

    float alpha = handle->Tf / (handle->Tf + dt);
    float y = alpha * handle->prev + (1.0f - alpha) * x;
    handle->prev = y;
    handle->last_time = timestamp;

    return y;
}