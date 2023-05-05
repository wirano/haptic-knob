//
// Created by wirano on 23-5-2.
//

#ifndef KNOB_H
#define KNOB_H

#include "foc.h"


typedef enum {
    MODE_DISABLE,
    MODE_INERTIA,
    MODE_ENCODER,
    MODE_SPRING,
    MODE_DAMPED,
    MODE_SPIN
} knob_mode_t;

typedef struct {
    knob_mode_t mode;
    foc_handle_t foc;
    float zeroPosition;
    float limitPositionMax;
    float limitPositionMin;
    int encoderDivides;
    int encoderPosition;

    float lastAngle;
    float lastVelocity;
} knob_instance_t;

typedef knob_instance_t *knob_handle_t;


void knob_init(knob_handle_t *handle, foc_handle_t foc);

void knob_loop(knob_handle_t handle);

void knob_set_mode(knob_handle_t handle, knob_mode_t mode);

int knob_encoder_read(knob_handle_t handle);

void knob_set_zero(knob_handle_t handle);


#endif //KNOB_H
