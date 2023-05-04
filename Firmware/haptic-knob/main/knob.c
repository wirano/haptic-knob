//
// Created by wirano on 23-5-2.
//

#include <malloc.h>
#include <math.h>
#include "knob.h"


#define _PI 3.14159265359f
#define _2PI 6.28318530718f


void knob_loop(knob_handle_t handle) {
    float a, v;

    switch (handle->mode) {
        case MODE_DISABLE:
            break;
        case MODE_INERTIA:
            v = handle->foc->data.velocity;
            if (v > 1 || v < -1) {
                if (fabsf(v - handle->lastVelocity) > 3.f)
                    handle->foc->target.velocity = v;
            } else {
                handle->foc->target.velocity = 0;
            }
            handle->lastVelocity = v;
            break;
        case MODE_ENCODER:
            a = handle->foc->data.angle_mech - handle->zeroPosition;
            if (a - handle->lastAngle > _PI / (float) handle->encoderDivides) {
                handle->foc->target.angle += _2PI / (float) handle->encoderDivides;
                handle->lastAngle = handle->foc->target.angle;
                pid_reset(handle->foc->pid_ctrl.angle_loop);
                handle->encoderPosition++;
            } else if (a - handle->lastAngle < -_PI / (float) handle->encoderDivides) {
                handle->foc->target.angle -= _2PI / (float) handle->encoderDivides;
                handle->lastAngle = handle->foc->target.angle;
                pid_reset(handle->foc->pid_ctrl.angle_loop);
                handle->encoderPosition--;
            }
            break;
        case MODE_SPRING:
            break;
        case MODE_DAMPED:
            if (handle->limitPositionMax != 0 && handle->limitPositionMin != 0) {
                a = handle->foc->data.angle_mech - handle->zeroPosition;
                if (a > handle->limitPositionMax) {
                    handle->foc->status.mode = FOC_MODE_POS;
                    handle->foc->target.angle = handle->limitPositionMax;
                    pid_reset(handle->foc->pid_ctrl.angle_loop);
                } else if (a < handle->limitPositionMin) {
                    handle->foc->status.mode = FOC_MODE_POS;
                    handle->foc->target.angle = handle->limitPositionMin;
                    pid_reset(handle->foc->pid_ctrl.angle_loop);
                } else {
                    handle->foc->status.mode = FOC_MODE_VEL;
                    handle->foc->target.velocity = 0;
                    pid_reset(handle->foc->pid_ctrl.velocity_loop);
                }
            }
            break;
        case MODE_SPIN:
            break;
    }

    foc_ctrl_loop(handle->foc);
}

void knob_set_mode(knob_handle_t handle, knob_mode_t mode) {

    handle->lastAngle = handle->foc->data.angle_mech;
    handle->lastVelocity = handle->foc->data.velocity;

    pid_reset(handle->foc->pid_ctrl.velocity_loop);
    pid_reset(handle->foc->pid_ctrl.angle_loop);

    switch (mode) {
        case MODE_DISABLE:
            foc_enable(handle->foc, 0);
            handle->mode = MODE_DISABLE;
            break;
        case MODE_INERTIA:
            handle->foc->pid_ctrl.current_q->limit = 6.f;
            handle->foc->status.mode = FOC_MODE_VEL;

            handle->foc->pid_ctrl.velocity_loop->P = 0.18f;
            handle->foc->pid_ctrl.velocity_loop->I = 0.02f;
            handle->foc->pid_ctrl.velocity_loop->D = 0;


            handle->foc->target.velocity = 0;

            handle->mode = MODE_INERTIA;

            foc_enable(handle->foc, 1);
            break;
        case MODE_ENCODER:
            handle->foc->pid_ctrl.current_q->limit = 12;
            handle->foc->status.mode = FOC_MODE_POS;

            handle->foc->pid_ctrl.angle_loop->P = 1.7f;
            handle->foc->pid_ctrl.angle_loop->I = 0.0f;
            handle->foc->pid_ctrl.angle_loop->D = 0.2f;

            handle->foc->target.angle = 4.2f;

            handle->lastAngle = 4.2f;

            handle->mode = MODE_ENCODER;

            foc_enable(handle->foc, 1);
            break;
        case MODE_SPRING:
            handle->foc->pid_ctrl.current_q->limit = 12;
            handle->foc->status.mode = FOC_MODE_POS;

            handle->foc->pid_ctrl.angle_loop->P = 0.35f;
            handle->foc->pid_ctrl.angle_loop->I = 0.1f;
            handle->foc->pid_ctrl.angle_loop->D = 0.09f    ;

            handle->foc->target.angle = 4.2f;

            handle->mode = MODE_SPRING;

            foc_enable(handle->foc, 1);
            break;
        case MODE_DAMPED:
            handle->foc->pid_ctrl.current_q->limit = 12;
            handle->foc->status.mode = FOC_MODE_VEL;

            handle->foc->pid_ctrl.angle_loop->P = 0.35f;
            handle->foc->pid_ctrl.angle_loop->I = 0.0f;
            handle->foc->pid_ctrl.angle_loop->D = 0.0f;

            handle->foc->pid_ctrl.velocity_loop->P = 0.018f;
            handle->foc->pid_ctrl.velocity_loop->I = 0.002f;
            handle->foc->pid_ctrl.velocity_loop->D = 0;

            handle->foc->target.velocity = 0;

            handle->mode = MODE_DAMPED;

            foc_enable(handle->foc, 1);
            break;
        case MODE_SPIN:
            handle->foc->pid_ctrl.current_q->limit = 12;
            handle->foc->status.mode = FOC_MODE_VEL;

            handle->foc->pid_ctrl.velocity_loop->P = 0.18f;
            handle->foc->pid_ctrl.velocity_loop->I = 0.02f;
            handle->foc->pid_ctrl.velocity_loop->D = 0;

            handle->foc->target.velocity = 15;

            handle->mode = MODE_SPIN;

            foc_enable(handle->foc, 1);
            break;
    }
}

void knob_init(knob_handle_t *handle, foc_handle_t foc) {
    knob_handle_t dev = malloc(sizeof(knob_instance_t));

    dev->mode = MODE_DISABLE;

    dev->limitPositionMax = 5.1f;
    dev->limitPositionMin = 3.3f;
    dev->encoderDivides = 12;

    dev->encoderPosition = 0;
    dev->zeroPosition = 0;

    dev->foc = foc;

    *handle = dev;
}

int knob_encoder_read(knob_handle_t handle) {
    return handle->encoderPosition;
}
