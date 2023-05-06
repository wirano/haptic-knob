// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.4
// Project name: haptic_knob

#ifndef _HAPTIC_KNOB_UI_H
#define _HAPTIC_KNOB_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl/lvgl.h"

#include "ui_helpers.h"
#include "components/ui_comp.h"
#include "components/ui_comp_hook.h"
#include "ui_events.h"
// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1(lv_event_t * e);
extern lv_obj_t * ui_Screen1;
void ui_event_enc(lv_event_t * e);
extern lv_obj_t * ui_enc;
extern lv_obj_t * ui_Label1;
void ui_event_damped(lv_event_t * e);
extern lv_obj_t * ui_damped;
extern lv_obj_t * ui_Label2;
void ui_event_light(lv_event_t * e);
extern lv_obj_t * ui_light;
extern lv_obj_t * ui_Label3;
// SCREEN: ui_ENCODER
void ui_ENCODER_screen_init(void);
extern lv_obj_t * ui_ENCODER;
void ui_event_Arc1(lv_event_t * e);
extern lv_obj_t * ui_Arc1;
void ui_event_Button2(lv_event_t * e);
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label7;
extern lv_obj_t * ui_Panel1;
extern lv_obj_t * ui_Label9;
// SCREEN: ui_DAMPED
void ui_DAMPED_screen_init(void);
extern lv_obj_t * ui_DAMPED;
void ui_event_Arc4(lv_event_t * e);
extern lv_obj_t * ui_Arc4;
void ui_event_Button3(lv_event_t * e);
extern lv_obj_t * ui_Button3;
extern lv_obj_t * ui_Label8;
extern lv_obj_t * ui_Panel2;
extern lv_obj_t * ui_Label10;
// SCREEN: ui_LIGHT
void ui_LIGHT_screen_init(void);
extern lv_obj_t * ui_LIGHT;
void ui_event_Arc2(lv_event_t * e);
extern lv_obj_t * ui_Arc2;
extern lv_obj_t * ui_Label4;
extern lv_obj_t * ui_Label5;
void ui_event_Button1(lv_event_t * e);
extern lv_obj_t * ui_Button1;
void ui_event_Label6(lv_event_t * e);
extern lv_obj_t * ui_Label6;
extern lv_obj_t * ui_Panel3;
extern lv_obj_t * ui_Label11;
void ui_event____initial_actions0(lv_event_t * e);
extern lv_obj_t * ui____initial_actions0;

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
