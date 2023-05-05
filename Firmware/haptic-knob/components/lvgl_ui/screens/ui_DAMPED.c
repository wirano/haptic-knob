// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.4
// Project name: haptic_knob

#include "../ui.h"

void ui_DAMPED_screen_init(void)
{
    ui_DAMPED = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_DAMPED, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Arc4 = lv_arc_create(ui_DAMPED);
    lv_obj_set_width(ui_Arc4, 380);
    lv_obj_set_height(ui_Arc4, 380);
    lv_obj_set_align(ui_Arc4, LV_ALIGN_CENTER);
    lv_arc_set_range(ui_Arc4, -51, 51);
    lv_arc_set_value(ui_Arc4, 1);
    lv_arc_set_bg_angles(ui_Arc4, 219, 321);
    lv_arc_set_mode(ui_Arc4, LV_ARC_MODE_SYMMETRICAL);

    ui_Button3 = lv_btn_create(ui_DAMPED);
    lv_obj_set_width(ui_Button3, 100);
    lv_obj_set_height(ui_Button3, 50);
    lv_obj_set_x(ui_Button3, 0);
    lv_obj_set_y(ui_Button3, lv_pct(33));
    lv_obj_set_align(ui_Button3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label8 = lv_label_create(ui_Button3);
    lv_obj_set_width(ui_Label8, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label8, 0);
    lv_obj_set_y(ui_Label8, lv_pct(3));
    lv_obj_set_align(ui_Label8, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label8, "BACK");
    lv_obj_set_style_text_font(ui_Label8, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel2 = lv_obj_create(ui_DAMPED);
    lv_obj_set_width(ui_Panel2, 100);
    lv_obj_set_height(ui_Panel2, 50);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label10 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_Label10, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label10, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label10, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(ui_Label10, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Arc4, ui_event_Arc4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button3, ui_event_Button3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_DAMPED, ui_event_DAMPED, LV_EVENT_ALL, NULL);

}
