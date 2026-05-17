//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_round_t *ui_g_30HZ_8009LF;
extern ui_interface_round_t *ui_g_30HZ_8009LB;
extern ui_interface_round_t *ui_g_30HZ_NUC;
extern ui_interface_number_t *ui_g_30HZ_FRIC_SPD_L;
extern ui_interface_number_t *ui_g_30HZ_AUTO_AIM;
extern ui_interface_number_t *ui_g_30HZ_SHOOT_NUM;
extern ui_interface_number_t *ui_g_30HZ_FRIC_SPD_R;
extern ui_interface_arc_t *ui_g_30HZ_BODY_FRONT;
extern ui_interface_round_t *ui_g_30HZ_8009RF;
extern ui_interface_line_t *ui_g_30HZ_SUPER_CUP;
extern ui_interface_line_t *ui_g_30HZ_L_LEG;
extern ui_interface_line_t *ui_g_30HZ_R_LEG;
extern ui_interface_line_t *ui_g_30HZ_BODY_PITCH;
extern ui_interface_round_t *ui_g_30HZ_8009RB;
extern ui_interface_number_t *ui_g_30HZ_BUFFER_NUM;
extern ui_interface_round_t *ui_g_30HZ_POWER_METER;
extern ui_interface_round_t *ui_g_30HZ_485;
extern ui_interface_round_t *ui_g_30HZ_UNNAME1;
extern ui_interface_round_t *ui_g_30HZ_UNNAME2;
extern ui_interface_round_t *ui_g_30HZ_UNNAME3;
extern ui_interface_round_t *ui_g_30HZ_3508L;
extern ui_interface_round_t *ui_g_30HZ_3508R;
extern ui_interface_round_t *ui_g_30HZ_PITCH;
extern ui_interface_round_t *ui_g_30HZ_ROLL;
extern ui_interface_round_t *ui_g_30HZ_FRIC_L;
extern ui_interface_round_t *ui_g_30HZ_FRIC_R;

void ui_init_g_30HZ();
void ui_update_g_30HZ();
void ui_remove_g_30HZ();

extern ui_interface_string_t *ui_g_5HZ_NewText;
extern ui_interface_string_t *ui_g_5HZ_NewText2;

void ui_init_g_5HZ();
void ui_update_g_5HZ();
void ui_remove_g_5HZ();

extern ui_interface_line_t *ui_g_INIT_NewLine;
extern ui_interface_line_t *ui_g_INIT_NewLine2;
extern ui_interface_line_t *ui_g_INIT_NewLine3;
extern ui_interface_line_t *ui_g_INIT_NewLine4;
extern ui_interface_line_t *ui_g_INIT_NewLine5;
extern ui_interface_string_t *ui_g_INIT_FRIC_SPD;
extern ui_interface_string_t *ui_g_INIT_AUTO_AIM;
extern ui_interface_string_t *ui_g_INIT_SHOOT_NUM;

void ui_init_g_INIT();
void ui_update_g_INIT();
void ui_remove_g_INIT();


#endif // UI_g_H
