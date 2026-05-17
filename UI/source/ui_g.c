//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_30HZ_0;

ui_interface_round_t *ui_g_30HZ_8009LF = (ui_interface_round_t*)&(ui_g_30HZ_0.data[0]);
ui_interface_round_t *ui_g_30HZ_8009LB = (ui_interface_round_t*)&(ui_g_30HZ_0.data[1]);
ui_interface_round_t *ui_g_30HZ_NUC = (ui_interface_round_t*)&(ui_g_30HZ_0.data[2]);
ui_interface_number_t *ui_g_30HZ_FRIC_SPD_L = (ui_interface_number_t*)&(ui_g_30HZ_0.data[3]);
ui_interface_number_t *ui_g_30HZ_AUTO_AIM = (ui_interface_number_t*)&(ui_g_30HZ_0.data[4]);
ui_interface_number_t *ui_g_30HZ_SHOOT_NUM = (ui_interface_number_t*)&(ui_g_30HZ_0.data[5]);
ui_interface_number_t *ui_g_30HZ_FRIC_SPD_R = (ui_interface_number_t*)&(ui_g_30HZ_0.data[6]);

void _ui_init_g_30HZ_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_0.data[i].figure_name[0] = 0;
        ui_g_30HZ_0.data[i].figure_name[1] = 0;
        ui_g_30HZ_0.data[i].figure_name[2] = i + 0;
        ui_g_30HZ_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_30HZ_0.data[i].operate_type = 0;
    }

    ui_g_30HZ_8009LF->figure_type = 2;
    ui_g_30HZ_8009LF->operate_type = 1;
    ui_g_30HZ_8009LF->layer = 0;
    ui_g_30HZ_8009LF->color = 4;
    ui_g_30HZ_8009LF->start_x = 1900;
    ui_g_30HZ_8009LF->start_y = 900;
    ui_g_30HZ_8009LF->width = 10;
    ui_g_30HZ_8009LF->r = 5;

    ui_g_30HZ_8009LB->figure_type = 2;
    ui_g_30HZ_8009LB->operate_type = 1;
    ui_g_30HZ_8009LB->layer = 0;
    ui_g_30HZ_8009LB->color = 4;
    ui_g_30HZ_8009LB->start_x = 1900;
    ui_g_30HZ_8009LB->start_y = 876;
    ui_g_30HZ_8009LB->width = 10;
    ui_g_30HZ_8009LB->r = 5;

    ui_g_30HZ_NUC->figure_type = 2;
    ui_g_30HZ_NUC->operate_type = 1;
    ui_g_30HZ_NUC->layer = 0;
    ui_g_30HZ_NUC->color = 4;
    ui_g_30HZ_NUC->start_x = 1900;
    ui_g_30HZ_NUC->start_y = 660;
    ui_g_30HZ_NUC->width = 10;
    ui_g_30HZ_NUC->r = 5;

    ui_g_30HZ_FRIC_SPD_L->figure_type = 6;
    ui_g_30HZ_FRIC_SPD_L->operate_type = 1;
    ui_g_30HZ_FRIC_SPD_L->layer = 0;
    ui_g_30HZ_FRIC_SPD_L->color = 4;
    ui_g_30HZ_FRIC_SPD_L->start_x = 300;
    ui_g_30HZ_FRIC_SPD_L->start_y = 870;
    ui_g_30HZ_FRIC_SPD_L->width = 2;
    ui_g_30HZ_FRIC_SPD_L->font_size = 20;
    ui_g_30HZ_FRIC_SPD_L->number = 12345;

    ui_g_30HZ_AUTO_AIM->figure_type = 6;
    ui_g_30HZ_AUTO_AIM->operate_type = 1;
    ui_g_30HZ_AUTO_AIM->layer = 0;
    ui_g_30HZ_AUTO_AIM->color = 4;
    ui_g_30HZ_AUTO_AIM->start_x = 300;
    ui_g_30HZ_AUTO_AIM->start_y = 835;
    ui_g_30HZ_AUTO_AIM->width = 2;
    ui_g_30HZ_AUTO_AIM->font_size = 20;
    ui_g_30HZ_AUTO_AIM->number = 12345;

    ui_g_30HZ_SHOOT_NUM->figure_type = 6;
    ui_g_30HZ_SHOOT_NUM->operate_type = 1;
    ui_g_30HZ_SHOOT_NUM->layer = 0;
    ui_g_30HZ_SHOOT_NUM->color = 4;
    ui_g_30HZ_SHOOT_NUM->start_x = 300;
    ui_g_30HZ_SHOOT_NUM->start_y = 800;
    ui_g_30HZ_SHOOT_NUM->width = 2;
    ui_g_30HZ_SHOOT_NUM->font_size = 20;
    ui_g_30HZ_SHOOT_NUM->number = 12345;

    ui_g_30HZ_FRIC_SPD_R->figure_type = 6;
    ui_g_30HZ_FRIC_SPD_R->operate_type = 1;
    ui_g_30HZ_FRIC_SPD_R->layer = 0;
    ui_g_30HZ_FRIC_SPD_R->color = 4;
    ui_g_30HZ_FRIC_SPD_R->start_x = 417;
    ui_g_30HZ_FRIC_SPD_R->start_y = 864;
    ui_g_30HZ_FRIC_SPD_R->width = 2;
    ui_g_30HZ_FRIC_SPD_R->font_size = 20;
    ui_g_30HZ_FRIC_SPD_R->number = 12345;


    ui_proc_7_frame(&ui_g_30HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_0, sizeof(ui_g_30HZ_0));
}

void _ui_update_g_30HZ_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_30HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_0, sizeof(ui_g_30HZ_0));
}

void _ui_remove_g_30HZ_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_30HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_0, sizeof(ui_g_30HZ_0));
}
ui_7_frame_t ui_g_30HZ_1;

ui_interface_arc_t *ui_g_30HZ_BODY_FRONT = (ui_interface_arc_t*)&(ui_g_30HZ_1.data[0]);
ui_interface_round_t *ui_g_30HZ_8009RF = (ui_interface_round_t*)&(ui_g_30HZ_1.data[1]);
ui_interface_line_t *ui_g_30HZ_SUPER_CUP = (ui_interface_line_t*)&(ui_g_30HZ_1.data[2]);
ui_interface_line_t *ui_g_30HZ_L_LEG = (ui_interface_line_t*)&(ui_g_30HZ_1.data[3]);
ui_interface_line_t *ui_g_30HZ_R_LEG = (ui_interface_line_t*)&(ui_g_30HZ_1.data[4]);
ui_interface_line_t *ui_g_30HZ_BODY_PITCH = (ui_interface_line_t*)&(ui_g_30HZ_1.data[5]);
ui_interface_round_t *ui_g_30HZ_8009RB = (ui_interface_round_t*)&(ui_g_30HZ_1.data[6]);

void _ui_init_g_30HZ_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_1.data[i].figure_name[0] = 0;
        ui_g_30HZ_1.data[i].figure_name[1] = 0;
        ui_g_30HZ_1.data[i].figure_name[2] = i + 7;
        ui_g_30HZ_1.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_30HZ_1.data[i].operate_type = 0;
    }

    ui_g_30HZ_BODY_FRONT->figure_type = 4;
    ui_g_30HZ_BODY_FRONT->operate_type = 1;
    ui_g_30HZ_BODY_FRONT->layer = 0;
    ui_g_30HZ_BODY_FRONT->color = 4;
    ui_g_30HZ_BODY_FRONT->start_x = 960;
    ui_g_30HZ_BODY_FRONT->start_y = 540;
    ui_g_30HZ_BODY_FRONT->width = 2;
    ui_g_30HZ_BODY_FRONT->start_angle = 340;
    ui_g_30HZ_BODY_FRONT->end_angle = 20;
    ui_g_30HZ_BODY_FRONT->rx = 200;
    ui_g_30HZ_BODY_FRONT->ry = 200;

    ui_g_30HZ_8009RF->figure_type = 2;
    ui_g_30HZ_8009RF->operate_type = 1;
    ui_g_30HZ_8009RF->layer = 0;
    ui_g_30HZ_8009RF->color = 4;
    ui_g_30HZ_8009RF->start_x = 1900;
    ui_g_30HZ_8009RF->start_y = 852;
    ui_g_30HZ_8009RF->width = 10;
    ui_g_30HZ_8009RF->r = 5;

    ui_g_30HZ_SUPER_CUP->figure_type = 0;
    ui_g_30HZ_SUPER_CUP->operate_type = 1;
    ui_g_30HZ_SUPER_CUP->layer = 0;
    ui_g_30HZ_SUPER_CUP->color = 2;
    ui_g_30HZ_SUPER_CUP->start_x = 623;
    ui_g_30HZ_SUPER_CUP->start_y = 123;
    ui_g_30HZ_SUPER_CUP->width = 15;
    ui_g_30HZ_SUPER_CUP->end_x = 1307;
    ui_g_30HZ_SUPER_CUP->end_y = 121;

    ui_g_30HZ_L_LEG->figure_type = 0;
    ui_g_30HZ_L_LEG->operate_type = 1;
    ui_g_30HZ_L_LEG->layer = 0;
    ui_g_30HZ_L_LEG->color = 2;
    ui_g_30HZ_L_LEG->start_x = 1596;
    ui_g_30HZ_L_LEG->start_y = 710;
    ui_g_30HZ_L_LEG->width = 1;
    ui_g_30HZ_L_LEG->end_x = 1594;
    ui_g_30HZ_L_LEG->end_y = 530;

    ui_g_30HZ_R_LEG->figure_type = 0;
    ui_g_30HZ_R_LEG->operate_type = 1;
    ui_g_30HZ_R_LEG->layer = 0;
    ui_g_30HZ_R_LEG->color = 2;
    ui_g_30HZ_R_LEG->start_x = 1713;
    ui_g_30HZ_R_LEG->start_y = 718;
    ui_g_30HZ_R_LEG->width = 1;
    ui_g_30HZ_R_LEG->end_x = 1715;
    ui_g_30HZ_R_LEG->end_y = 538;

    ui_g_30HZ_BODY_PITCH->figure_type = 0;
    ui_g_30HZ_BODY_PITCH->operate_type = 1;
    ui_g_30HZ_BODY_PITCH->layer = 0;
    ui_g_30HZ_BODY_PITCH->color = 2;
    ui_g_30HZ_BODY_PITCH->start_x = 1791;
    ui_g_30HZ_BODY_PITCH->start_y = 711;
    ui_g_30HZ_BODY_PITCH->width = 1;
    ui_g_30HZ_BODY_PITCH->end_x = 1476;
    ui_g_30HZ_BODY_PITCH->end_y = 718;

    ui_g_30HZ_8009RB->figure_type = 2;
    ui_g_30HZ_8009RB->operate_type = 1;
    ui_g_30HZ_8009RB->layer = 0;
    ui_g_30HZ_8009RB->color = 4;
    ui_g_30HZ_8009RB->start_x = 1900;
    ui_g_30HZ_8009RB->start_y = 828;
    ui_g_30HZ_8009RB->width = 10;
    ui_g_30HZ_8009RB->r = 5;


    ui_proc_7_frame(&ui_g_30HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_1, sizeof(ui_g_30HZ_1));
}

void _ui_update_g_30HZ_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_1.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_30HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_1, sizeof(ui_g_30HZ_1));
}

void _ui_remove_g_30HZ_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_1.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_30HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_1, sizeof(ui_g_30HZ_1));
}
ui_7_frame_t ui_g_30HZ_2;

ui_interface_number_t *ui_g_30HZ_BUFFER_NUM = (ui_interface_number_t*)&(ui_g_30HZ_2.data[0]);
ui_interface_round_t *ui_g_30HZ_POWER_METER = (ui_interface_round_t*)&(ui_g_30HZ_2.data[1]);
ui_interface_round_t *ui_g_30HZ_485 = (ui_interface_round_t*)&(ui_g_30HZ_2.data[2]);
ui_interface_round_t *ui_g_30HZ_UNNAME1 = (ui_interface_round_t*)&(ui_g_30HZ_2.data[3]);
ui_interface_round_t *ui_g_30HZ_UNNAME2 = (ui_interface_round_t*)&(ui_g_30HZ_2.data[4]);
ui_interface_round_t *ui_g_30HZ_UNNAME3 = (ui_interface_round_t*)&(ui_g_30HZ_2.data[5]);
ui_interface_round_t *ui_g_30HZ_3508L = (ui_interface_round_t*)&(ui_g_30HZ_2.data[6]);

void _ui_init_g_30HZ_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_2.data[i].figure_name[0] = 0;
        ui_g_30HZ_2.data[i].figure_name[1] = 0;
        ui_g_30HZ_2.data[i].figure_name[2] = i + 14;
        ui_g_30HZ_2.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_30HZ_2.data[i].operate_type = 0;
    }

    ui_g_30HZ_BUFFER_NUM->figure_type = 6;
    ui_g_30HZ_BUFFER_NUM->operate_type = 1;
    ui_g_30HZ_BUFFER_NUM->layer = 0;
    ui_g_30HZ_BUFFER_NUM->color = 4;
    ui_g_30HZ_BUFFER_NUM->start_x = 628;
    ui_g_30HZ_BUFFER_NUM->start_y = 102;
    ui_g_30HZ_BUFFER_NUM->width = 2;
    ui_g_30HZ_BUFFER_NUM->font_size = 20;
    ui_g_30HZ_BUFFER_NUM->number = 12345;

    ui_g_30HZ_POWER_METER->figure_type = 2;
    ui_g_30HZ_POWER_METER->operate_type = 1;
    ui_g_30HZ_POWER_METER->layer = 0;
    ui_g_30HZ_POWER_METER->color = 4;
    ui_g_30HZ_POWER_METER->start_x = 1900;
    ui_g_30HZ_POWER_METER->start_y = 636;
    ui_g_30HZ_POWER_METER->width = 10;
    ui_g_30HZ_POWER_METER->r = 5;

    ui_g_30HZ_485->figure_type = 2;
    ui_g_30HZ_485->operate_type = 1;
    ui_g_30HZ_485->layer = 0;
    ui_g_30HZ_485->color = 4;
    ui_g_30HZ_485->start_x = 1900;
    ui_g_30HZ_485->start_y = 612;
    ui_g_30HZ_485->width = 10;
    ui_g_30HZ_485->r = 5;

    ui_g_30HZ_UNNAME1->figure_type = 2;
    ui_g_30HZ_UNNAME1->operate_type = 1;
    ui_g_30HZ_UNNAME1->layer = 0;
    ui_g_30HZ_UNNAME1->color = 4;
    ui_g_30HZ_UNNAME1->start_x = 1900;
    ui_g_30HZ_UNNAME1->start_y = 588;
    ui_g_30HZ_UNNAME1->width = 10;
    ui_g_30HZ_UNNAME1->r = 5;

    ui_g_30HZ_UNNAME2->figure_type = 2;
    ui_g_30HZ_UNNAME2->operate_type = 1;
    ui_g_30HZ_UNNAME2->layer = 0;
    ui_g_30HZ_UNNAME2->color = 4;
    ui_g_30HZ_UNNAME2->start_x = 1900;
    ui_g_30HZ_UNNAME2->start_y = 564;
    ui_g_30HZ_UNNAME2->width = 10;
    ui_g_30HZ_UNNAME2->r = 5;

    ui_g_30HZ_UNNAME3->figure_type = 2;
    ui_g_30HZ_UNNAME3->operate_type = 1;
    ui_g_30HZ_UNNAME3->layer = 0;
    ui_g_30HZ_UNNAME3->color = 4;
    ui_g_30HZ_UNNAME3->start_x = 1900;
    ui_g_30HZ_UNNAME3->start_y = 540;
    ui_g_30HZ_UNNAME3->width = 10;
    ui_g_30HZ_UNNAME3->r = 5;

    ui_g_30HZ_3508L->figure_type = 2;
    ui_g_30HZ_3508L->operate_type = 1;
    ui_g_30HZ_3508L->layer = 0;
    ui_g_30HZ_3508L->color = 4;
    ui_g_30HZ_3508L->start_x = 1900;
    ui_g_30HZ_3508L->start_y = 804;
    ui_g_30HZ_3508L->width = 10;
    ui_g_30HZ_3508L->r = 5;


    ui_proc_7_frame(&ui_g_30HZ_2);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_2, sizeof(ui_g_30HZ_2));
}

void _ui_update_g_30HZ_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_2.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_30HZ_2);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_2, sizeof(ui_g_30HZ_2));
}

void _ui_remove_g_30HZ_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_30HZ_2.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_30HZ_2);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_2, sizeof(ui_g_30HZ_2));
}
ui_5_frame_t ui_g_30HZ_3;

ui_interface_round_t *ui_g_30HZ_3508R = (ui_interface_round_t*)&(ui_g_30HZ_3.data[0]);
ui_interface_round_t *ui_g_30HZ_PITCH = (ui_interface_round_t*)&(ui_g_30HZ_3.data[1]);
ui_interface_round_t *ui_g_30HZ_ROLL = (ui_interface_round_t*)&(ui_g_30HZ_3.data[2]);
ui_interface_round_t *ui_g_30HZ_FRIC_L = (ui_interface_round_t*)&(ui_g_30HZ_3.data[3]);
ui_interface_round_t *ui_g_30HZ_FRIC_R = (ui_interface_round_t*)&(ui_g_30HZ_3.data[4]);

void _ui_init_g_30HZ_3() {
    for (int i = 0; i < 5; i++) {
        ui_g_30HZ_3.data[i].figure_name[0] = 0;
        ui_g_30HZ_3.data[i].figure_name[1] = 0;
        ui_g_30HZ_3.data[i].figure_name[2] = i + 21;
        ui_g_30HZ_3.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_30HZ_3.data[i].operate_type = 0;
    }

    ui_g_30HZ_3508R->figure_type = 2;
    ui_g_30HZ_3508R->operate_type = 1;
    ui_g_30HZ_3508R->layer = 0;
    ui_g_30HZ_3508R->color = 4;
    ui_g_30HZ_3508R->start_x = 1900;
    ui_g_30HZ_3508R->start_y = 780;
    ui_g_30HZ_3508R->width = 10;
    ui_g_30HZ_3508R->r = 5;

    ui_g_30HZ_PITCH->figure_type = 2;
    ui_g_30HZ_PITCH->operate_type = 1;
    ui_g_30HZ_PITCH->layer = 0;
    ui_g_30HZ_PITCH->color = 4;
    ui_g_30HZ_PITCH->start_x = 1900;
    ui_g_30HZ_PITCH->start_y = 756;
    ui_g_30HZ_PITCH->width = 10;
    ui_g_30HZ_PITCH->r = 5;

    ui_g_30HZ_ROLL->figure_type = 2;
    ui_g_30HZ_ROLL->operate_type = 1;
    ui_g_30HZ_ROLL->layer = 0;
    ui_g_30HZ_ROLL->color = 4;
    ui_g_30HZ_ROLL->start_x = 1900;
    ui_g_30HZ_ROLL->start_y = 732;
    ui_g_30HZ_ROLL->width = 10;
    ui_g_30HZ_ROLL->r = 5;

    ui_g_30HZ_FRIC_L->figure_type = 2;
    ui_g_30HZ_FRIC_L->operate_type = 1;
    ui_g_30HZ_FRIC_L->layer = 0;
    ui_g_30HZ_FRIC_L->color = 4;
    ui_g_30HZ_FRIC_L->start_x = 1900;
    ui_g_30HZ_FRIC_L->start_y = 708;
    ui_g_30HZ_FRIC_L->width = 10;
    ui_g_30HZ_FRIC_L->r = 5;

    ui_g_30HZ_FRIC_R->figure_type = 2;
    ui_g_30HZ_FRIC_R->operate_type = 1;
    ui_g_30HZ_FRIC_R->layer = 0;
    ui_g_30HZ_FRIC_R->color = 4;
    ui_g_30HZ_FRIC_R->start_x = 1900;
    ui_g_30HZ_FRIC_R->start_y = 684;
    ui_g_30HZ_FRIC_R->width = 10;
    ui_g_30HZ_FRIC_R->r = 5;


    ui_proc_5_frame(&ui_g_30HZ_3);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_3, sizeof(ui_g_30HZ_3));
}

void _ui_update_g_30HZ_3() {
    for (int i = 0; i < 5; i++) {
        ui_g_30HZ_3.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_30HZ_3);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_3, sizeof(ui_g_30HZ_3));
}

void _ui_remove_g_30HZ_3() {
    for (int i = 0; i < 5; i++) {
        ui_g_30HZ_3.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_30HZ_3);
    SEND_MESSAGE((uint8_t *) &ui_g_30HZ_3, sizeof(ui_g_30HZ_3));
}


void ui_init_g_30HZ() {
    _ui_init_g_30HZ_0();
    _ui_init_g_30HZ_1();
    _ui_init_g_30HZ_2();
    _ui_init_g_30HZ_3();
}

void ui_update_g_30HZ() {
    _ui_update_g_30HZ_0();
    _ui_update_g_30HZ_1();
    _ui_update_g_30HZ_2();
    _ui_update_g_30HZ_3();
}

void ui_remove_g_30HZ() {
    _ui_remove_g_30HZ_0();
    _ui_remove_g_30HZ_1();
    _ui_remove_g_30HZ_2();
    _ui_remove_g_30HZ_3();
}


ui_string_frame_t ui_g_5HZ_0;
ui_interface_string_t* ui_g_5HZ_NewText = &(ui_g_5HZ_0.option);

void _ui_init_g_5HZ_0() {
    ui_g_5HZ_0.option.figure_name[0] = 0;
    ui_g_5HZ_0.option.figure_name[1] = 1;
    ui_g_5HZ_0.option.figure_name[2] = 0;
    ui_g_5HZ_0.option.operate_type = 1;

    ui_g_5HZ_NewText->figure_type = 7;
    ui_g_5HZ_NewText->operate_type = 1;
    ui_g_5HZ_NewText->layer = 0;
    ui_g_5HZ_NewText->color = 4;
    ui_g_5HZ_NewText->start_x = 743;
    ui_g_5HZ_NewText->start_y = 852;
    ui_g_5HZ_NewText->width = 4;
    ui_g_5HZ_NewText->font_size = 40;
    ui_g_5HZ_NewText->str_length = 11;
    strcpy(ui_g_5HZ_NewText->string, "PLEASE SPIN");


    ui_proc_string_frame(&ui_g_5HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_0, sizeof(ui_g_5HZ_0));
}

void _ui_update_g_5HZ_0() {
    ui_g_5HZ_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_5HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_0, sizeof(ui_g_5HZ_0));
}

void _ui_remove_g_5HZ_0() {
    ui_g_5HZ_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_5HZ_0);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_0, sizeof(ui_g_5HZ_0));
}
ui_string_frame_t ui_g_5HZ_1;
ui_interface_string_t* ui_g_5HZ_NewText2 = &(ui_g_5HZ_1.option);

void _ui_init_g_5HZ_1() {
    ui_g_5HZ_1.option.figure_name[0] = 0;
    ui_g_5HZ_1.option.figure_name[1] = 1;
    ui_g_5HZ_1.option.figure_name[2] = 1;
    ui_g_5HZ_1.option.operate_type = 1;

    ui_g_5HZ_NewText2->figure_type = 7;
    ui_g_5HZ_NewText2->operate_type = 1;
    ui_g_5HZ_NewText2->layer = 0;
    ui_g_5HZ_NewText2->color = 4;
    ui_g_5HZ_NewText2->start_x = 1346;
    ui_g_5HZ_NewText2->start_y = 852;
    ui_g_5HZ_NewText2->width = 4;
    ui_g_5HZ_NewText2->font_size = 40;
    ui_g_5HZ_NewText2->str_length = 8;
    strcpy(ui_g_5HZ_NewText2->string, "LONG LEG");


    ui_proc_string_frame(&ui_g_5HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_1, sizeof(ui_g_5HZ_1));
}

void _ui_update_g_5HZ_1() {
    ui_g_5HZ_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_5HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_1, sizeof(ui_g_5HZ_1));
}

void _ui_remove_g_5HZ_1() {
    ui_g_5HZ_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_5HZ_1);
    SEND_MESSAGE((uint8_t *) &ui_g_5HZ_1, sizeof(ui_g_5HZ_1));
}

void ui_init_g_5HZ() {
    _ui_init_g_5HZ_0();
    _ui_init_g_5HZ_1();
}

void ui_update_g_5HZ() {
    _ui_update_g_5HZ_0();
    _ui_update_g_5HZ_1();
}

void ui_remove_g_5HZ() {
    _ui_remove_g_5HZ_0();
    _ui_remove_g_5HZ_1();
}

ui_5_frame_t ui_g_INIT_0;

ui_interface_line_t *ui_g_INIT_NewLine = (ui_interface_line_t*)&(ui_g_INIT_0.data[0]);
ui_interface_line_t *ui_g_INIT_NewLine2 = (ui_interface_line_t*)&(ui_g_INIT_0.data[1]);
ui_interface_line_t *ui_g_INIT_NewLine3 = (ui_interface_line_t*)&(ui_g_INIT_0.data[2]);
ui_interface_line_t *ui_g_INIT_NewLine4 = (ui_interface_line_t*)&(ui_g_INIT_0.data[3]);
ui_interface_line_t *ui_g_INIT_NewLine5 = (ui_interface_line_t*)&(ui_g_INIT_0.data[4]);

void _ui_init_g_INIT_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_INIT_0.data[i].figure_name[0] = 0;
        ui_g_INIT_0.data[i].figure_name[1] = 2;
        ui_g_INIT_0.data[i].figure_name[2] = i + 0;
        ui_g_INIT_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_INIT_0.data[i].operate_type = 0;
    }

    ui_g_INIT_NewLine->figure_type = 0;
    ui_g_INIT_NewLine->operate_type = 1;
    ui_g_INIT_NewLine->layer = 0;
    ui_g_INIT_NewLine->color = 8;
    ui_g_INIT_NewLine->start_x = 571;
    ui_g_INIT_NewLine->start_y = 101;
    ui_g_INIT_NewLine->width = 1;
    ui_g_INIT_NewLine->end_x = 824;
    ui_g_INIT_NewLine->end_y = 412;

    ui_g_INIT_NewLine2->figure_type = 0;
    ui_g_INIT_NewLine2->operate_type = 1;
    ui_g_INIT_NewLine2->layer = 0;
    ui_g_INIT_NewLine2->color = 8;
    ui_g_INIT_NewLine2->start_x = 1345;
    ui_g_INIT_NewLine2->start_y = 92;
    ui_g_INIT_NewLine2->width = 1;
    ui_g_INIT_NewLine2->end_x = 1066;
    ui_g_INIT_NewLine2->end_y = 395;

    ui_g_INIT_NewLine3->figure_type = 0;
    ui_g_INIT_NewLine3->operate_type = 1;
    ui_g_INIT_NewLine3->layer = 0;
    ui_g_INIT_NewLine3->color = 8;
    ui_g_INIT_NewLine3->start_x = 1023;
    ui_g_INIT_NewLine3->start_y = 688;
    ui_g_INIT_NewLine3->width = 1;
    ui_g_INIT_NewLine3->end_x = 883;
    ui_g_INIT_NewLine3->end_y = 690;

    ui_g_INIT_NewLine4->figure_type = 0;
    ui_g_INIT_NewLine4->operate_type = 1;
    ui_g_INIT_NewLine4->layer = 0;
    ui_g_INIT_NewLine4->color = 8;
    ui_g_INIT_NewLine4->start_x = 1025;
    ui_g_INIT_NewLine4->start_y = 641;
    ui_g_INIT_NewLine4->width = 1;
    ui_g_INIT_NewLine4->end_x = 885;
    ui_g_INIT_NewLine4->end_y = 643;

    ui_g_INIT_NewLine5->figure_type = 0;
    ui_g_INIT_NewLine5->operate_type = 1;
    ui_g_INIT_NewLine5->layer = 0;
    ui_g_INIT_NewLine5->color = 8;
    ui_g_INIT_NewLine5->start_x = 626;
    ui_g_INIT_NewLine5->start_y = 278;
    ui_g_INIT_NewLine5->width = 1;
    ui_g_INIT_NewLine5->end_x = 1216;
    ui_g_INIT_NewLine5->end_y = 285;


    ui_proc_5_frame(&ui_g_INIT_0);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_0, sizeof(ui_g_INIT_0));
}

void _ui_update_g_INIT_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_INIT_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_INIT_0);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_0, sizeof(ui_g_INIT_0));
}

void _ui_remove_g_INIT_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_INIT_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_INIT_0);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_0, sizeof(ui_g_INIT_0));
}

ui_string_frame_t ui_g_INIT_1;
ui_interface_string_t* ui_g_INIT_FRIC_SPD = &(ui_g_INIT_1.option);

void _ui_init_g_INIT_1() {
    ui_g_INIT_1.option.figure_name[0] = 0;
    ui_g_INIT_1.option.figure_name[1] = 2;
    ui_g_INIT_1.option.figure_name[2] = 5;
    ui_g_INIT_1.option.operate_type = 1;

    ui_g_INIT_FRIC_SPD->figure_type = 7;
    ui_g_INIT_FRIC_SPD->operate_type = 1;
    ui_g_INIT_FRIC_SPD->layer = 0;
    ui_g_INIT_FRIC_SPD->color = 1;
    ui_g_INIT_FRIC_SPD->start_x = 60;
    ui_g_INIT_FRIC_SPD->start_y = 870;
    ui_g_INIT_FRIC_SPD->width = 2;
    ui_g_INIT_FRIC_SPD->font_size = 20;
    ui_g_INIT_FRIC_SPD->str_length = 8;
    strcpy(ui_g_INIT_FRIC_SPD->string, "FRIC SPD");


    ui_proc_string_frame(&ui_g_INIT_1);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_1, sizeof(ui_g_INIT_1));
}

void _ui_update_g_INIT_1() {
    ui_g_INIT_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_INIT_1);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_1, sizeof(ui_g_INIT_1));
}

void _ui_remove_g_INIT_1() {
    ui_g_INIT_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_INIT_1);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_1, sizeof(ui_g_INIT_1));
}
ui_string_frame_t ui_g_INIT_2;
ui_interface_string_t* ui_g_INIT_AUTO_AIM = &(ui_g_INIT_2.option);

void _ui_init_g_INIT_2() {
    ui_g_INIT_2.option.figure_name[0] = 0;
    ui_g_INIT_2.option.figure_name[1] = 2;
    ui_g_INIT_2.option.figure_name[2] = 6;
    ui_g_INIT_2.option.operate_type = 1;

    ui_g_INIT_AUTO_AIM->figure_type = 7;
    ui_g_INIT_AUTO_AIM->operate_type = 1;
    ui_g_INIT_AUTO_AIM->layer = 0;
    ui_g_INIT_AUTO_AIM->color = 1;
    ui_g_INIT_AUTO_AIM->start_x = 60;
    ui_g_INIT_AUTO_AIM->start_y = 835;
    ui_g_INIT_AUTO_AIM->width = 2;
    ui_g_INIT_AUTO_AIM->font_size = 20;
    ui_g_INIT_AUTO_AIM->str_length = 8;
    strcpy(ui_g_INIT_AUTO_AIM->string, "AUTO AIM");


    ui_proc_string_frame(&ui_g_INIT_2);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_2, sizeof(ui_g_INIT_2));
}

void _ui_update_g_INIT_2() {
    ui_g_INIT_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_INIT_2);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_2, sizeof(ui_g_INIT_2));
}

void _ui_remove_g_INIT_2() {
    ui_g_INIT_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_INIT_2);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_2, sizeof(ui_g_INIT_2));
}
ui_string_frame_t ui_g_INIT_3;
ui_interface_string_t* ui_g_INIT_SHOOT_NUM = &(ui_g_INIT_3.option);

void _ui_init_g_INIT_3() {
    ui_g_INIT_3.option.figure_name[0] = 0;
    ui_g_INIT_3.option.figure_name[1] = 2;
    ui_g_INIT_3.option.figure_name[2] = 7;
    ui_g_INIT_3.option.operate_type = 1;

    ui_g_INIT_SHOOT_NUM->figure_type = 7;
    ui_g_INIT_SHOOT_NUM->operate_type = 1;
    ui_g_INIT_SHOOT_NUM->layer = 0;
    ui_g_INIT_SHOOT_NUM->color = 1;
    ui_g_INIT_SHOOT_NUM->start_x = 60;
    ui_g_INIT_SHOOT_NUM->start_y = 800;
    ui_g_INIT_SHOOT_NUM->width = 2;
    ui_g_INIT_SHOOT_NUM->font_size = 20;
    ui_g_INIT_SHOOT_NUM->str_length = 9;
    strcpy(ui_g_INIT_SHOOT_NUM->string, "SHOOT NUM");


    ui_proc_string_frame(&ui_g_INIT_3);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_3, sizeof(ui_g_INIT_3));
}

void _ui_update_g_INIT_3() {
    ui_g_INIT_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_INIT_3);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_3, sizeof(ui_g_INIT_3));
}

void _ui_remove_g_INIT_3() {
    ui_g_INIT_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_INIT_3);
    SEND_MESSAGE((uint8_t *) &ui_g_INIT_3, sizeof(ui_g_INIT_3));
}

void ui_init_g_INIT() {
    _ui_init_g_INIT_0();
    _ui_init_g_INIT_1();
    _ui_init_g_INIT_2();
    _ui_init_g_INIT_3();
}

void ui_update_g_INIT() {
    _ui_update_g_INIT_0();
    _ui_update_g_INIT_1();
    _ui_update_g_INIT_2();
    _ui_update_g_INIT_3();
}

void ui_remove_g_INIT() {
    _ui_remove_g_INIT_0();
    _ui_remove_g_INIT_1();
    _ui_remove_g_INIT_2();
    _ui_remove_g_INIT_3();
}

