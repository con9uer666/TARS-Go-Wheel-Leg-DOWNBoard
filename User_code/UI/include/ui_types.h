//
// Created by bismarckkk on 2024/2/17.
//
// ============================================================================
//  ui_types.h —— UI 模块"类型层"
//  ----------------------------------------------------------------------------
//  作用：定义裁判系统"客户端绘图"协议里用到的全部结构体（位域 + packed），
//        包括 6 种几何图元（figure/line/rect/round/ellipse/arc）、
//        number/string 两种特殊数据图元，以及最终上链路的 1/2/5/7-图元帧
//        与字符串帧。
//
//  上层依赖关系：
//     ui_interface.{c,h}  → 使用 ui_*_frame_t 做封口 (CRC、长度、cmd_id)
//     ui_g.{c,h}          → 把这些指针指向静态帧缓冲，用作"图层"
//     UI_Task.c           → 通过这些指针修改图元属性
// ============================================================================

#ifndef UI_TYPES_H
#define UI_TYPES_H

// User Code Begin

// MESSAGE_PACKED：让结构体按 1 字节对齐打包，符合裁判系统线协议要求。
// 这里只支持 GCC / ARMCC（__CC_ARM），其它编译器直接报错避免静默 bug。
#if defined(__GNUC__) || defined(__CC_ARM)
#define MESSAGE_PACKED __attribute__((packed))
#include <stdint.h>
#else
#error "MESSAGE_PACKED not defined for this compiler"
#endif

// User Code End

// ----- 宏拼接小工具：把 x 与 y 拼成一个新的标识符 ----------------------------
// 例：CAT(_, end_x) -> _end_x。两层间接展开是 C 预处理器要求的标准写法，
//     PRIMITIVE_CAT 是"实拼接"，CAT 多一层使得参数本身先被展开。
#define PRIMITIVE_CAT(x, y) x ## y
#define CAT(x, y) PRIMITIVE_CAT(x, y)

// ----- 通用几何图元帧结构定义宏 ---------------------------------------------
// 协议规定每个图元 15 字节，由头部 3 字节 figure_name + 7 个共享字段
// (operate_type/figure_type/layer/color/起点/线宽) + 5 个图元特有字段
// (p_a~p_e) 组成。不同图元对 p_a~p_e 的解释不同，用宏统一生成。
//
//   name : 图元名称（生成 ui_interface_<name>_t）
//   p_a~p_e : 该图元 5 个尾部字段的实际字段名（占位 _a~_e 表示"留空字段"）
//
// 字段位域说明（按发送顺序）：
//   figure_name[3]  : 图元唯一标识（3 字节），由上层分配
//   operate_type:3  : 1=新增，2=修改，3=删除，0=空操作
//   figure_type:3   : 0=直线 1=矩形 2=圆 3=椭圆 4=圆弧 5=浮点 6=整数 7=字符
//   layer:4         : 图层 0~9
//   color:4         : 颜色编号（裁判系统枚举）
//   p_a/p_b:9       : 图元特有字段（如圆弧起始角等）
//   width:10        : 线宽
//   start_x:11      : 起点 X (0~2047)
//   start_y:11      : 起点 Y (0~2047)
//   p_c:10 / p_d:11 / p_e:11 : 图元特有字段
#define DEFINE_MESSAGE(name, p_a, p_b, p_c, p_d, p_e)   \
typedef struct {                                        \
uint8_t figure_name[3];                                 \
uint32_t operate_type:3;                                \
uint32_t figure_type:3;                                 \
uint32_t layer:4;                                       \
uint32_t color:4;                                       \
uint32_t PRIMITIVE_CAT(,p_a) :9;                        \
uint32_t PRIMITIVE_CAT(,p_b):9;                         \
uint32_t width:10;                                      \
uint32_t start_x:11;                                    \
uint32_t start_y:11;                                    \
uint32_t PRIMITIVE_CAT(,p_c):10;                        \
uint32_t PRIMITIVE_CAT(,p_d):11;                        \
uint32_t PRIMITIVE_CAT(,p_e):11;                        \
} MESSAGE_PACKED ui_interface_ ## name ##_t

// 6 类几何图元结构体的实际生成。
// 注：figure 是"通用占位"，5 个特殊字段全留空，ui_g.c 中把每个 data[i] 都
//     声明为 ui_interface_figure_t，再按需 cast 成具体子类（round/line 等）。
DEFINE_MESSAGE(figure,  _a,          _b,         _c, _d,    _e);    // 通用占位
DEFINE_MESSAGE(line,    _a,          _b,         _c, end_x, end_y); // 直线：终点 (end_x, end_y)
DEFINE_MESSAGE(rect,    _a,          _b,         _c, end_x, end_y); // 矩形：对角终点
DEFINE_MESSAGE(round,   _a,          _b,         r,  _d,    _e);    // 圆：半径 r
DEFINE_MESSAGE(ellipse, _a,          _b,         _c, rx,    ry);    // 椭圆：长短半轴 rx/ry
DEFINE_MESSAGE(arc,     start_angle, end_angle,  _c, rx,    ry);    // 圆弧：起止角 + 半轴

// ----- 整数图元结构 ---------------------------------------------------------
// 图元类型固定 figure_type=6，内容为一个 int32_t number；font_size 控制字号；
// _b 字段保留对齐协议位。
typedef struct {
    uint8_t figure_name[3];   // 图元唯一标识
    uint32_t operate_type: 3; // 1=新增 2=修改 3=删除 0=空
    uint32_t figure_type: 3;  // 必须=6
    uint32_t layer: 4;        // 图层
    uint32_t color: 4;        // 颜色
    uint32_t font_size: 9;    // 字号
    uint32_t _b: 9;           // 保留位
    uint32_t width: 10;       // 线宽（数字也支持加粗）
    uint32_t start_x: 11;     // 文本起点 X
    uint32_t start_y: 11;     // 文本起点 Y
    int32_t number;           // 要显示的整数
} MESSAGE_PACKED ui_interface_number_t;

// ----- 字符串图元结构 -------------------------------------------------------
// 图元类型固定 figure_type=7，最多 30 字节字符；
// str_length 必须等于 strlen(string)，否则客户端可能不渲染。
typedef struct {
    uint8_t figure_name[3];   // 图元唯一标识
    uint32_t operate_type: 3; // 1/2/3/0
    uint32_t figure_type: 3;  // 必须=7
    uint32_t layer: 4;        // 图层
    uint32_t color: 4;        // 颜色
    uint32_t font_size: 9;    // 字号
    uint32_t str_length: 9;   // 实际字符数（含 \0 前的有效字节数）
    uint32_t width: 10;       // 线宽
    uint32_t start_x: 11;     // 起点 X
    uint32_t start_y: 11;     // 起点 Y
    uint32_t _c: 10;          // 保留
    uint32_t _d: 11;          // 保留
    uint32_t _e: 11;          // 保留
    char string[30];          // 字符串内容（不必 \0 结尾，但写入时通常 strcpy）
} MESSAGE_PACKED ui_interface_string_t;

// ----- 裁判系统通用帧头 -----------------------------------------------------
// SOF 恒为 0xA5；length=本帧数据段长度（不含帧头/CRC16）；
// seq 自增；crc8 校验前 4 字节（SOF/length/seq）；
// cmd_id 恒为 0x0301（机器人间交互/绘图）；sub_id 区分 1/2/5/7 图元或字符串；
// send_id 当前机器人 id；recv_id = send_id + 256（操作手客户端约定）。
typedef struct {
    uint8_t SOF;              // 起始字节，固定 0xA5
    uint16_t length;          // 数据段长度
    uint8_t seq, crc8;        // 包序号 + CRC8(头部前 4 字节)
    uint16_t cmd_id, sub_id;  // 主/子命令码
    uint16_t send_id, recv_id;// 发送/接收方 id
} MESSAGE_PACKED ui_frame_header_t;

// ----- 多图元帧批量定义宏 ---------------------------------------------------
// num 是每帧承载的图元数（1/2/5/7），由协议定死的几种打包模式。
// 生成的结构体长度 = 9(header) + 15*num + 2(crc16)。
#define DEFINE_FIGURE_MESSAGE(num)      \
typedef struct {                        \
ui_frame_header_t header;               \
ui_interface_figure_t data[num];        \
uint16_t crc16;                         \
} MESSAGE_PACKED ui_ ## num##_frame_t

DEFINE_FIGURE_MESSAGE(1); // ui_1_frame_t：1 图元帧 (sub_id=0x0101)
DEFINE_FIGURE_MESSAGE(2); // ui_2_frame_t：2 图元帧 (sub_id=0x0102)
DEFINE_FIGURE_MESSAGE(5); // ui_5_frame_t：5 图元帧 (sub_id=0x0103)
DEFINE_FIGURE_MESSAGE(7); // ui_7_frame_t：7 图元帧 (sub_id=0x0104)

// ----- 字符串帧 -------------------------------------------------------------
// 字符串图元单独走 sub_id=0x0110，固定长度 51。
typedef struct {
    ui_frame_header_t header;        // 通用帧头
    ui_interface_string_t option;    // 字符串图元体
    uint16_t crc16;                  // 整包 CRC16
} MESSAGE_PACKED ui_string_frame_t;

#endif //UI_TYPES_H
