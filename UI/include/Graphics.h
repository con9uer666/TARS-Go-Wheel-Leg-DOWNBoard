#ifndef _GRAPHICS_H_
#define _GRAPHICS_H_

#include "main.h"

//图形操作
typedef enum _GraphOperation
{
	Operation_Null=0,
	Operation_Add,
	Operation_Change,
	Operation_Delete
}GraphOperation;

//图形形状
typedef enum _GraphShape
{
	Shape_Line=0,
	Shape_Rect,
	Shape_Circle,
	Shape_Oval,
	Shape_Arc,
	Shape_Int,
	Shape_Float,
	Shape_Text
}GraphShape;

//图形颜色
typedef enum _GraphColor
{
	Color_Self=0,//己方主色
	Color_Yellow,
	Color_Green,
	Color_Orange,
	Color_Purple,
	Color_Pink,
	Color_Cyan,
	Color_Black,
	Color_White
}GraphColor;

//数对，可作为一个二维向量
typedef struct NumPair
{
	int16_t x,y;
}Point,Vector;

//直线
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point startPoint,endPoint;//起始和结束坐标
}Line;

//矩形
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point startPoint,endPoint;//起始和结束坐标
}Rect;

//正圆
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point centPoint;//圆心坐标
	uint16_t radius;//半径
}Circle;

//椭圆
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point centPoint;//圆心坐标
	Vector semiaxis;//xy半轴长
}Oval;

//圆弧
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point centPoint;//圆心坐标
	Vector semiaxis;//xy半轴长
	int16_t startAngle,endAngle;//起始、终止角度
}Arc;

//浮点数
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point pos;//位置坐标
	float value;//值
	uint8_t digit;//有效位数
	uint16_t size;//字体大小
}FloatShape;

//整型数
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point pos;//位置坐标
	int32_t value;//值
	uint16_t size;//字体大小
}IntShape;

//字符串
typedef struct
{
	char *name;//名字
	GraphColor color;//颜色
	uint8_t width;//粗细
	uint8_t layer;//所在图层数
	
	Point pos;//位置坐标
	uint8_t data[30];//值
	uint16_t length;//字符数量
	uint16_t size;//字体大小
}Text;

void Graph_SetLine(Line *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x1,int16_t y1,int16_t x2,int16_t y2);
void Graph_DrawLine(Line *shape,GraphOperation opera);
void Graph_SetRect(Rect *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x1,int16_t y1,int16_t x2,int16_t y2);
void Graph_DrawRect(Rect *shape,GraphOperation opera);
void Graph_SetCircle(Circle *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t radius);
void Graph_DrawCircle(Circle *shape,GraphOperation opera);
void Graph_SetOval(Oval *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t semiaxisX,int16_t semiaxisY);
void Graph_DrawOval(Oval *shape,GraphOperation opera);
void Graph_SetArc(Arc *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t semiaxisX,int16_t semiaxisY,int16_t startAngle,int16_t endAngle);
void Graph_DrawArc(Arc *shape,GraphOperation opera);
void Graph_SetFloat(FloatShape *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,float value,uint8_t digit,uint16_t size);
void Graph_DrawFloat(FloatShape *shape,GraphOperation opera);
void Graph_SetInt(IntShape *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int32_t value,uint16_t size);
void Graph_DrawInt(IntShape *shape,GraphOperation opera);
void Graph_SetText(Text *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,char *text,uint16_t len,uint16_t size);
void Graph_DrawText(Text *shape,GraphOperation opera);

#endif
