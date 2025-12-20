#include "Graphics.h"
#include "Judge.h"
#include <stdarg.h>
#include <string.h>

//配置一条直线
void Graph_SetLine(Line *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x1,int16_t y1,int16_t x2,int16_t y2)
{
	//将传入参数依次写入结构体
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->startPoint.x=x1;
	shape->startPoint.y=y1;
	shape->endPoint.x=x2;
	shape->endPoint.y=y2;
}

//绘制一条直线
void Graph_DrawLine(Line *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;//用于裁判系统发送的结构体
	//将shape中各数据放入graphData中
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Line;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->startPoint.x;
	graphData.start_y=shape->startPoint.y;
	graphData.end_x=shape->endPoint.x;
	graphData.end_y=shape->endPoint.y;
	//向裁判系统发出数据
	JUDGE_SendGraphStruct(&graphData);
}

//配置一个矩形
void Graph_SetRect(Rect *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x1,int16_t y1,int16_t x2,int16_t y2)
{
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->startPoint.x=x1;
	shape->startPoint.y=y1;
	shape->endPoint.x=x2;
	shape->endPoint.y=y2;
}

//绘制一个矩形
void Graph_DrawRect(Rect *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Rect;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->startPoint.x;
	graphData.start_y=shape->startPoint.y;
	graphData.end_x=shape->endPoint.x;
	graphData.end_y=shape->endPoint.y;
	JUDGE_SendGraphStruct(&graphData);
}

//配置一个圆形
void Graph_SetCircle(Circle *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t radius)
{
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->centPoint.x=x;
	shape->centPoint.y=y;
	shape->radius=radius;
}

//绘制一个圆形
void Graph_DrawCircle(Circle *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Circle;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->centPoint.x;
	graphData.start_y=shape->centPoint.y;
	graphData.radius=shape->radius;
	JUDGE_SendGraphStruct(&graphData);
}

//配置一个椭圆
void Graph_SetOval(Oval *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t semiaxisX,int16_t semiaxisY)
{
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->centPoint.x=x;
	shape->centPoint.y=y;
	shape->semiaxis.x=semiaxisX;
	shape->semiaxis.y=semiaxisY;
}

//绘制一个椭圆
void Graph_DrawOval(Oval *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Oval;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->centPoint.x;
	graphData.start_y=shape->centPoint.y;
	graphData.end_x=shape->semiaxis.x;
	graphData.end_y=shape->semiaxis.y;
	JUDGE_SendGraphStruct(&graphData);
}

//配置一个弧形
void Graph_SetArc(Arc *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t semiaxisX,int16_t semiaxisY,int16_t startAngle,int16_t endAngle)
{
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->centPoint.x=x;
	shape->centPoint.y=y;
	shape->semiaxis.x=semiaxisX;
	shape->semiaxis.y=semiaxisY;
	shape->startAngle=startAngle;
	shape->endAngle=endAngle;
}

//绘制一个弧形
void Graph_DrawArc(Arc *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Arc;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->centPoint.x;
	graphData.start_y=shape->centPoint.y;
	graphData.end_x=shape->semiaxis.x;
	graphData.end_y=shape->semiaxis.y;
	graphData.start_angle=shape->startAngle;
	graphData.end_angle=shape->endAngle;
	JUDGE_SendGraphStruct(&graphData);
}

//配置一个浮点数据图形
void Graph_SetFloat(FloatShape *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,float value,uint8_t digit,uint16_t size)
{
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->pos.x=x;
	shape->pos.y=y;
	shape->value=value;
	shape->digit=digit;
	shape->size=size;
}

//绘制一个浮点数据图形
void Graph_DrawFloat(FloatShape *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Float;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->pos.x;
	graphData.start_y=shape->pos.y;
	graphData.start_angle=shape->size;
	graphData.end_angle=shape->digit;
	graphData.radius=((int32_t)(shape->value*1000))&0x3FF;
	graphData.end_x=((int32_t)(shape->value*1000)>>10)&0x7FF;
	graphData.end_y=((int32_t)(shape->value*1000)>>21)&0x7FF;
	JUDGE_SendGraphStruct(&graphData);
}

//配置一个整型数据图形
void Graph_SetInt(IntShape *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int32_t value,uint16_t size)
{
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->pos.x=x;
	shape->pos.y=y;
	shape->value=value;
	shape->size=size;
}

//绘制一个整型数据图形
void Graph_DrawInt(IntShape *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Int;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->pos.x;
	graphData.start_y=shape->pos.y;
	graphData.start_angle=shape->size;
	graphData.radius=shape->value&0x3FF;
	graphData.end_x=(shape->value>>10)&0x7FF;
	graphData.end_y=(shape->value>>21)&0x7FF;
	JUDGE_SendGraphStruct(&graphData);
}

//配置一个字符图形
void Graph_SetText(Text *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,char *text,uint16_t len,uint16_t size)
{
	shape->name=name;
	shape->color=color;
	shape->width=width;
	shape->layer=layer;
	shape->pos.x=x;
	shape->pos.y=y;
	if(len>30)
		len=30;
	memcpy(shape->data,text,len);
	shape->length=len;
	shape->size=size;
}

//绘制一个字符图形
void Graph_DrawText(Text *shape,GraphOperation opera)
{
	graphic_data_struct_t graphData;
	memcpy(graphData.graphic_name,shape->name,3);
	graphData.operate_tpye=opera;
	graphData.graphic_tpye=Shape_Text;
	graphData.layer=shape->layer;
	graphData.color=shape->color;
	graphData.width=shape->width;
	graphData.start_x=shape->pos.x;
	graphData.start_y=shape->pos.y;
	graphData.start_angle=shape->size;
	graphData.end_angle=shape->length;
	JUDGE_SendTextStruct(&graphData,shape->data,30);
}
