//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : Evk123
//  生成日期   : 2014-0101
//  最近修改   :
//  功能描述   : 0.69寸OLED 接口演示例程(STM32F103ZE系列IIC)
//              说明:
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PD6（SCL）
//              SDA   接PD7（SDA）
//              ----------------------------------------------------------------
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////�

#include "oled.h"
#include "stdlib.h"
#include "delay.h"
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127

/**********************************************
// IIC Write byte
**********************************************/

void Write_IIC_Byte(unsigned char IIC_Byte)
{
    unsigned char i;
    unsigned char m, da;
    da = IIC_Byte;
    OLED_SCLK_Clr();
    for (i = 0; i < 8; i++)
    {
        m = da;
        //	OLED_SCLK_Clr();
        m = m & 0x80;
        if (m == 0x80)
        {
            OLED_SDIN_Set();
        }
        else
            OLED_SDIN_Clr();
        da = da << 1;
        OLED_SCLK_Set();
        OLED_SCLK_Clr();
    }
}
/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
    IIC_Start();
    Write_IIC_Byte(0x78); //Slave address,SA0=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x00); //write command
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Command);
    IIC_Wait_Ack();
    IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
    IIC_Start();
    Write_IIC_Byte(0x78); //D/C#=0; R/W#=0
    IIC_Wait_Ack();
    Write_IIC_Byte(0x40); //write data
    IIC_Wait_Ack();
    Write_IIC_Byte(IIC_Data);
    IIC_Wait_Ack();
    IIC_Stop();
}
void OLED_WR_Byte(unsigned dat, unsigned cmd)
{
    if (cmd)
    {

        Write_IIC_Data(dat);
    }
    else
    {
        Write_IIC_Command(dat);
    }
}

/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
    unsigned char m, n;
    for (m = 0; m < 8; m++)
    {
        OLED_WR_Byte(0xb0 + m, 0); //page0-page1
        OLED_WR_Byte(0x00, 0);     //low column start address
        OLED_WR_Byte(0x10, 0);     //high column start address
        for (n = 0; n < 128; n++)
        {
            OLED_WR_Byte(fill_Data, 1);
        }
    }
}

/***********************Delay****************************************/
void Delay_50ms(unsigned int Del_50ms)
{
    unsigned int m;
    for (; Del_50ms > 0; Del_50ms--)
        for (m = 6245; m > 0; m--)
            ;
}

void Delay_1ms(unsigned int Del_1ms)
{
    unsigned char j;
    while (Del_1ms--)
    {
        for (j = 0; j < 123; j++)
            ;
    }
}

//坐标设置

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}
//开启OLED显示
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC命令
    OLED_WR_Byte(0X14, OLED_CMD); //DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD); //DISPLAY ON
}
//关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC命令
    OLED_WR_Byte(0X10, OLED_CMD); //DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD); //DISPLAY OFF
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
    u8 i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置页地址（0~7）
        OLED_WR_Byte(0x00, OLED_CMD);     //设置显示位置—列低地址
        OLED_WR_Byte(0x10, OLED_CMD);     //设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    } //更新显示
}
void OLED_On(void)
{
    u8 i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); //设置页地址（0~7）
        OLED_WR_Byte(0x00, OLED_CMD);     //设置显示位置—列低地址
        OLED_WR_Byte(0x10, OLED_CMD);     //设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(1, OLED_DATA);
    } //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 Char_Size)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; //得到偏移后的值
    if (x > Max_Column - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
    else
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_WR_Byte(F6x8[c][i], OLED_DATA);
    }
}
//m^n函数
u32 oled_pow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--)
        result *= m;
    return result;
}
//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size2)
{
    u8 t, temp;
    u8 enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);
    }
}

/**
 * 显示SHORT型数字 -32768 ~ 32767
 */
void OLED_ShowShort(u8 *x, u8 *y, short num)
{
    short tmpNum = num / 10;
    if (tmpNum == 0)
    {
        OLED_ShowChar(*x, *y, num + 0x30, 8);
        *x += 6;
        return;
    }
    OLED_ShowShort(x, y, tmpNum);
    OLED_ShowChar(*x, *y, (num % 10) + 0x30, 8);
    *x += 6;
}

//显示一个字符号串
void OLED_ShowString(u8 x, u8 y, char *chr, u8 Char_Size)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], Char_Size);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}
//显示汉字
void OLED_ShowCHinese(u8 x, u8 y, u8 no)
{
    u8 t, adder = 0;
    OLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * no][t], OLED_DATA);
        adder += 1;
    }
    OLED_Set_Pos(x, y + 1);
    for (t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * no + 1][t], OLED_DATA);
        adder += 1;
    }
}

// 显示11像素列汉字
void OLED_ShowCHinese_12(u8 x,u8 y,u8 no)
{
    OLED_Set_Pos(x, y);
    for (u8 t = 0, adder = 0; t < 12; t++, adder++)
    {
        OLED_WR_Byte(hzk12us_font_data[2 * no][t], OLED_DATA);
    }
    OLED_Set_Pos(x, y + 1);
    for (u8 t = 0, adder = 0; t < 12; t++, adder++)
    {
        OLED_WR_Byte(hzk12us_font_data[2 * no + 1][t], OLED_DATA);
    }
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, const unsigned char BMP[])
{
    unsigned int j = 0;
    unsigned char x, y;

    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++)
    {
        OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

//初始化SSD1306
void OLED_Init(void)
{
    delay_ms(800);
    OLED_WR_Byte(0xAE, OLED_CMD); //--display off
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address
    OLED_WR_Byte(0xB0, OLED_CMD); //--set page address
    OLED_WR_Byte(0x81, OLED_CMD); // contract control
    OLED_WR_Byte(0xFF, OLED_CMD); //--128
    OLED_WR_Byte(0xA1, OLED_CMD); //set segment remap
    OLED_WR_Byte(0xA6, OLED_CMD); //--normal / reverse
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3F, OLED_CMD); //--1/32 duty
    OLED_WR_Byte(0xC8, OLED_CMD); //Com scan direction
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset
    OLED_WR_Byte(0x00, OLED_CMD); //

    OLED_WR_Byte(0xD5, OLED_CMD); //set osc division
    OLED_WR_Byte(0x80, OLED_CMD); //

    OLED_WR_Byte(0xD8, OLED_CMD); //set area color mode off
    OLED_WR_Byte(0x05, OLED_CMD); //

    OLED_WR_Byte(0xD9, OLED_CMD); //Set Pre-Charge Period
    OLED_WR_Byte(0xF1, OLED_CMD); //

    OLED_WR_Byte(0xDA, OLED_CMD); //set com pin configuartion
    OLED_WR_Byte(0x12, OLED_CMD); //

    OLED_WR_Byte(0xDB, OLED_CMD); //set Vcomh
    OLED_WR_Byte(0x30, OLED_CMD); //

    OLED_WR_Byte(0x8D, OLED_CMD); //set charge pump enable
    OLED_WR_Byte(0x14, OLED_CMD); //

    OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel
}

const unsigned char F6x8[][6] =
{
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},// sp
{0x00, 0x00, 0x00, 0x2f, 0x00, 0x00},// !
{0x00, 0x00, 0x07, 0x00, 0x07, 0x00},// "
{0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14},// #
{0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12},// $
{0x00, 0x62, 0x64, 0x08, 0x13, 0x23},// %
{0x00, 0x36, 0x49, 0x55, 0x22, 0x50},// &
{0x00, 0x00, 0x05, 0x03, 0x00, 0x00},// '
{0x00, 0x00, 0x1c, 0x22, 0x41, 0x00},// (
{0x00, 0x00, 0x41, 0x22, 0x1c, 0x00},// )
{0x00, 0x14, 0x08, 0x3E, 0x08, 0x14},// *
{0x00, 0x08, 0x08, 0x3E, 0x08, 0x08},// +
{0x00, 0x00, 0x00, 0xA0, 0x60, 0x00},// ,
{0x00, 0x08, 0x08, 0x08, 0x08, 0x08},// -
{0x00, 0x00, 0x60, 0x60, 0x00, 0x00},// .
{0x00, 0x20, 0x10, 0x08, 0x04, 0x02},// /
{0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E},// 0
{0x00, 0x00, 0x42, 0x7F, 0x40, 0x00},// 1
{0x00, 0x42, 0x61, 0x51, 0x49, 0x46},// 2
{0x00, 0x21, 0x41, 0x45, 0x4B, 0x31},// 3
{0x00, 0x18, 0x14, 0x12, 0x7F, 0x10},// 4
{0x00, 0x27, 0x45, 0x45, 0x45, 0x39},// 5
{0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30},// 6
{0x00, 0x01, 0x71, 0x09, 0x05, 0x03},// 7
{0x00, 0x36, 0x49, 0x49, 0x49, 0x36},// 8
{0x00, 0x06, 0x49, 0x49, 0x29, 0x1E},// 9
{0x00, 0x00, 0x36, 0x36, 0x00, 0x00},// :
{0x00, 0x00, 0x56, 0x36, 0x00, 0x00},// ;
{0x00, 0x08, 0x14, 0x22, 0x41, 0x00},// <
{0x00, 0x14, 0x14, 0x14, 0x14, 0x14},// =
{0x00, 0x00, 0x41, 0x22, 0x14, 0x08},// >
{0x00, 0x02, 0x01, 0x51, 0x09, 0x06},// ?
{0x00, 0x32, 0x49, 0x59, 0x51, 0x3E},// @
{0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C},// A
{0x00, 0x7F, 0x49, 0x49, 0x49, 0x36},// B
{0x00, 0x3E, 0x41, 0x41, 0x41, 0x22},// C
{0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C},// D
{0x00, 0x7F, 0x49, 0x49, 0x49, 0x41},// E
{0x00, 0x7F, 0x09, 0x09, 0x09, 0x01},// F
{0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A},// G
{0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F},// H
{0x00, 0x00, 0x41, 0x7F, 0x41, 0x00},// I
{0x00, 0x20, 0x40, 0x41, 0x3F, 0x01},// J
{0x00, 0x7F, 0x08, 0x14, 0x22, 0x41},// K
{0x00, 0x7F, 0x40, 0x40, 0x40, 0x40},// L
{0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F},// M
{0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F},// N
{0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E},// O
{0x00, 0x7F, 0x09, 0x09, 0x09, 0x06},// P
{0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E},// Q
{0x00, 0x7F, 0x09, 0x19, 0x29, 0x46},// R
{0x00, 0x46, 0x49, 0x49, 0x49, 0x31},// S
{0x00, 0x01, 0x01, 0x7F, 0x01, 0x01},// T
{0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F},// U
{0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F},// V
{0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F},// W
{0x00, 0x63, 0x14, 0x08, 0x14, 0x63},// X
{0x00, 0x07, 0x08, 0x70, 0x08, 0x07},// Y
{0x00, 0x61, 0x51, 0x49, 0x45, 0x43},// Z
{0x00, 0x00, 0x7F, 0x41, 0x41, 0x00},// [
{0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55},// 55
{0x00, 0x00, 0x41, 0x41, 0x7F, 0x00},// ]
{0x00, 0x04, 0x02, 0x01, 0x02, 0x04},// ^
{0x00, 0x40, 0x40, 0x40, 0x40, 0x40},// _
{0x00, 0x00, 0x01, 0x02, 0x04, 0x00},// '
{0x00, 0x20, 0x54, 0x54, 0x54, 0x78},// a
{0x00, 0x7F, 0x48, 0x44, 0x44, 0x38},// b
{0x00, 0x38, 0x44, 0x44, 0x44, 0x20},// c
{0x00, 0x38, 0x44, 0x44, 0x48, 0x7F},// d
{0x00, 0x38, 0x54, 0x54, 0x54, 0x18},// e
{0x00, 0x08, 0x7E, 0x09, 0x01, 0x02},// f
{0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C},// g
{0x00, 0x7F, 0x08, 0x04, 0x04, 0x78},// h
{0x00, 0x00, 0x44, 0x7D, 0x40, 0x00},// i
{0x00, 0x40, 0x80, 0x84, 0x7D, 0x00},// j
{0x00, 0x7F, 0x10, 0x28, 0x44, 0x00},// k
{0x00, 0x00, 0x41, 0x7F, 0x40, 0x00},// l
{0x00, 0x7C, 0x04, 0x18, 0x04, 0x78},// m
{0x00, 0x7C, 0x08, 0x04, 0x04, 0x78},// n
{0x00, 0x38, 0x44, 0x44, 0x44, 0x38},// o
{0x00, 0xFC, 0x24, 0x24, 0x24, 0x18},// p
{0x00, 0x18, 0x24, 0x24, 0x18, 0xFC},// q
{0x00, 0x7C, 0x08, 0x04, 0x04, 0x08},// r
{0x00, 0x48, 0x54, 0x54, 0x54, 0x20},// s
{0x00, 0x04, 0x3F, 0x44, 0x40, 0x20},// t
{0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C},// u
{0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C},// v
{0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C},// w
{0x00, 0x44, 0x28, 0x10, 0x28, 0x44},// x
{0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C},// y
{0x00, 0x44, 0x64, 0x54, 0x4C, 0x44},// z
{0x14, 0x14, 0x14, 0x14, 0x14, 0x14}// horiz lines
};
/****************************************8*16的点阵************************************/
const unsigned char F8X16[] =
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//! 1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//" 2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//# 3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$ 4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//% 5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//& 6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//' 7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//( 8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//) 9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//* 10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+ 11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//, 12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//- 13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//. 14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,/// 15
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//0 16
  0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//1 17
  0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//2 18
  0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//3 19
  0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//4 20
  0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//5 21
  0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//6 22
  0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//7 23
  0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//8 24
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//9 25
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//: 26
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//; 27
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//< 28
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//= 29
  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//> 30
  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//? 31
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@ 32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A 33
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B 34
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C 35
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D 36
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E 37
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F 38
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G 39
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H 40
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I 41
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J 42
  0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K 43
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L 44
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M 45
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N 46
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O 47
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P 48
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q 49
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R 50
  0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S 51
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T 52
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U 53
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V 54
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W 55
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X 56
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y 57
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z 58
  0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[ 59
  0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\ 60
  0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//] 61
  0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^ 62
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_ 63
  0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//` 64
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a 65
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b 66
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c 67
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d 68
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e 69
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f 70
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g 71
  0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h 72
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i 73
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j 74
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k 75
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l 76
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m 77
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n 78
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o 79
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p 80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q 81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r 82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s 83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t 84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u 85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v 86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w 87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x 88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y 89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z 90
  0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{ 91
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//| 92
  0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//} 93
  0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~ 94
};
const char Hzk[][32] = {


{0x00,0x00,0xF8,0x88,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x88,0xF8,0x00,0x00,0x00},
{0x00,0x00,0x1F,0x08,0x08,0x08,0x08,0x7F,0x88,0x88,0x88,0x88,0x9F,0x80,0xF0,0x00},/*"电",3*/

{0x80,0x82,0x82,0x82,0x82,0x82,0x82,0xE2,0xA2,0x92,0x8A,0x86,0x82,0x80,0x80,0x00},
{0x00,0x00,0x00,0x00,0x00,0x40,0x80,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"子",4*/

{0x24,0x24,0xA4,0xFE,0xA3,0x22,0x00,0x22,0xCC,0x00,0x00,0xFF,0x00,0x00,0x00,0x00},
{0x08,0x06,0x01,0xFF,0x00,0x01,0x04,0x04,0x04,0x04,0x04,0xFF,0x02,0x02,0x02,0x00},/*"科",5*/

{0x10,0x10,0x10,0xFF,0x10,0x90,0x08,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x08,0x00},
{0x04,0x44,0x82,0x7F,0x01,0x80,0x80,0x40,0x43,0x2C,0x10,0x28,0x46,0x81,0x80,0x00},/*"?",0*/

{0x40,0x40,0x48,0x48,0x48,0xC8,0x78,0x4F,0x48,0x48,0x48,0x48,0x48,0x40,0x40,0x00},
{0x00,0x00,0x00,0x00,0x03,0x12,0x12,0x22,0x22,0x52,0x8A,0x06,0x00,0x00,0x00,0x00},/*"专",0*/

{0x00,0x10,0x60,0x80,0x00,0xFF,0x00,0x00,0x00,0xFF,0x00,0x00,0xC0,0x30,0x00,0x00},
{0x40,0x40,0x40,0x43,0x40,0x7F,0x40,0x40,0x40,0x7F,0x42,0x41,0x40,0x40,0x40,0x00},/*"业",1*/

{0x10,0x10,0x10,0xFF,0x10,0x90,0x08,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x08,0x00},
{0x04,0x44,0x82,0x7F,0x01,0x80,0x80,0x40,0x43,0x2C,0x10,0x28,0x46,0x81,0x80,0x00},/*"技",2*/

{0x00,0x10,0x10,0x10,0x10,0xD0,0x30,0xFF,0x30,0xD0,0x12,0x1C,0x10,0x10,0x00,0x00},
{0x10,0x08,0x04,0x02,0x01,0x00,0x00,0xFF,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x00},/*"术",3*/

{0x00,0x00,0xFE,0x22,0x22,0x22,0xFE,0x00,0xFE,0x82,0x82,0x92,0xA2,0x9E,0x00,0x00},
{0x80,0x60,0x1F,0x02,0x42,0x82,0x7F,0x00,0xFF,0x40,0x2F,0x10,0x2C,0x43,0x80,0x00},/*"服",4*/

{0x00,0x00,0x90,0x88,0x4C,0x57,0xA4,0x24,0x54,0x54,0x8C,0x84,0x00,0x00,0x00,0x00},
{0x01,0x01,0x80,0x42,0x22,0x1A,0x07,0x02,0x42,0x82,0x42,0x3E,0x01,0x01,0x01,0x00},/*"务",5*/

{0x00,0x04,0xE4,0x24,0x2C,0xB4,0x25,0x26,0x24,0xB4,0x2C,0x24,0xE4,0x04,0x00,0x00},
{0x00,0x00,0xFF,0x02,0x01,0x1E,0x12,0x12,0x12,0x1E,0x41,0x82,0x7F,0x00,0x00,0x00},/*"商",6*/
};

const unsigned char BMP1[] =
{0x00,0x06,0x0A,0xFE,0x0A,0xC6,0x00,0xE0,0x00,0xF0,0x00,0xF8,0x00,0x00,0x00,0x00,
0x00,0x00,0xFE,0x7D,0xBB,0xC7,0xEF,0xEF,0xEF,0xEF,0xEF,0xEF,0xEF,0xC7,0xBB,0x7D,
0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,
0x0C,0xFE,0xFE,0x0C,0x08,0x20,0x60,0xFE,0xFE,0x60,0x20,0x00,0x00,0x00,0x78,0x48,
0xFE,0x82,0xBA,0xBA,0x82,0xBA,0xBA,0x82,0xBA,0xBA,0x82,0xBA,0xBA,0x82,0xFE,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xFF,
0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0x00,0x00,0xFE,0xFF,0x03,
0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0xFF,0xFE,0x00,0x00,0x00,0x00,0xC0,0xC0,
0xC0,0x00,0x00,0x00,0x00,0xFE,0xFF,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
0xFF,0xFE,0x00,0x00,0xFE,0xFF,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0xFF,
0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0xFF,0xFF,0x0C,
0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0xFF,0xFF,0x00,0x00,0x00,0x00,0xE1,0xE1,
0xE1,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xFF,0xFF,0x00,0x00,0xFF,0xFF,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0xFF,
0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x1F,
0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x1F,0x0F,0x00,0x00,0x0F,0x1F,0x18,
0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x1F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x0F,0x1F,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
0x1F,0x0F,0x00,0x00,0x0F,0x1F,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x1F,
0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x8C,0x42,0x22,0x12,0x0C,0x00,0xFC,0x02,0x02,0x02,0xFC,0x00,0x00,0x04,
0xFE,0x00,0x00,0x00,0x20,0x58,0x44,0xFE,0x40,0x00,0x10,0x10,0x10,0x10,0x10,0x00,
0x00,0x04,0xFE,0x00,0x00,0x00,0xFC,0x02,0x02,0x02,0xFC,0x00,0x10,0x10,0x10,0x10,
0x10,0x00,0xFC,0x02,0x02,0x02,0xFC,0x00,0x00,0x04,0xFE,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x24,0xA4,0x2E,0x24,0xE4,0x24,0x2E,0xA4,0x24,0x00,0x00,0x00,0xF8,0x4A,0x4C,
0x48,0xF8,0x48,0x4C,0x4A,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,
0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x20,0x10,0x10,
0x10,0x10,0x20,0xC0,0x00,0x00,0xC0,0x20,0x10,0x10,0x10,0x10,0x20,0xC0,0x00,0x00,
0x00,0x12,0x0A,0x07,0x02,0x7F,0x02,0x07,0x0A,0x12,0x00,0x00,0x00,0x0B,0x0A,0x0A,
0x0A,0x7F,0x0A,0x0A,0x0A,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x40,0x40,
0x40,0x50,0x20,0x5F,0x80,0x00,0x1F,0x20,0x40,0x40,0x40,0x50,0x20,0x5F,0x80,0x00,/*"C:\Users\evk\Desktop\??2014.bmp",0*/

};

const unsigned char hzk12us_font_data[][12] = {
    {0x12, 0x00, 0x12, 0x20, 0x27, 0xf0, 0x25, 0x00, 0x69, 0x00, 0xb1, 0xf0}, //  作
    {0x21, 0x00, 0x21, 0x00, 0x21, 0xf0, 0x21, 0x00, 0x21, 0x00, 0x21, 0x00},

    {0x04, 0x20, 0x3f, 0x40, 0x04, 0x80, 0x05, 0x20, 0xff, 0xf0, 0x04, 0x00}, //  者
    {0x1f, 0xc0, 0x30, 0x40, 0xdf, 0xc0, 0x10, 0x40, 0x1f, 0xc0, 0x10, 0x40},

    {0x12, 0x00, 0xfa, 0x20, 0x12, 0x40, 0x72, 0x80, 0x43, 0x20, 0x4f, 0xf0}, //  张
    {0x73, 0x00, 0x12, 0x80, 0x12, 0x40, 0x12, 0xa0, 0x93, 0x10, 0x62, 0x00},

    {0x21, 0x00, 0x20, 0x80, 0xff, 0xf0, 0x24, 0x40, 0x22, 0x80, 0x6f, 0xf0}, //  梓
    {0xb1, 0x00, 0xa1, 0x00, 0x27, 0xe0, 0x21, 0x00, 0x21, 0x00, 0x21, 0x00},

    {0x04, 0x20, 0xff, 0xf0, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x3f, 0xc0}, //  言
    {0x00, 0x00, 0x3f, 0xc0, 0x20, 0x40, 0x20, 0x40, 0x3f, 0xc0, 0x20, 0x40}
};