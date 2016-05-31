/*
 * Nokia 3110 / 5110 unbuffered driver
 * (C)2013 Radu Motisan (C++ version)
 * www.pocketmagic.net
 * Based on a work by Tony Myatt - 2007
 * Updated to C code by Tyson Loveless - 2016
 */

#pragma once

// define  LCD screen size 
#define LCD_WIDTH 84
#define LCD_HEIGHT 48
#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8

#define controlPort &PORTD
#define dataPort &DDRD
#define SCE PORTD6
//#define RST PORTD0
#define DC PORTD4
#define DATA PORTD3
#define CLK PORTD5

typedef enum { LCD_CMD  = 0, LCD_DATA = 1 } LcdCmdData;

unsigned char  m_dq_SCE, m_dq_RST, m_dq_DC, m_dq_DATA, m_dq_CLK;
volatile unsigned char  *m_port_SCE, *m_port_RST, *m_port_DC, *m_port_DATA, *m_port_CLK;

volatile unsigned char* Port2DDR(volatile unsigned char *port) {
	return port - 1;
}

int lcdCacheIdx;

void lcd_init();
void lcd_contrast(unsigned char contrast);
void lcd_clear(void);
void lcd_clear_area(unsigned char line, unsigned char startX, unsigned char endX);
void lcd_clear_line(unsigned char line);
void lcd_goto_xy(unsigned char x, unsigned char y);
void lcd_goto_xy_exact(unsigned char x, unsigned char y);
void lcd_chr(char chr);
void lcd_str(char* str);
void lcd_string_format(char *szFormat, ...);
void lcd_send(unsigned char data, LcdCmdData cd);
void lcd_base_addr(unsigned int addr) ;
void lcd_col(char chr);
void lcd_pixelBack(void);
void printPictureOnLCD ( const unsigned char *data);
void drawPixel(unsigned char  x, unsigned char  y, int color);
void delay_ms(int miliSec);


