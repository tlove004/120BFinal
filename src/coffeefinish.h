#include <avr/pgmspace.h>

const unsigned char finishPic [] PROGMEM= {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0,
	0x10, 0x18, 0x18, 0x10, 0x30, 0x30, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00,
	0x00, 0x80, 0xC0, 0x40, 0xC0, 0xC0, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0x40, 0xC0, 0xC0, 0x00, 0x80,
	0xC0, 0xC0, 0x40, 0xC0, 0xC0, 0x00, 0x00, 0x80, 0xC0, 0x40, 0x40, 0xC0, 0xC0, 0x00, 0x00, 0xC0,
	0xC0, 0x40, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0xFE, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x30, 0x31, 0x21, 0x23, 0x23, 0x33, 0x3E, 0x0C, 0x00, 0x0F, 0x3F, 0x20, 0x20, 0x20,
	0x37, 0x3F, 0x00, 0x00, 0x1F, 0x3F, 0x30, 0x20, 0x20, 0x30, 0x00, 0x8F, 0x9F, 0xB0, 0xA0, 0xA0,
	0x30, 0x30, 0x00, 0x1F, 0x32, 0x22, 0x22, 0x22, 0x33, 0x33, 0x00, 0x33, 0x23, 0x26, 0x26, 0x3C,
	0x1C, 0x00, 0x31, 0x33, 0x26, 0x26, 0x24, 0x3C, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE,
	0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFF, 0xFF,
	0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF0, 0xFC, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xFC, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0,
	0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x1F,
	0x1F, 0x1F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x0F, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xEF, 0x83, 0x83,
	0x83, 0x83, 0xC7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0x83, 0x83, 0x83, 0x83, 0xEF, 0xFF, 0xFF,
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x0F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x18, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x03, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x7F, 0x3F, 0x3F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0x3F, 0x3F,
	0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};