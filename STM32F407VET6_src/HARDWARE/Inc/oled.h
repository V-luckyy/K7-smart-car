#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"

#define OLED_CMD  0	//Command //Ð´ÃüÁî
#define OLED_DATA 1	//Data //Ð´Êý¾Ý

#define CNSizeWidth  16
#define CNSizeHeight 16

/*--------OLED config--------*/
#define OLED_SCLK_Clr()  OLED_PinWrite(&oled_pins->scl, Bit_RESET)   //SCL
#define OLED_SCLK_Set()  OLED_PinWrite(&oled_pins->scl, Bit_SET)     //SCL

#define OLED_SDIN_Clr()  OLED_PinWrite(&oled_pins->sda, Bit_RESET)   //SDA
#define OLED_SDIN_Set()  OLED_PinWrite(&oled_pins->sda, Bit_SET)     //SDA

#define OLED_RST_Clr()   OLED_PinWrite(&oled_pins->rst, Bit_RESET)   //RES
#define OLED_RST_Set()   OLED_PinWrite(&oled_pins->rst, Bit_SET)     //RES

#define OLED_RS_Clr()    OLED_PinWrite(&oled_pins->dc,  Bit_RESET)   //DC
#define OLED_RS_Set()    OLED_PinWrite(&oled_pins->dc,  Bit_SET)     //DC
/*----------------------------------*/



/*--------OLED Interface Fun--------*/
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh_Gram(void);		
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);
void oled_showfloat(const float needtoshow,u8 show_x,u8 show_y,u8 zs_num,u8 xs_num);
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 font_width,u8 font_height);	\
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,const unsigned char BMP[]);
extern const unsigned char gImage_usb_bmp[];
void OLED_Refresh_Line(void);
void OLED_ClearBuf(void);
/*----------------------------------*/


#endif  
	 
