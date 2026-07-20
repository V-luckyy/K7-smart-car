#ifndef __BSP_RGBLIGHT_H
#define __BSP_RGBLIGHT_H

#include <stdint.h>

typedef struct {
	void (*init)(void);                                            //灯带初始化
	void (*turnoff)(void);                                         //关闭灯带
    void (*SetColor)(uint8_t r,uint8_t g,uint8_t b);               //直接设置灯带颜色
    void (*SetColorFade)(uint8_t r,uint8_t g,uint8_t b);           //渐变设置灯带颜色
	void (*SetBreath)(uint8_t r,uint8_t g,uint8_t b);              //指定一个颜色让灯带进行呼吸
	void (*ShowError)(uint8_t times);                              //让灯带闪烁报错,入口参数为闪烁次数
	void (*RainbowMode)(void);                                     //彩虹灯带
	void (*SetBlink)(uint8_t r,uint8_t g,uint8_t b,uint32_t times);//设置灯带闪烁,times为闪烁延迟时间,单位ms
}RGBLightInterface_t,*pRGBLightInterface_t;

extern RGBLightInterface_t UserRGBLight;

#endif

