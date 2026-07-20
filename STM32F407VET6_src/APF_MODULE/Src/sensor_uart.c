#include "system.h"
#include "sensor_uart.h"

/* ====================== 全局变量 ====================== */
volatile float  g_sensor_dist_front = 4.0f;   /* 默认超量程 4m */
volatile float  g_sensor_dist_left  = 4.0f;
volatile float  g_sensor_dist_right = 4.0f;
volatile uint8_t g_sensor_fresh = 0;

/* ====================== 协议解析 ====================== */
/*
 * TFmini 格式示例 (9字节帧):
 *   0x59 0x59 DistL DistH ... Checksum
 *   dist = DistL + DistH*256
 *
 * [TODO: 根据实际传感器型号修改帧解析逻辑]
 */
static float parse_sensor_frame(uint8_t *buf)
{
    return (float)buf[0];
}

/* ====================== 初始化 (STM32F4) ====================== */

void sensor_uart_init(uint32_t baud)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* ---- USART2: 前方传感器 (PA2=TX, PA3=RX) ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    /* ---- USART3: 左侧传感器 (PC10=TX, PC11=RX) ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_Init(USART3, &USART_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    /* ---- UART5: 右侧传感器 (PC12=TX, PD2=RX) ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_Init(UART5, &USART_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE);
}

/* ====================== 中断服务函数 ====================== */

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        static uint8_t buf[16], idx = 0;
        uint8_t byte = USART_ReceiveData(USART2);
        buf[idx++] = byte;
        if(idx >= 9) {
            g_sensor_dist_front = parse_sensor_frame(buf);
            idx = 0;
        }
    }
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        static uint8_t buf[16], idx = 0;
        uint8_t byte = USART_ReceiveData(USART3);
        buf[idx++] = byte;
        if(idx >= 9) {
            g_sensor_dist_left = parse_sensor_frame(buf);
            idx = 0;
        }
    }
}

void UART5_IRQHandler(void)
{
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        static uint8_t buf[16], idx = 0;
        uint8_t byte = USART_ReceiveData(UART5);
        buf[idx++] = byte;
        if(idx >= 9) {
            g_sensor_dist_right = parse_sensor_frame(buf);
            g_sensor_fresh = 1;
            idx = 0;
        }
    }
}
