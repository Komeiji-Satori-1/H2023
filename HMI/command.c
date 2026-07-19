#include "command.h"
#include "state.h"

#define HMI_FRAME_HEAD_A1 0xA1
#define HMI_FRAME_HEAD_A2 0xA2
#define HMI_FRAME_HEAD_A3 0xA3
#define HMI_FRAME_HEAD_A4 0xA4
#define HMI_FRAME_HEAD_A5 0xA5
#define HMI_FRAME_HEAD_A6 0xA6
#define HMI_FRAME_HEAD_A7 0xA7

#define HMI_FRAME_LEN 5

#define HMI_RING_SIZE 128
#define HMI_RX_CHUNK_SIZE 32

static uint8_t hmi_ring_buf[HMI_RING_SIZE];
static uint8_t hmi_read_index = 0;
static uint8_t hmi_write_index = 0;

static uint8_t hmi_rx_chunk[HMI_RX_CHUNK_SIZE];

// 包头数据和标志位
volatile uint8_t hmi_a1_update_flag = 0;
volatile uint8_t hmi_a2_update_flag = 0;
volatile uint8_t hmi_a3_update_flag = 0;
volatile uint8_t hmi_a4_update_flag = 0;
volatile uint8_t hmi_a5_update_flag = 0;
volatile uint8_t hmi_a6_update_flag = 0;
volatile uint8_t hmi_a7_update_flag = 0;

volatile uint32_t hmi_a1_value = 0;
volatile uint32_t hmi_a2_value = 0;
volatile uint32_t hmi_a3_value = 0;
volatile uint32_t hmi_a4_value = 0;
volatile uint32_t hmi_a5_value = 0;
volatile uint32_t hmi_a6_value = 0;
volatile uint32_t hmi_a7_value = 0;

static uint8_t HMI_IsFrameHead(uint8_t data)
{
    switch (data)
    {
    case HMI_FRAME_HEAD_A1:
    case HMI_FRAME_HEAD_A2:
    case HMI_FRAME_HEAD_A3:
    case HMI_FRAME_HEAD_A4:
    case HMI_FRAME_HEAD_A5:
    case HMI_FRAME_HEAD_A6:
    case HMI_FRAME_HEAD_A7:
        return 1;

    default:
        return 0;
    }
}

void Usart_Send_Computer(UART_HandleTypeDef huart, char *msg)
{
    HAL_UART_Transmit(&huart, (uint8_t *)msg, strlen(msg), 1000);
}

// 增加循环缓冲区索引
static void HMI_AddReadIndex(uint8_t length)
{
    hmi_read_index += length;
    hmi_read_index %= HMI_RING_SIZE;
}

// 从循环缓冲区读取指定位置的数据
static uint8_t HMI_Read(uint8_t index)
{
    return hmi_ring_buf[index % HMI_RING_SIZE];
}

// 获取循环缓冲区中未处理的数据长度
static uint8_t HMI_GetLength(void)
{
    return (hmi_write_index + HMI_RING_SIZE - hmi_read_index) % HMI_RING_SIZE;
}

// 获取循环缓冲区剩余空间
static uint8_t HMI_GetRemain(void)
{
    return (HMI_RING_SIZE - 1U) - HMI_GetLength();
}

// 将 ReceiveToIdle 收到的一段数据写入循环缓冲区
static uint8_t HMI_Write(uint8_t *data, uint8_t length)
{
    if (HMI_GetRemain() < length)
    {
        return 0;
    }

    if ((hmi_write_index + length) < HMI_RING_SIZE)
    {
        memcpy(&hmi_ring_buf[hmi_write_index], data, length);
        hmi_write_index += length;
    }
    else
    {
        uint8_t first_len = HMI_RING_SIZE - hmi_write_index;

        memcpy(&hmi_ring_buf[hmi_write_index], data, first_len);
        memcpy(hmi_ring_buf, data + first_len, length - first_len);

        hmi_write_index = length - first_len;
    }

    hmi_write_index %= HMI_RING_SIZE;
    return length;
}

static uint32_t HMI_GetFrameValue(uint8_t *frame)
{
    return ((uint32_t)frame[1]) | ((uint32_t)frame[2] << 8) | ((uint32_t)frame[3] << 16) | ((uint32_t)frame[4] << 24);
}

static uint8_t HMI_GetFrame(uint8_t *frame)
{
    uint8_t length;

    while (1)
    {
        length = HMI_GetLength();

        if (length < HMI_FRAME_LEN)
        {
            return 0;
        }

        if (!HMI_IsFrameHead(HMI_Read(hmi_read_index)))
        {
            HMI_AddReadIndex(1);
            continue;
        }

        for (uint8_t i = 0; i < HMI_FRAME_LEN; i++)
        {
            frame[i] = HMI_Read(hmi_read_index + i);
        }

        HMI_AddReadIndex(HMI_FRAME_LEN);
        return HMI_FRAME_LEN;
    }
}

static uint8_t HMI_ParseFrame(uint8_t *frame, uint8_t frame_len, uint8_t *head, uint32_t *value)
{
    if (frame_len != HMI_FRAME_LEN)
    {
        return 0;
    }

    if (!HMI_IsFrameHead(frame[0]))
    {
        return 0;
    }

    *head = frame[0];

    *value = HMI_GetFrameValue(frame);

    return 1;
}

void My_Usart_Init(void)
{
    memset(hmi_ring_buf, 0, sizeof(hmi_ring_buf));
    memset(hmi_rx_chunk, 0, sizeof(hmi_rx_chunk));

    hmi_read_index = 0;
    hmi_write_index = 0;

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, hmi_rx_chunk, sizeof(hmi_rx_chunk));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        HMI_Write(hmi_rx_chunk, (uint8_t)Size);

        memset(hmi_rx_chunk, 0, sizeof(hmi_rx_chunk));

        HAL_UARTEx_ReceiveToIdle_IT(&huart1, hmi_rx_chunk, sizeof(hmi_rx_chunk));
    }
}

void Usart_Rx_Proc(void)
{
    uint8_t frame[HMI_FRAME_LEN];
    uint8_t frame_len;
    uint32_t value;

    while ((frame_len = HMI_GetFrame(frame)) != 0)
    {
        if (frame_len != HMI_FRAME_LEN)
        {
            continue;
        }

        value = HMI_GetFrameValue(frame);

        switch (frame[0])
        {
        case HMI_FRAME_HEAD_A1:
            hmi_a1_value = value;
            hmi_a1_update_flag = 1;
            break;

        case HMI_FRAME_HEAD_A2:
            hmi_a2_value = value;
            hmi_a2_update_flag = 1;
            break;

        case HMI_FRAME_HEAD_A3:
            hmi_a3_value = value;
            hmi_a3_update_flag = 1;
            break;

        case HMI_FRAME_HEAD_A4:
            hmi_a4_value = value;
            hmi_a4_update_flag = 1;
            break;

        case HMI_FRAME_HEAD_A5:
            hmi_a5_update_flag = 1;
            break;

        case HMI_FRAME_HEAD_A6:
            hmi_a6_update_flag = 1;
            break;

        case HMI_FRAME_HEAD_A7:
            hmi_a7_value = value;
            hmi_a7_update_flag = 1;
            break;

        default:
            break;
        }
    }
}
