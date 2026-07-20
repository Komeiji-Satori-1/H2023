#include "AD9833.h"

/* =========================================================================
 * 模块内部常量
 * ========================================================================= */
#define MCP41010_WRITE_POT0 0x1100U

/* =========================================================================
 * 全局变量
 * ========================================================================= */
WaveFormConfig_t WaveFormConfig;

/* =========================================================================
 * 模块内部变量
 * ========================================================================= */
static uint16_t ad9833_control_word = (ad9833_Reg_control_B28 | ad9833_Sine);

/* =========================================================================
 * 内部辅助函数
 * ========================================================================= */
static void AD9833_SPI_Delay(void)
{
    __NOP();
    __NOP();
}

static void AD9833_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = AD9833_CS1_Pin |
                          AD9833_CS2_Pin |
                          AD9833_SCLK_Pin |
                          AD9833_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static uint16_t ad9833_freq_reg(uint8_t ch)
{
    return (ch == ad9833_CH1) ? ad9833_Reg_freq1 : ad9833_Reg_freq0;
}

static uint16_t ad9833_fselect_bit(uint8_t ch)
{
    return (ch == ad9833_CH1) ? ad9833_Reg_control_FSELECT : 0;
}

/* =========================================================================
 * SPI底层（原 My_SPI/My_SPI.c）
 * ========================================================================= */
void AD9833_SPI_16bits_Write(uint16_t data)
{
    uint8_t i;

    AD9833_SPI_SCK_H;

    for (i = 0; i < 16; i++)
    {
        if ((data & 0x8000U) != 0U)
        {
            AD9833_SPI_SDA_H;
        }
        else
        {
            AD9833_SPI_SDA_L;
        }

        AD9833_SPI_Delay();
        AD9833_SPI_SCK_L;
        AD9833_SPI_Delay();
        AD9833_SPI_SCK_H;

        data <<= 1;
    }
}

/* =========================================================================
 * AD9833驱动层（原 My_Driver/AD9833.c）
 * ========================================================================= */

void ad9833_init(void)
{
    AD9833_GPIO_Init();

    AD9833_SPI_CS1_H;
    AD9833_SPI_CS2_H;

    AD9833_SPI_SCK_H;

    HAL_Delay(10);

    uint32_t freq_word = ad9833_freq_to_word(ad9833_Freq);
    uint16_t fre_l = (uint16_t)(ad9833_Reg_freq0 | (freq_word & AD9833_FREQ_DATA_MASK));
    uint16_t fre_h = (uint16_t)(ad9833_Reg_freq0 | ((freq_word >> AD9833_FREQ_DATA_BITS) & AD9833_FREQ_DATA_MASK));

    ad9833_control_word = ad9833_Reg_control_B28 | ad9833_Sine;

    /* 两片同步操作：复位 -> 同步写频率 -> 同时释放复位
     * 要求两片共用同一 MCLK，方可保证相位对齐               */
    ad9833_write_reg(ad9833_control_word | ad9833_Reg_control_Reset, AD9833_ALL);
    ad9833_write_reg(fre_l, AD9833_ALL);
    ad9833_write_reg(fre_h, AD9833_ALL);
    ad9833_write_reg(ad9833_control_word, AD9833_ALL);  /* 同时释放复位，相位同步 */
}

void ad9833_write_reg(uint16_t value, uint16_t ch)
{
    if(ch == AD9833_ALL)
	{
		AD9833_SPI_CS1_L;
		AD9833_SPI_CS2_L;
		AD9833_SPI_16bits_Write(value);
		AD9833_SPI_CS1_H;
		AD9833_SPI_CS2_H;
	}
	else if(ch == AD9833_CH1)
	{
		AD9833_SPI_CS1_L;
		AD9833_SPI_16bits_Write(value);
		AD9833_SPI_CS1_H;
	}
	else if(ch == AD9833_CH2)
	{
		AD9833_SPI_CS2_L;
		AD9833_SPI_16bits_Write(value);
		AD9833_SPI_CS2_H;
	}
}

uint32_t ad9833_freq_to_word(uint32_t freq_hz)
{
    uint64_t word = ((uint64_t)freq_hz * AD9833_FREQ_WORD_SCALE + (AD9833_MCLK_HZ / 2U)) / AD9833_MCLK_HZ;

    if (word > AD9833_FREQ_WORD_MAX)
    {
        word = AD9833_FREQ_WORD_MAX;
    }

    return (uint32_t)word;
}

void ad9833_set_waveform(uint16_t type)
{
    ad9833_control_word &= (uint16_t)(ad9833_Reg_control_B28 | ad9833_Reg_control_FSELECT | ad9833_Reg_control_PSELECT);
    ad9833_control_word |= type;
    ad9833_write_reg(ad9833_control_word, AD9833_ALL);
}

void ad9833_set_freq(uint32_t freq, uint16_t type)
{
    ad9833_set_freq_ch(freq, type, ad9833_CH0);
}

void ad9833_set_freq_ch(uint32_t freq, uint16_t type, uint8_t ch)
{
    ad9833_set_freq_word(ad9833_freq_to_word(freq), type, ch);
}

void ad9833_set_freq_word(uint32_t freq_word, uint16_t type, uint8_t ch)
{
    uint16_t freq_reg = ad9833_freq_reg(ch);
    uint16_t fselect  = ad9833_fselect_bit(ch);

    freq_word &= AD9833_FREQ_WORD_MAX;

    uint16_t fre_l = (uint16_t)(freq_word & AD9833_FREQ_DATA_MASK);
    uint16_t fre_h = (uint16_t)((freq_word >> AD9833_FREQ_DATA_BITS) & AD9833_FREQ_DATA_MASK);

    ad9833_control_word = ad9833_Reg_control_B28 | fselect | type;

    ad9833_write_reg(ad9833_control_word | ad9833_Reg_control_Reset, ch);
    ad9833_write_reg(freq_reg | fre_l, ch);
    ad9833_write_reg(freq_reg | fre_h, ch);
    ad9833_write_reg(ad9833_control_word, ch);
}

void ad9833_set_freq_ch_live(uint32_t freq, uint16_t type, uint8_t ch)
{
    ad9833_set_freq_word_live(ad9833_freq_to_word(freq), type, ch);
}

void ad9833_set_freq_word_live(uint32_t freq_word, uint16_t type, uint8_t ch)
{
    uint16_t fre_l;
    uint16_t fre_h;
    uint16_t ctrl;

    freq_word &= AD9833_FREQ_WORD_MAX;

    fre_l = (uint16_t)(ad9833_Reg_freq0 |
                       (freq_word & AD9833_FREQ_DATA_MASK));
    fre_h = (uint16_t)(ad9833_Reg_freq0 |
                       ((freq_word >> AD9833_FREQ_DATA_BITS) &
                        AD9833_FREQ_DATA_MASK));
    ctrl = (uint16_t)(ad9833_Reg_control_B28 | type);

    ad9833_write_reg(ctrl, ch);
    ad9833_write_reg(fre_l, ch);
    ad9833_write_reg(fre_h, ch);
}

void ad9833_sweep_start(ad9833_sweep_t *sweep, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms, uint16_t type)
{
    ad9833_sweep_start_ch(sweep, start_hz, stop_hz, step_hz, dwell_ms, type, ad9833_CH0);
}

void ad9833_sweep_start_ch(ad9833_sweep_t *sweep, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms, uint16_t type, uint8_t ch)
{
    if (sweep == 0)
    {
        return;
    }

    if (step_hz == 0U)
    {
        step_hz = 1U;
    }

    sweep->start_hz   = start_hz;
    sweep->stop_hz    = stop_hz;
    sweep->step_hz    = step_hz;
    sweep->current_hz = start_hz;
    sweep->dwell_ms   = dwell_ms;
    sweep->last_tick  = HAL_GetTick();
    sweep->type       = type;
    sweep->ch         = ch;
    sweep->enable     = 1U;

    ad9833_set_freq_ch(start_hz, type, ch);
}

uint8_t ad9833_sweep_process(ad9833_sweep_t *sweep)
{
    uint32_t now;
    uint32_t next_hz;

    if ((sweep == 0) || (sweep->enable == 0U))
    {
        return 0U;
    }

    now = HAL_GetTick();
    if ((now - sweep->last_tick) < sweep->dwell_ms)
    {
        return 0U;
    }

    sweep->last_tick = now;

    if (sweep->start_hz <= sweep->stop_hz)
    {
        next_hz = sweep->current_hz + sweep->step_hz;
        if (next_hz > sweep->stop_hz)
        {
            next_hz = sweep->start_hz;
        }
    }
    else
    {
        if (sweep->current_hz <= (sweep->stop_hz + sweep->step_hz))
        {
            next_hz = sweep->start_hz;
        }
        else
        {
            next_hz = sweep->current_hz - sweep->step_hz;
        }
    }

    sweep->current_hz = next_hz;
    ad9833_set_freq_ch(next_hz, sweep->type, sweep->ch);

    return 1U;
}

void ad9833_sweep_stop(ad9833_sweep_t *sweep)
{
    if (sweep != 0)
    {
        sweep->enable = 0U;
    }
}

/* =========================================================================
 * 两通道不同频率同步启动（初始相位均为 0）
 * 要求两片共用同一 MCLK
 * freq1_hz : 芯片1频率（Hz）
 * type1    : 芯片1波形类型
 * freq2_hz : 芯片2频率（Hz）
 * type2    : 芯片2波形类型
 * ========================================================================= */
void ad9833_sync_start(uint32_t freq1_hz, uint16_t type1, uint32_t freq2_hz, uint16_t type2)
{
    uint32_t fw1 = ad9833_freq_to_word(freq1_hz);
    uint32_t fw2 = ad9833_freq_to_word(freq2_hz);

    uint16_t ctrl1 = (uint16_t)(ad9833_Reg_control_B28 | type1);
    uint16_t ctrl2 = (uint16_t)(ad9833_Reg_control_B28 | type2);

    /* 1. 两片同时进入复位，相位累加器锁定在 0 */
    ad9833_write_reg(ad9833_Reg_control_B28 | ad9833_Reg_control_Reset, AD9833_ALL);

    /* 2. 分别写入不同频率字（复位期间不影响输出） */
    ad9833_write_reg((uint16_t)(ad9833_Reg_freq0 | (fw1 & AD9833_FREQ_DATA_MASK)), AD9833_CH1);
    ad9833_write_reg((uint16_t)(ad9833_Reg_freq0 | ((fw1 >> AD9833_FREQ_DATA_BITS) & AD9833_FREQ_DATA_MASK)), AD9833_CH1);
    ad9833_write_reg((uint16_t)(ad9833_Reg_freq0 | (fw2 & AD9833_FREQ_DATA_MASK)), AD9833_CH2);
    ad9833_write_reg((uint16_t)(ad9833_Reg_freq0 | ((fw2 >> AD9833_FREQ_DATA_BITS) & AD9833_FREQ_DATA_MASK)), AD9833_CH2);

    /* 3. 分别写入正确的控制字（保持复位） */
    ad9833_write_reg(ctrl1 | ad9833_Reg_control_Reset, AD9833_CH1);
    ad9833_write_reg(ctrl2 | ad9833_Reg_control_Reset, AD9833_CH2);

    /* 4. 释放复位：波形相同时同时释放（最佳同步）；不同时逐片释放 */
    if (type1 == type2)
    {
        ad9833_write_reg(ctrl1, AD9833_ALL);
    }
    else
    {
        ad9833_write_reg(ctrl1, AD9833_CH1);
        ad9833_write_reg(ctrl2, AD9833_CH2);
    }

    ad9833_control_word = ctrl1;
}

/* =========================================================================
 * 仅设置两片相位寄存器，不动频率、不复位、不中断输出
 * phase1_deg : 芯片1相位（0~360，单位度）
 * phase2_deg : 芯片2相位（0~360，单位度）
 * ========================================================================= */
void ad9833_write_phase(float phase1_deg, float phase2_deg)
{
    uint16_t p1 = (phase1_deg <= 0.0f || phase1_deg >= 360.0f)
                  ? 0U
                  : (uint16_t)(phase1_deg / 360.0f * 4096.0f);
    uint16_t p2 = (phase2_deg <= 0.0f || phase2_deg >= 360.0f)
                  ? 0U
                  : (uint16_t)(phase2_deg / 360.0f * 4096.0f);

    ad9833_write_reg((uint16_t)(ad9833_Reg_phase0 | (p1 & 0x0FFFU)), AD9833_CH1);
    ad9833_write_reg((uint16_t)(ad9833_Reg_phase0 | (p2 & 0x0FFFU)), AD9833_CH2);
}
void waveset(uint32_t Freq, uint16_t type, uint16_t ch)
{
    ad9833_set_freq_ch(Freq, type, (uint8_t)ch);
}

/* =========================================================================
 * APL应用层（原 My_APL/dds_apl.c）
 * ========================================================================= */
void dds_process(void)
{
    WaveFormConfig.ch   = ad9833_CH0;
    WaveFormConfig.freq = 1000;
    WaveFormConfig.type = ad9833_Sine;
    waveset(WaveFormConfig.freq, WaveFormConfig.type, WaveFormConfig.ch);
}
