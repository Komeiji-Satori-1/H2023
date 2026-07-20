#include "dac_nco.h"
#include "dac.h"
#include <math.h>
#include <string.h>

#define DAC_NCO_BUF_LEN 2048U
#define DAC_NCO_HALF_LEN (DAC_NCO_BUF_LEN / 2U)
#define DAC_NCO_TABLE_LEN 1024U
#define DAC_NCO_TABLE_BITS 10U
#define DAC_NCO_PHASE_BITS 32U
#define DAC_NCO_DAC_FULL_SCALE 4095.0f
#define DAC_NCO_DAC_MID 2048.0f
#define DAC_NCO_DEFAULT_FS_HZ 1025641.0f
#define DAC_NCO_PI 3.14159265358979323846f
#define DAC_NCO_TWO_PI (2.0f * DAC_NCO_PI)
#define DAC_NCO_ADC_TO_DAC_SCALE (4095.0f / 65535.0f)
#define DAC_NCO_MIN_DAC_AMP 1.0f
#define DAC_NCO_MAX_DAC_AMP 2047.0f

#define DAC_NCO_PLL_FRAME_LEN 1024U
#define DAC_NCO_PLL_PHASE_KP_SHIFT 1U
#define DAC_NCO_PLL_STEP_KP_DIV 20U
#define DAC_NCO_PLL_STEP_KI_DIV 200U
#define DAC_NCO_PLL_STEP_KD_DIV 0U
#define DAC_NCO_PLL_MAX_CORR_DIV 2000U
#define DAC_NCO_PLL_INTEGRATOR_LIMIT 8589934592LL
#define DAC_NCO_PLL_INTEGRATOR_LEAK_NUM 65535LL
#define DAC_NCO_PLL_PHASE_DEADBAND_Q32 3000000L

extern float fs;

typedef struct
{
    uint32_t nominal_inc;
    int32_t inc_corr;
    int64_t integrator;
    uint32_t phase_ref;
    uint64_t sample_ref;
    int32_t last_error;
} DacNcoPllState_t;

static SignalInfo s_sig[2];
static DacNcoPllState_t s_pll[2];
static uint8_t s_ready = 0;
static uint8_t s_running = 0;
static uint8_t s_table_ready = 0;
static uint16_t s_phase_deg = 0;
static uint32_t s_output_phase_offset_b = 0;
static int16_t s_sine_table[DAC_NCO_TABLE_LEN];
static int16_t s_triangle_table[DAC_NCO_TABLE_LEN];
static uint16_t dac_ch1_buf[DAC_NCO_BUF_LEN];
static uint16_t dac_ch2_buf[DAC_NCO_BUF_LEN];
static volatile uint8_t s_dac_ready_mask = 0;
static volatile uint64_t s_dac_sample_count = 0;
static volatile uint64_t s_dac_ready_play_sample[2] = {0};

/*
 * 获取 NCO 使用的采样率。
 * 优先使用 FFT 模块维护的全局采样率 fs；如果 fs 尚未初始化，
 * 则使用当前 TIM8 设置对应的默认实际采样率。
 */
static float DacNco_GetFs(void)
{
    if (fs > 1.0f)
    {
        return fs;
    }

    return DAC_NCO_DEFAULT_FS_HZ;
}

/*
 * 将角度制相位转换为 Q32 相位格式。
 * Q32 中 0x00000000 表示 0 度，0xFFFFFFFF 接近 360 度。
 */
static uint32_t DacNco_DegToPhaseQ32(uint16_t phase_deg)
{
    return (uint32_t)(((uint64_t)phase_deg << DAC_NCO_PHASE_BITS) / 360U);
}

/*
 * 将弧度制相位转换为 Q32 相位格式。
 * 输入相位会先归一化到 [0, 2*pi) 范围，再映射到 0 ~ 2^32。
 */
static uint32_t DacNco_RadToPhaseQ32(float phase_rad)
{
    double phase = (double)phase_rad;

    while (phase < 0.0)
    {
        phase += (double)DAC_NCO_TWO_PI;
    }

    while (phase >= (double)DAC_NCO_TWO_PI)
    {
        phase -= (double)DAC_NCO_TWO_PI;
    }

    return (uint32_t)((phase * 4294967296.0) / (double)DAC_NCO_TWO_PI);
}

/*
 * 根据目标输出频率计算 NCO 的 Q32 相位步进值。
 * phase_inc = freq / fs * 2^32。
 * 每输出一个 DAC 采样点，相位累加器增加一次该步进值。
 */
static uint32_t DacNco_FreqToInc(float freq_hz)
{
    double inc;
    double sample_rate = (double)DacNco_GetFs();

    if ((freq_hz <= 0.0f) || (sample_rate <= 1.0))
    {
        return 0U;
    }

    inc = ((double)freq_hz * 4294967296.0) / sample_rate;
    if (inc < 0.0)
    {
        return 0U;
    }

    if (inc > 4294967295.0)
    {
        return 0xFFFFFFFFU;
    }

    return (uint32_t)(inc + 0.5);
}

/*
 * 构建 NCO 使用的正弦波和三角波查找表。
 * 查找表保存一个完整周期，格式为有符号 Q15，范围约为 -32767 ~ +32767。
 * 该函数只在第一次调用时真正生成表，避免实时输出时反复计算 sinf()。
 */
static void DacNco_BuildTables(void)
{
    uint32_t i;
    float angle;
    float phase;
    float sine_val;
    float triangle_val;

    if (s_table_ready)
    {
        return;
    }

    for (i = 0; i < DAC_NCO_TABLE_LEN; i++)
    {
        angle = (2.0f * DAC_NCO_PI * (float)i) / (float)DAC_NCO_TABLE_LEN;
        sine_val = sinf(angle);
        s_sine_table[i] = (int16_t)(sine_val * 32767.0f);

        phase = (float)i / (float)DAC_NCO_TABLE_LEN;
        if (phase < 0.25f)
        {
            triangle_val = 4.0f * phase;
        }
        else if (phase < 0.75f)
        {
            triangle_val = 2.0f - 4.0f * phase;
        }
        else
        {
            triangle_val = -4.0f + 4.0f * phase;
        }
        //    triangle_val = 1.0f - (4.0f * fabsf(phase - 0.5f));
        s_triangle_table[i] = (int16_t)(triangle_val * 32767.0f);
    }

    s_table_ready = 1;
}

/*
 * 获取指定通道当前实际使用的相位步进值。
 * 实际步进 = 标称步进 nominal_inc + PLL 修正量 inc_corr。
 */
static uint32_t DacNco_Step(uint8_t ch)
{
    if (ch >= 2U)
    {
        return 0U;
    }

    return s_pll[ch].nominal_inc + (uint32_t)s_pll[ch].inc_corr;
}

/*
 * 计算指定通道在某个绝对采样编号处的预测相位。
 * phase_ref 是 sample_ref 时刻的相位锚点，
 * sample 与 sample_ref 的差值乘以当前 phase_inc 得到相位推进量。
 */
static uint32_t DacNco_PhaseAtSample(uint8_t ch, uint64_t sample)
{
    uint64_t delta = 0U;

    if (ch >= 2U)
    {
        return 0U;
    }

    if (sample >= s_pll[ch].sample_ref)
    {
        delta = sample - s_pll[ch].sample_ref;
    }

    return s_pll[ch].phase_ref + (uint32_t)((uint64_t)DacNco_Step(ch) * delta);
}

/*
 * 将识别得到的 ADC 幅值转换为 DAC 输出幅值码值。
 * 同时做最小/最大限幅，避免输出过小或超过 DAC 中点可摆幅。
 */
static float DacNco_GetAmplitudeCounts(const SignalInfo *sig)
{
    float amp;

    if (sig == NULL)
    {
        return 0.0f;
    }

    amp = fabsf(sig->amp) * DAC_NCO_ADC_TO_DAC_SCALE;

    if (amp < DAC_NCO_MIN_DAC_AMP)
    {
        amp = DAC_NCO_MIN_DAC_AMP;
    }

    if (amp > DAC_NCO_MAX_DAC_AMP)
    {
        amp = DAC_NCO_MAX_DAC_AMP;
    }

    return amp;
}

/*
 * 根据相位累加器查表得到当前波形采样值。
 * Q32 相位的高 DAC_NCO_TABLE_BITS 位作为查表索引；
 * 波形类型为三角波时查三角表，否则默认查正弦表。
 */
static int16_t DacNco_GetShapeSample(const SignalInfo *sig, uint32_t phase_acc)
{
    uint32_t index;

    if (sig == NULL)
    {
        return 0;
    }

    index = phase_acc >> (DAC_NCO_PHASE_BITS - DAC_NCO_TABLE_BITS);
    index &= (DAC_NCO_TABLE_LEN - 1U);

    if (sig->type == WAVE_TRIANGLE)
    {
        return s_triangle_table[index];
    }

    return s_sine_table[index];
}

/*
 * 生成一个 12 位 DAC 采样点。
 * 查表得到的 Q15 波形值先乘以输出幅值，再叠加 DAC 中点 2048，
 * 最后限制在 0 ~ 4095 范围内。
 */
static uint16_t DacNco_MakeSample(const SignalInfo *sig, uint32_t phase_acc)
{
    float amp_counts;
    float sample;
    int16_t shape;

    if (sig == NULL)
    {
        return (uint16_t)DAC_NCO_DAC_MID;
    }

    amp_counts = DacNco_GetAmplitudeCounts(sig);
    shape = DacNco_GetShapeSample(sig, phase_acc);
    sample = DAC_NCO_DAC_MID + (((float)shape * amp_counts) / 32767.0f);

    if (sample < 0.0f)
    {
        sample = 0.0f;
    }
    else if (sample > DAC_NCO_DAC_FULL_SCALE)
    {
        sample = DAC_NCO_DAC_FULL_SCALE;
    }

    return (uint16_t)(sample + 0.5f);
}

/*
 * 重填 DAC DMA 的一个半缓冲区。
 * half_index = 0 表示前半缓冲，half_index = 1 表示后半缓冲。
 * play_sample 表示该半缓冲第一个采样点对应的绝对播放采样编号，
 * 用它计算起始相位，保证输出相位能和 PLL 的时间轴对齐。
 */
static void DacNco_FillHalf(uint8_t half_index, uint64_t play_sample)
{
    uint16_t offset;
    uint16_t i;
    uint32_t phase_a;
    uint32_t phase_b;
    uint32_t inc_a;
    uint32_t inc_b;

    if ((!s_ready) || (half_index > 1U))
    {
        return;
    }

    offset = (uint16_t)((uint16_t)half_index * DAC_NCO_HALF_LEN);
    phase_a = DacNco_PhaseAtSample(0U, play_sample);
    phase_b = DacNco_PhaseAtSample(1U, play_sample) + s_output_phase_offset_b;
    inc_a = DacNco_Step(0U);
    inc_b = DacNco_Step(1U);

    for (i = 0; i < DAC_NCO_HALF_LEN; i++)
    {
        dac_ch1_buf[offset + i] = DacNco_MakeSample(&s_sig[0], phase_a);
        dac_ch2_buf[offset + i] = DacNco_MakeSample(&s_sig[1], phase_b);

        phase_a += inc_a;
        phase_b += inc_b;
    }
}

/*
 * 初始化指定 DAC 通道的 NCO/PLL 状态。
 * 根据识别出的频率计算 nominal_inc，根据单频 DFT 相位初始化 phase_ref。
 * inc_corr 和积分项清零，表示刚开始锁相时不做频率修正。
 */
static void DacNco_InitPll(uint8_t ch, const SignalInfo *sig)
{
    if ((ch >= 2U) || (sig == NULL))
    {
        return;
    }

    s_pll[ch].nominal_inc = DacNco_FreqToInc(sig->freq);
    s_pll[ch].inc_corr = 0;
    s_pll[ch].integrator = 0;
    s_pll[ch].phase_ref = DacNco_RadToPhaseQ32(sig->phase);
    s_pll[ch].sample_ref = 0U;
    s_pll[ch].last_error = 0;
}

static int32_t DacNco_MaxCorrection(uint8_t ch)
{
    int32_t max_corr;

    if (ch >= 2U)
    {
        return 0;
    }

    max_corr = (int32_t)(s_pll[ch].nominal_inc / DAC_NCO_PLL_MAX_CORR_DIV);
    if (max_corr < 1)
    {
        max_corr = 1;
    }

    return max_corr;
}

/*
 * 限制 PLL 对相位步进的修正量。
 * 修正量最大为 nominal_inc / DAC_NCO_PLL_MAX_CORR_DIV，
 * 防止异常相位测量导致 NCO 频率被一次性拉偏太多。
 */
static int32_t DacNco_LimitCorrection(uint8_t ch, int64_t correction)
{
    int32_t max_corr = DacNco_MaxCorrection(ch);

    if (correction > (int64_t)max_corr)
    {
        correction = max_corr;
    }
    else if (correction < -(int64_t)max_corr)
    {
        correction = -(int64_t)max_corr;
    }

    return (int32_t)correction;
}

static int32_t DacNco_ApplyPhaseDeadband(int32_t phase_error)
{
    if (phase_error > (int32_t)DAC_NCO_PLL_PHASE_DEADBAND_Q32)
    {
        return phase_error - (int32_t)DAC_NCO_PLL_PHASE_DEADBAND_Q32;
    }

    if (phase_error < -(int32_t)DAC_NCO_PLL_PHASE_DEADBAND_Q32)
    {
        return phase_error + (int32_t)DAC_NCO_PLL_PHASE_DEADBAND_Q32;
    }

    return 0;
}

static int64_t DacNco_LimitIntegrator(int64_t integrator, int32_t max_corr)
{
    int64_t integrator_limit;

    integrator_limit = (int64_t)max_corr *
                       (int64_t)DAC_NCO_PLL_FRAME_LEN *
                       (int64_t)DAC_NCO_PLL_STEP_KI_DIV;

    if (integrator_limit > DAC_NCO_PLL_INTEGRATOR_LIMIT)
    {
        integrator_limit = DAC_NCO_PLL_INTEGRATOR_LIMIT;
    }

    if (integrator > integrator_limit)
    {
        return integrator_limit;
    }

    if (integrator < -integrator_limit)
    {
        return -integrator_limit;
    }

    return integrator;
}

static int32_t DacNco_PidStepCorrection(uint8_t ch, int32_t phase_error)
{
    int32_t max_corr;
    int64_t integrator;
    int64_t correction;

    if (ch >= 2U)
    {
        return 0;
    }

    max_corr = DacNco_MaxCorrection(ch);
    integrator = ((s_pll[ch].integrator * DAC_NCO_PLL_INTEGRATOR_LEAK_NUM) / 65536LL) +
                 (int64_t)phase_error;
    integrator = DacNco_LimitIntegrator(integrator, max_corr);

    correction = ((int64_t)phase_error /
                  (int64_t)(DAC_NCO_PLL_FRAME_LEN * DAC_NCO_PLL_STEP_KP_DIV)) +
                 (integrator /
                  (int64_t)(DAC_NCO_PLL_FRAME_LEN * DAC_NCO_PLL_STEP_KI_DIV));

#if (DAC_NCO_PLL_STEP_KD_DIV != 0U)
    correction += ((int64_t)(phase_error - s_pll[ch].last_error) /
                   (int64_t)(DAC_NCO_PLL_FRAME_LEN * DAC_NCO_PLL_STEP_KD_DIV));
#endif

    s_pll[ch].integrator = integrator;
    s_pll[ch].last_error = phase_error;

    return DacNco_LimitCorrection(ch, correction);
}

/*
 * 配置 DAC NCO 输出参数。
 * A/B 保存两路已识别信号的频率、幅值、相位和波形类型；
 * phase_deg 是 CH2 相对 CH1 额外叠加的输出相位偏移。
 * 本函数只完成参数保存和 PLL 初始化，不启动 DAC DMA。
 */
void DacNco_Config(const SignalInfo *A,
                   const SignalInfo *B,
                   uint16_t phase_deg)
{
    if ((A == NULL) || (B == NULL))
    {
        s_ready = 0;
        return;
    }

    DacNco_BuildTables();
    memset((void *)dac_ch1_buf, 0, sizeof(dac_ch1_buf));
    memset((void *)dac_ch2_buf, 0, sizeof(dac_ch2_buf));

    s_sig[0] = *A;
    s_sig[1] = *B;
    s_phase_deg = phase_deg;
    s_output_phase_offset_b = DacNco_DegToPhaseQ32(s_phase_deg);

    if (s_sig[0].freq <= 0.0f)
    {
        s_sig[0].freq = 1000.0f;
    }

    if (s_sig[1].freq <= 0.0f)
    {
        s_sig[1].freq = 1000.0f;
    }

    DacNco_InitPll(0U, &s_sig[0]);
    DacNco_InitPll(1U, &s_sig[1]);

    s_dac_ready_mask = 0U;
    s_dac_sample_count = 0U;
    s_dac_ready_play_sample[0] = 0U;
    s_dac_ready_play_sample[1] = 0U;
    s_ready = 1;
}

/*
 * 设置两个 NCO 通道的相位参考采样编号。
 * 初次分离时，单频 DFT 测得的相位对应某一帧 ADC 的起始采样点，
 * 这里把该采样点编号作为 sample_ref，供后续相位预测使用。
 */
void DacNco_SetReferenceSample(uint64_t sample_ref)
{
    s_pll[0].sample_ref = sample_ref;
    s_pll[1].sample_ref = sample_ref;
}

/*
 * 启动 DAC 双通道 DMA 输出。
 * 启动前先根据当前 NCO/PLL 状态填好两个半缓冲区，
 * 然后依次启动 DAC CH1 和 CH2 的循环 DMA。
 */
/*
 * CODEx 修改：
 * 从指定的统一采样时间轴起点启动 DAC 输出。
 * 初次分离得到的 DFT 相位对应 adc_frame_sample_start；
 * DAC 真正启动时已经晚于那一帧，所以这里传入当前 adc_sample_count，
 * 让 DMA buffer 第一个点从当前 ADC 时间对应的 NCO 相位开始播放。
 */
void DacNco_StartAtSample(uint64_t start_sample)
{
    if ((!s_ready) || (s_running))
    {
        return;
    }

    s_dac_sample_count = start_sample;

    DacNco_FillHalf(0U, start_sample);
    DacNco_FillHalf(1U, start_sample + DAC_NCO_HALF_LEN);

    s_running = 1;
    if (HAL_DAC_Start_DMA(&hdac1,
                          DAC_CHANNEL_1,
                          (uint32_t *)dac_ch1_buf,
                          DAC_NCO_BUF_LEN,
                          DAC_ALIGN_12B_R) != HAL_OK)
    {
        s_running = 0;
        return;
    }

    if (HAL_DAC_Start_DMA(&hdac1,
                          DAC_CHANNEL_2,
                          (uint32_t *)dac_ch2_buf,
                          DAC_NCO_BUF_LEN,
                          DAC_ALIGN_12B_R) != HAL_OK)
    {
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        s_running = 0;
        return;
    }
}

/*
 * CODEx 修改：
 * 兼容旧接口。锁相启动路径应优先调用 DacNco_StartAtSample(adc_sample_count)。
 */
void DacNco_Start(void)
{
    DacNco_StartAtSample(s_pll[0].sample_ref);
}

/*
 * 停止 DAC 双通道 DMA 输出，并清除待重填标志。
 * 重新分离或异常停止时调用，避免继续输出旧的缓冲数据。
 */
void DacNco_Stop(void)
{
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
    s_running = 0;
    s_dac_ready_mask = 0U;
}

/*
 * 更新 CH2 相对 CH1 的额外输出相位偏移。
 * 该函数不重启 DMA，只更新 Q32 相位偏移量；
 * 后续 DAC 半缓冲重填时会自动使用新的相位偏移。
 */
void DacNco_UpdatePhase(uint16_t phase_deg)
{
    s_phase_deg = phase_deg;
    s_output_phase_offset_b = DacNco_DegToPhaseQ32(s_phase_deg);
}

/*
 * 更新指定通道的幅值和波形类型。
 * 跟踪过程中频率由 PLL 的 nominal_inc/inc_corr 管理，
 * 这里只更新输出幅值和正弦/三角类型，避免频率突然跳变。
 */
void DacNco_UpdateComponent(uint8_t ch, const SignalInfo *sig)
{
    if ((ch >= 2U) || (sig == NULL))
    {
        return;
    }

    s_sig[ch].amp = sig->amp;
    s_sig[ch].type = sig->type;
}

/*
 * 使用新一帧 ADC 的单频 DFT 相位更新 NCO 锁相环。
 * measured_phase_rad 是当前帧测得的相位；
 * frame_start_sample 是该帧第一个 ADC 采样点的绝对编号。
 * 函数会比较“测得相位”和“NCO 预测相位”，得到相位误差，
 * 再用 PID 风格的环路滤波器微调 phase_ref 与 inc_corr。
 */
void DacNco_PllUpdate(uint8_t ch, float measured_phase_rad, uint64_t frame_start_sample)
{
    uint32_t measured_phase;
    uint32_t predicted_phase;
    int32_t phase_error;

    if ((!s_ready) || (ch >= 2U))
    {
        return;
    }

    measured_phase = DacNco_RadToPhaseQ32(measured_phase_rad);
    predicted_phase = DacNco_PhaseAtSample(ch, frame_start_sample);
    phase_error = (int32_t)(measured_phase - predicted_phase);
    phase_error = DacNco_ApplyPhaseDeadband(phase_error);

    s_pll[ch].inc_corr = DacNco_PidStepCorrection(ch, phase_error);
    s_pll[ch].phase_ref = predicted_phase +
                          (uint32_t)(phase_error /
                                     (int32_t)(1UL << DAC_NCO_PLL_PHASE_KP_SHIFT));
    s_pll[ch].sample_ref = frame_start_sample;
}

/*
 * DAC NCO 后台任务函数。
 * DAC DMA 回调只设置哪个半缓冲已经释放；
 * 本函数在主循环中检查这些标志，并重填对应半缓冲。
 * 这样可以避免在中断回调里执行较长的查表和循环填充。
 */
void DacNco_Task(void)
{
    uint8_t ready_mask;
    uint64_t play_sample0;
    uint64_t play_sample1;

    if ((!s_ready) || (!s_running))
    {
        return;
    }

    __disable_irq();
    ready_mask = s_dac_ready_mask;
    play_sample0 = s_dac_ready_play_sample[0];
    play_sample1 = s_dac_ready_play_sample[1];
    s_dac_ready_mask = 0U;
    __enable_irq();

    if ((ready_mask & 0x01U) != 0U)
    {
        DacNco_FillHalf(0U, play_sample0);
    }

    if ((ready_mask & 0x02U) != 0U)
    {
        DacNco_FillHalf(1U, play_sample1);
    }
}

/*
 * DAC CH1 半传输完成回调。
 * 前半缓冲已经被 DMA 播放完，可以重新填充。
 * 这里只记录播放采样编号和置位标志，实际填充在 DacNco_Task() 中完成。
 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if ((hdac == &hdac1) && s_running)
    {
        s_dac_sample_count += DAC_NCO_HALF_LEN;
        s_dac_ready_play_sample[0] = s_dac_sample_count + DAC_NCO_HALF_LEN;
        s_dac_ready_mask |= 0x01U;
    }
}

/*
 * DAC CH1 全传输完成回调。
 * 后半缓冲已经被 DMA 播放完，可以重新填充。
 * 这里只记录播放采样编号和置位标志，实际填充在 DacNco_Task() 中完成。
 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if ((hdac == &hdac1) && s_running)
    {
        s_dac_sample_count += DAC_NCO_HALF_LEN;
        s_dac_ready_play_sample[1] = s_dac_sample_count + DAC_NCO_HALF_LEN;
        s_dac_ready_mask |= 0x02U;
    }
}
