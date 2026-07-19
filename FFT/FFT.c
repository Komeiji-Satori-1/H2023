#include "FFT.h"

/* -----------------------------------------------------------------------
 * 宏定义
 * ----------------------------------------------------------------------- */
#define FFT_LEN 1024 // FFT 点数，必须是2的幂
#define ADC_LEN 1024 // ADC 采样点数，与 FFT_LEN 保持一致


/* 变量 */
uint8_t ifftFlag = 0; 
int BaseIdx = 0; // 基波下标

float FFT_Output[FFT_LEN]; 
float FFT_Input[FFT_LEN*2]; 
float IFFT_Output[FFT_LEN];
float FFT_mag[FFT_LEN];

uint8_t EnableWindow=1; // 是否加窗
float Window_OutputBuffer[ADC_LEN]; // 窗函数输出缓冲

float FFT_Freq = 0;  // 当前帧 FFT 计算得到的基波频率 (Hz)
float DC = 0;        // 直流偏置（各采样点均值），FFT 前去除以消除直流分量
float FFT_mag_max = 0;          // 幅度谱峰值（归一化后）
uint32_t FFT_mag_max_index = 0; // 幅度谱峰值所在 bin 下标

static float window_power_correction = 1.0f; // 窗函数幅值补偿系数（Flat Top≈4.63867）
float fs = 200000.0f;//200k

//串口调试
void showdata(float *buffer, uint16_t n){
     for(uint8_t i=0;i<n;i++){
        printf("%.3f ", buffer[i]);
     }
}


float Calculate_DC_Value(uint16_t *ADC_Buffer)
{
    uint32_t sum = 0;

    for (uint16_t i = 0; i < 1024; i++)
    {
        sum += ADC_Buffer[i];
    }

    return (float)sum / 1024.0f;
}


//等效采樣換算頻率
void FFT_SetSampling(float sampling_freq)
{
    if(sampling_freq >1.0f) {
        fs=sampling_freq; 
    }
}

/**
 * @brief 在幅度谱前半段（单边谱）中找峰值 bin
 *
 * 只搜索 [0, FFT_LEN/2) 范围，因为 FFT 输出后半段是前半段的镜像（共轭对称）。
 * 同时更新全局 FFT_Freq（粗略频率）和 FFT_Ampl1（峰值幅度）。
 */
void Process_FFT_mag(float *FFT_mag, float *FFT_mag_max, uint32_t *FFT_mag_max_index, float *FFT_Ampl)
{

    arm_max_f32(FFT_mag, FFT_LEN / 2, FFT_mag_max, FFT_mag_max_index);
    FFT_Freq = (float)(*FFT_mag_max_index) * fs / (float)FFT_LEN;
    *FFT_Ampl = *FFT_mag_max;
}


void FFT_Process(uint16_t *ADC_Buffer, float *FFT_Ampl)
{
	/*
	arm_rfft_fast_instance_f32 S;
	arm_rfft_fast_init_f32(&S, FFT_LEN);
	
	ifftFlag = 0; 
	for(int i=0; i<FFT_LEN; i++)
	{
		FFT_Input[i] = ADC_Buffer[i]*Window_OutputBuffer[i];
	}
	arm_rfft_fast_f32(&S, FFT_Input, FFT_Output, ifftFlag);
	*/
	uint32_t adc_sum = 0;
    float *ampl;
    ampl = FFT_Ampl;

    //清空缓存区
    memset(FFT_Input, 0, sizeof(FFT_Input));
    memset(FFT_mag, 0, sizeof(FFT_mag));
    memset(FFT_Output, 0, sizeof(FFT_Output));
    //printf("clear cache\n");
    // 计算均值（直流偏置），后续减去以消除 DC 分量 
    DC = Calculate_DC_Value(ADC_Buffer);
  
    window();

    //去直流 + 加窗，虚部置0
    for (int i = 0; i < ADC_LEN; i++)
    {
        FFT_Input[i * 2] = ((float)ADC_Buffer[i] - DC) * Window_OutputBuffer[i];
        FFT_Input[i * 2 + 1] = 0.0f;
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_Input, 0, 1);

    // 计算幅度谱
    arm_cmplx_mag_f32(FFT_Input, FFT_mag, FFT_LEN);

    // 归一化 + 窗函数功率补偿 
    for (uint16_t i = 0; i < FFT_LEN; i++)
    {
        if (i == 0)
            FFT_mag[i] = FFT_mag[i] / FFT_LEN * window_power_correction;
        else
            FFT_mag[i] = FFT_mag[i] * 2.0f / FFT_LEN * window_power_correction;
    }

    //求幅度 矫正
    Process_FFT_mag(FFT_mag, &FFT_mag_max, &FFT_mag_max_index, ampl);
    ADC_FFT_Get_Wave_Mes(FFT_mag_max_index, fs, ampl, &FFT_Freq, 2);
  // *index = FFT_mag_max_index;

}



void IFFT_Process(void)
{
	/*
	arm_rfft_fast_instance_f32 S;
	arm_rfft_fast_init_f32(&S, FFT_LEN);

    ifftFlag = 1;
	arm_rfft_fast_f32(&S, FFT_Output, IFFT_Output, ifftFlag);
	*/
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_Input, 1, 1);

    // 提取实部作为 IFFT 输出
    for (int i = 0; i < FFT_LEN; i++) {
        IFFT_Output[i] = FFT_Input[2*i];  // 取实部
    }
}



void window(void)
{
    if (EnableWindow)
    {
        for (int i = 0; i < ADC_LEN; i++)
        {
            float tempCos = cosf(2.0f * PI * i / (ADC_LEN - 1));
            Window_OutputBuffer[i] = 0.5f * (1.0f - tempCos);
        }
        /* 窗函数增益系数*/
        window_power_correction = 1.55f;
    }
    else
    {
        for (int i = 0; i < ADC_LEN; i++)
        {
            Window_OutputBuffer[i] = 1.0f;
        }
        window_power_correction = 1.0f;
    }
}

/* 找到基波的下标*/
void Find_BaseIndex(void)
{
    BaseIdx = 0;
    float max_val = 0;
    for (int i = 2; i < FFT_LEN / 2; i++) { // 遍历 0 ~ Fs/2 部分
        if (FFT_Output[i] > max_val) {
            max_val = FFT_Output[i];
            BaseIdx = i; // 记录基波的索引
        }
    }
}

/*输入参数为FFT计算后的结果，输出矫正后的频率和幅度

FFT_mag_max_index				FFT结果中峰值的位置
fs				采样频率
FFT_Ampl	    矫正后的幅值
Freq[0]			矫正后的频率
correctNum		矫正的点数，一般取2即可，确保峰值左右的correctNum内没有其他信号
FFT_mag		FFT结果的幅值数组
FFT_mag_max_index				FFT结果中峰值的位置
fs				采样频率
FFT_Ampl	    矫正后的幅值
Freq[0]			矫正后的频率
correctNum		矫正的点数，一般取2即可，确保峰值左右的correctNum内没有其他信号
FFT_mag		FFT结果的幅值数组
*/

void ADC_FFT_Get_Wave_Mes(uint32_t FFT_mag_max_index, float fs, float *FFT_Ampl, float *Freq, int correctNum)
{
    int i;
    float DatePower1 = 0.0f;
    float DatePower2 = 0.0f;
    float f;

    for (i = -correctNum; i <= correctNum; i++)
    {
        DatePower1 += (FFT_mag_max_index + i) * FFT_mag[FFT_mag_max_index + i] * FFT_mag[FFT_mag_max_index + i];
        DatePower2 += FFT_mag[FFT_mag_max_index + i] * FFT_mag[FFT_mag_max_index + i];
    }

    f = DatePower1 / DatePower2;
    *Freq = f * fs / FFT_LEN;
    *FFT_Ampl = sqrtf(DatePower2);
}

//单频点DFT
#define DFT_PI 3.14159265358979323846f

void FFT_SingleFreqDFT_U16(const uint16_t *adc_buf,
                           uint16_t len,
                           float fs,
                           float target_freq,
                           FFT_SingleFreqResult_t *result)
{
    uint16_t i;
    float dc = 0.0f;
    float real = 0.0f;
    float imag = 0.0f;
    float sample;
    float angle;

    if ((adc_buf == NULL) || (result == NULL) || (len == 0U) || (fs <= 0.0f))
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        dc += (float)adc_buf[i];
    }

    dc /= (float)len;

    for (i = 0; i < len; i++)
    {
        sample = (float)adc_buf[i] - dc;
        angle = 2.0f * DFT_PI * target_freq * (float)i / fs;

        real += sample * cosf(angle);
        imag -= sample * sinf(angle);
    }

    result->real = real;
    result->imag = imag;
    result->mag = 2.0f * sqrtf(real * real + imag * imag) / (float)len;
    result->phase = atan2f(imag, real);
}

void FFT_CalcTransfer(const FFT_SingleFreqResult_t *input,
                      const FFT_SingleFreqResult_t *output,
                      FFT_TransferResult_t *h)
{
    float denominator;

    if ((input == NULL) || (output == NULL) || (h == NULL))
    {
        return;
    }

    denominator = input->real * input->real + input->imag * input->imag;

    if (denominator < 1e-12f)
    {
        h->real = 0.0f;
        h->imag = 0.0f;
        h->mag = 0.0f;
        h->phase = 0.0f;
        return;
    }

    // H = output / input
    h->real = (output->real * input->real + output->imag * input->imag) / denominator;
    h->imag = (output->imag * input->real - output->real * input->imag) / denominator;

    h->mag = sqrtf(h->real * h->real + h->imag * h->imag);
    h->phase = atan2f(h->imag, h->real);
}