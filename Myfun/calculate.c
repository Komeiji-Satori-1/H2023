#include "calculate.h"
#include "AD9833.h"
#include "iir.h"
#include "modify_adc.h"

extern void Start_ADC_Capture(void);
extern uint16_t ADC1_IN[ADC_LEN];
extern uint16_t ADC2_OUT[ADC_LEN];
#define VIN_AMP_TABLE_SIZE (sizeof(vin_amp_table) / sizeof(vin_amp_table[0]))

#define LEARN_START_FREQ_HZ 100U
#define LEARN_STOP_FREQ_HZ 100000U
#define LEARN_STEP_FREQ_HZ 200U
#define LEARN_SETTLE_MS 20U
#define LEARN_PRINT_INTERVAL_HZ 1000U

#define LEARN_POINT_NUM (((LEARN_STOP_FREQ_HZ - LEARN_START_FREQ_HZ) / LEARN_STEP_FREQ_HZ) + 1U)

float freq_table[LEARN_POINT_NUM];
static complex h_table[LEARN_POINT_NUM];
static analog_coef analog_coef_data;
static digital_coef digital_coef_data;
static uint8_t learn_done = 0;
static uint8_t iir_coeff_ready = 0;

typedef enum
{
    LEARN_IDLE = 0,
    LEARN_SET_FREQ,
    LEARN_WAIT_STABLE,
    LEARN_START_ADC,
    LEARN_WAIT_ADC,
    LEARN_PROCESS_FFT,
    LEARN_NEXT_FREQ,
    LEARN_CALC_IIR,
    LEARN_DONE,
} LearnState_t;

typedef struct
{
    LearnState_t state;
    uint32_t freq;
    uint32_t wait_tick;
    uint16_t index;
    uint8_t running;
} LearnCtrl_t;

static LearnCtrl_t learn = {0};

typedef struct
{
    float vin;
    uint8_t code;
} VinAmpCodeTable_t;

float vout = 2.0f;
uint32_t f = 1000.0f;
float H_jw = 5.0f;
float vin = 0.0f;

static const VinAmpCodeTable_t vin_amp_table[] =
    {
        {0.0542f, 1},
        {0.0754f, 2},
        {0.01f, 3},
        {0.125f, 4},
        {0.150f, 5},
        {0.180f, 6},
        {0.204f, 7},
        {0.231f, 8},
        {0.255f, 9},
        {0.279f, 10},
        {0.305f, 11},
        {0.329f, 12},
        {0.354f, 13},
        {0.380f, 14},
        {0.415f, 15},
        {0.438f, 16},
        {0.462f, 17},
        {0.490f, 18},
        {0.514f, 19},
        {0.540f, 20},
        {0.564f, 21},
        {0.590f, 22},
        {0.613f, 23},
        {0.638f, 24},
        {0.662f, 25},
        {0.684f, 26},
        {0.714f, 27},
        {0.736f, 28},
        {0.762f, 29},
        {0.788f, 30},
        {0.828f, 31},
        {0.848f, 32},
        {0.874f, 33},
        {0.896f, 34},
        {0.924f, 35},
        {0.948f, 36},
        {0.976f, 37},
        {1.000f, 38},
        {1.020f, 39},
        {1.040f, 40},
        {1.080f, 41},
        {1.100f, 42},
        {1.120f, 43},
        {1.150f, 44},
        {1.170f, 45},
        {1.200f, 46},
        {1.220f, 47},
        {1.240f, 48},
        {1.270f, 49},
        {1.300f, 50},
        {1.320f, 51},
        {1.350f, 52},
        {1.370f, 53},
        {1.400f, 54},
        {1.420f, 55},
        {1.450f, 56},
        {1.470f, 57},
        {1.500f, 58},
        {1.520f, 59},
        {1.540f, 60},
        {1.570f, 61},
        {1.600f, 62},
        {1.660f, 63},
        {1.680f, 64},
        {1.710f, 65},
        {1.740f, 66},
        {1.770f, 67},
        {1.800f, 68},
        {1.820f, 69},
        {1.840f, 70},
        {1.870f, 71},
        {1.890f, 72},
        {1.920f, 73},
        {1.940f, 74},
        {1.970f, 75},
        {2.000f, 76},
        {2.020f, 77},
        {2.040f, 78},
        {2.070f, 79},
        {2.100f, 80},
        {2.120f, 81},
        {2.140f, 82},
        {2.170f, 83},
        {2.200f, 84},
        {2.220f, 85},
        {2.240f, 86},
        {2.270f, 87},
        {2.300f, 88},
        {2.320f, 89},
        {2.350f, 90},
        {2.370f, 91},
        {2.400f, 92},
        {2.420f, 93},
        {2.450f, 94},
        {2.480f, 95},
        {2.500f, 96},
        {2.520f, 97},
        {2.540f, 98},
        {2.570f, 99},
        {2.600f, 100},
        {2.620f, 101},
        {2.640f, 102},
        {2.670f, 103},
        {2.700f, 104},
        {2.720f, 105},
        {2.740f, 106},
        {2.770f, 107},
        {2.800f, 108},
        {2.820f, 109},
        {2.840f, 110},
        {2.860f, 111},
        {2.890f, 112},
        {2.920f, 113},
        {2.940f, 114},
        {2.970f, 115},
        {3.000f, 116},
        {3.020f, 117},
        {3.040f, 118},
        {3.080f, 119},
        {3.090f, 120},
        {3.120f, 121},
        {3.140f, 122},
        {3.170f, 123},
        {3.200f, 124},
        {3.220f, 125},
        {3.250f, 126},
        {3.270f, 127},
        {3.300f, 128},
        {3.330f, 129},
        {3.350f, 130},
        {3.370f, 131},
        {3.390f, 132},
        {3.420f, 133},
        {3.440f, 134},
        {3.470f, 135},
        {3.490f, 136},
        {3.520f, 137},
        {3.540f, 138},
        {3.570f, 139},
        {3.610f, 140},
        {3.630f, 141},
        {3.650f, 142},
        {3.680f, 143},
        {3.700f, 144},
        {3.720f, 145},
        {3.740f, 146},
        {3.760f, 147},
        {3.800f, 148},
        {3.820f, 149},
        {3.840f, 150},
        {3.870f, 151},
        {3.910f, 152},
        {3.930f, 153},
        {3.950f, 154},
        {3.970f, 155},
        {4.000f, 156},
        {4.120f, 157},
        {4.160f, 158},
        {4.200f, 159},
        {4.220f, 160},
        {4.240f, 161},
        {4.280f, 162},
        {4.300f, 163},
        {4.320f, 164},
        {4.340f, 165},
        {4.360f, 166},
        {4.390f, 167},
        {4.420f, 168},
        {4.440f, 169},
        {4.480f, 170},
        {4.500f, 171},
        {4.520f, 172},
        {4.540f, 173},
        {4.560f, 174},
        {4.580f, 175},
        {4.620f, 176},
        {4.640f, 177},
        {4.670f, 178},
        {4.700f, 179},
        {4.720f, 180},
        {4.750f, 181},
        {4.770f, 182},
        {4.800f, 183},
        {4.830f, 184},
        {4.850f, 185},
        {4.870f, 186},
        {4.880f, 187},
        {4.890f, 188},
        {4.900f, 189},
        {4.910f, 190},
        {4.911f, 191},
        {4.912f, 192},
        {4.913f, 193},
        {4.914f, 194},
        {4.915f, 195},
        {4.916f, 196},
        {4.917f, 197},
        {4.918f, 198},
        {4.919f, 199},
        {4.920f, 200},
        {4.921f, 201},
        {4.922f, 202},
        {4.923f, 203},
        {4.924f, 204},
        {4.925f, 205},
        {4.926f, 206},
        {4.927f, 207},
        {4.928f, 208},
        {4.929f, 209},
        {4.930f, 210},
        {4.931f, 211},
        {4.932f, 212},
        {4.933f, 213},
        {4.934f, 214},
        {4.935f, 215},
        {4.936f, 216},
        {4.937f, 217},
        {4.938f, 218},
        {4.939f, 219},
        {4.940f, 220},
        {4.941f, 221},
        {4.942f, 222},
        {4.943f, 223},
        {4.944f, 224},
        {4.945f, 225},
        {4.946f, 226},
        {4.947f, 227},
        {4.948f, 228},
        {4.949f, 229},
        {4.950f, 230},
        {4.951f, 231},
        {4.952f, 232},
        {4.953f, 233},
        {4.954f, 234},
        {4.955f, 235},
        {4.956f, 236},
        {4.957f, 237},
        {4.958f, 238},
        {4.959f, 239},
        {4.960f, 240},
        {4.961f, 241},
        {4.962f, 242},
        {4.963f, 243},
        {4.964f, 244},
        {4.965f, 245},
        {4.966f, 246},
        {4.967f, 247},
        {4.968f, 248},
        {4.969f, 249},
        {4.970f, 250},
        {4.971f, 251},
        {4.972f, 252},
        {4.973f, 253},
        {4.974f, 254},
        {4.975f, 255},
};

#define S0_WAVE_ID      16      // 改成串口屏里 s0 的真实 id
#define S0_WAVE_CH      0U
#define S0_WAVE_POINTS  219U    // 改成 s0 波形控件宽度，常见 240/320

static float64_t calc_fit_mag_at_freq(const analog_coef *coef, float64_t freq_hz)
{
    float64_t w = 2.0 * PI * freq_hz;
    float64_t w2 = w * w;

    float64_t nr = coef->b0 - coef->b2 * w2;
    float64_t ni = coef->b1 * w;

    float64_t dr = 1.0 - coef->a2 * w2;
    float64_t di = coef->a1 * w;

    float64_t den = dr * dr + di * di;
    if (den <= EPS)
    {
        return 0.0;
    }

    return sqrt((nr * nr + ni * ni) / den);
}

static void show_fit_curve_on_s0(const analog_coef *coef)
{
    uint8_t wave_data[S0_WAVE_POINTS];
    float64_t mag_data[S0_WAVE_POINTS];
    float64_t max_mag = 0.0;

    for (uint16_t i = 0; i < S0_WAVE_POINTS; i++)
    {
        float64_t freq = LEARN_START_FREQ_HZ +
                         ((float64_t)(LEARN_STOP_FREQ_HZ - LEARN_START_FREQ_HZ) * i) /
                         (float64_t)(S0_WAVE_POINTS - 1U);

        mag_data[i] = calc_fit_mag_at_freq(coef, freq);

        if (mag_data[i] > max_mag)
        {
            max_mag = mag_data[i];
        }
    }

    for (uint16_t i = 0; i < S0_WAVE_POINTS; i++)
    {
        if (max_mag <= EPS)
        {
            wave_data[i] = 0U;
        }
        else
        {
            float64_t y = mag_data[i] * 255.0 / max_mag;
            if (y > 255.0)
            {
                y = 255.0;
            }
            wave_data[S0_WAVE_POINTS - 1U - i] = (uint8_t)(y + 0.5);
        }
    }

    HMI_Wave_Clear(S0_WAVE_ID, S0_WAVE_CH);
    HAL_Delay(20);
    HMI_Wave_Fast(S0_WAVE_ID, S0_WAVE_CH, S0_WAVE_POINTS, wave_data);
}

float calculate_vin(float vout, uint32_t f)
{

    uint32_t f2 = f * f; // f^2
    // (1 - 3.94784 * 10^-7 * f^2)
    float term1 = 1.0f - 3.94784e-7f * f2;
    //(1.88496 * 10^-3 * f)
    float term2 = 1.88496e-3f * f;
    H_jw = 5.0f / sqrtf(term1 * term1 + term2 * term2);

    vin = vout / H_jw;

    return vin;
}

static uint8_t clamp_amp_code(float code)
{
    if (code <= 0.0f)
    {
        return 0;
    }

    if (code >= 255.0f)
    {
        return 255;
    }

    return (uint8_t)(code+0.5f);
}

uint8_t calculate_ad9833_amp_code(float vin)
{
    uint16_t i;
    float vin0;
    float vin1;
    float code0;
    float code1;
    float ratio;
    float code;

    if (vin <= vin_amp_table[0].vin)
    {
        return vin_amp_table[0].code;
    }

    if (vin >= vin_amp_table[VIN_AMP_TABLE_SIZE - 1].vin)
    {
        return vin_amp_table[VIN_AMP_TABLE_SIZE - 1].code;
    }

    for (i = 0; i < VIN_AMP_TABLE_SIZE - 1; i++)
    {
        vin0 = vin_amp_table[i].vin;
        vin1 = vin_amp_table[i + 1].vin;

        if ((vin >= vin0) && (vin <= vin1))
        {
            code0 = (float)vin_amp_table[i].code;
            code1 = (float)vin_amp_table[i + 1].code;

            ratio = (vin - vin0) / (vin1 - vin0);
            code = code0 + ratio * (code1 - code0);

            return clamp_amp_code(code);
        }
    }

    return vin_amp_table[VIN_AMP_TABLE_SIZE - 1].code;
}

void calculate_set_ad9833_amp_by_vin(float vin)
{
    uint8_t amp_code;

    amp_code = calculate_ad9833_amp_code(vin);
    AD9833_AmpSet(amp_code);
}

void calculate_learn_start(void)
{
    learn_done = 0;
    learn.running = 1;
    learn_done = 0;
    iir_coeff_ready = 0;
    learn.state = LEARN_SET_FREQ;
    learn.freq = LEARN_START_FREQ_HZ;
    learn.index = 0;
    printf("freq,H_mag,H_phase\r\n");
}
uint8_t calculate_learn_is_done(void)
{
    return learn_done;
}


void calculate_learn_proc(void)
{
    switch (learn.state)
    {
    case LEARN_IDLE:
        break;

    case LEARN_SET_FREQ:
        //printf("Setting frequency to %lu Hz\n", learn.freq);
        AD9833_WaveSeting(learn.freq, 0, SIN_WAVE, 0);
        learn.wait_tick = HAL_GetTick(); // 计时函数
        learn.state = LEARN_WAIT_STABLE;
        break;

    case LEARN_WAIT_STABLE:
        if (HAL_GetTick() - learn.wait_tick >= LEARN_SETTLE_MS) // 判断是否超过等待时间，让信号稳定在开启adc
        {
            learn.state = LEARN_START_ADC;
        }
        break;

    case LEARN_START_ADC:
        Start_ADC_Capture();
        learn.state = LEARN_WAIT_ADC;
        break;

    case LEARN_WAIT_ADC:
        if (g_adc_mode_ctrl.adc_flag)
        {
            learn.state = LEARN_PROCESS_FFT;
        }
        break;

    case LEARN_PROCESS_FFT:
        FFT_SingleFreqResult_t input_dft;
        FFT_SingleFreqResult_t output_dft;
        FFT_TransferResult_t h;

        FFT_SingleFreqDFT_U16(ADC1_IN, ADC_LEN, (float)Fs, (float)learn.freq, &input_dft);
        FFT_SingleFreqDFT_U16(ADC2_OUT, ADC_LEN, (float)Fs, (float)learn.freq, &output_dft);

        FFT_CalcTransfer(&input_dft, &output_dft, &h);

        if (((learn.freq - LEARN_START_FREQ_HZ) % LEARN_PRINT_INTERVAL_HZ) == 0U)
        {
            printf("Freq = %lu Hz\n", (unsigned long)learn.freq);
            printf("Input:  real=%.3f, imag=%.3f, mag=%.3f, phase=%.3f\n",
                   input_dft.real, input_dft.imag, input_dft.mag, input_dft.phase);
            printf("Output: real=%.3f, imag=%.3f, mag=%.3f, phase=%.3f\n",
                   output_dft.real, output_dft.imag, output_dft.mag, output_dft.phase);
            printf("H:      real=%.6f, imag=%.6f, mag=%.6f, phase=%.6f\n",
                   h.real, h.imag, h.mag, h.phase);
        }

        freq_table[learn.index] = (float)learn.freq;
        h_table[learn.index].r = h.real;
        h_table[learn.index].i = h.imag;
        printf("%lu,%.6f,%.6f\r\n",
               learn.freq,
               h.mag,
               h.phase);
        learn.state = LEARN_NEXT_FREQ;
        break;

    case LEARN_NEXT_FREQ:
        if ((learn.index + 1U) >= LEARN_POINT_NUM)
        {
            learn.state = LEARN_CALC_IIR;
        }
        else
        {
            learn.index++;
            learn.freq = LEARN_START_FREQ_HZ + learn.index * LEARN_STEP_FREQ_HZ;
            learn.state = LEARN_SET_FREQ;
        }
        break;

    case LEARN_CALC_IIR:
        coef_calc(h_table);
        analog_coef_data = matrix_calc();
        digital_coef_data = bilinear_transform_quant(&analog_coef_data);
        iir_coeff_ready = 1;
        show_filter_type(get_last_fit_filter_type());
        show_fit_curve_on_s0(&analog_coef_data);

        // 根据扫频得到的 H(jw) 计算 IIR 参数
        // 判断滤波器类型
        learn.state = LEARN_DONE;
        break;

    case LEARN_DONE:
        learn_done = 1;
        learn.running = 0;
        learn_done = 1;
        learn.state = LEARN_IDLE;
        break;

    default:
        learn.state = LEARN_IDLE;
        break;
    }
}

uint8_t get_learn_done(void)
{
    return learn_done;
}

const digital_coef *calculate_get_digital_coef(void)
{
    return &digital_coef_data;
}

uint8_t calculate_iir_coeff_ready(void)
{
    return iir_coeff_ready;
}


