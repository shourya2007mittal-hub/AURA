/*
 * ============================================================================
 * CUSTOM TAILLESS TWIN-MOTOR FLIGHT CONTROLLER FIRMWARE
 * Target MCU: STM32F401CBUx
 * ============================================================================
 */

#include "main.h"
#include <math.h>

// --- DEFINITIONS & CONSTANTS ---
#define MPU6050_ADDR    (0x68 << 1)
#define BMP280_ADDR     (0x76 << 1)
#define SBUS_FRAME_SIZE 25

// --- PERIPHERAL HANDLES ---
I2C_HandleTypeDef hi2c1;   // IMU 1 (MPU6050) via PB6/PB7
I2C_HandleTypeDef hi2c2;   // Barometer (BMP280) via PB10/PB3
TIM_HandleTypeDef htim1;   // ESCs (PA8, PA9)
TIM_HandleTypeDef htim2;   // Servos (PA0, PA1)
UART_HandleTypeDef huart1; // SBUS Receiver Input (PA10)

// --- FLIGHT CONTROL VARIABLES ---
float roll_input = 0.0f, pitch_input = 0.0f, throttle_input = 1000.0f;
float current_roll = 0.0f, current_pitch = 0.0f;
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f;

// PID Tuning Constants (Adjust these during test flights)
float Kp_roll = 1.2f,  Ki_roll = 0.05f,  Kd_roll = 0.1f;
float Kp_pitch = 1.5f, Ki_pitch = 0.05f, Kd_pitch = 0.1f;

// PID Memory Registers
float error_roll, integral_roll, derivative_roll, last_error_roll, out_roll;
float error_pitch, integral_pitch, derivative_pitch, last_error_pitch, out_pitch;

// SBUS Protocol Variables
uint8_t sbus_buffer[SBUS_FRAME_SIZE];
uint16_t sbus_channels[4];
uint8_t sbus_index = 0;
uint8_t is_armed = 0;
uint8_t rx_byte;

// Barometer Calibration Factors
uint16_t dig_T1; int16_t dig_T2, dig_T3; uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// --- FUNCTION PROTOTYPES ---
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

void IMU_Calibrate(void);
void IMU_Read(float *r, float *p);
void BMP280_Init(void);
float BMP280_GetAltitude(void);
void Mix_Elevons(float throttle, float roll, float pitch);

// --- MAIN PROGRAM EXECUTION ---
int main(void) {
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock to 84 MHz using the 25MHz external crystal
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    // Device Sensors Boot & Calibration Sequence
    BMP280_Init();
    IMU_Calibrate();
    
    // Start Non-blocking UART Reception for RC Signal
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    // Start PWM Hardware Timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Left ESC
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Right ESC
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Left Servo
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Right Servo

    uint32_t loop_timer = HAL_GetTick();

    while (1) {
        // Main Flight Controller Loop (Executes strictly at 200Hz -> every 5ms)
        if ((HAL_GetTick() - loop_timer) >= 5) {
            loop_timer = HAL_GetTick();

            // 1. Refresh Flight Performance Matrix Data
            IMU_Read(&current_roll, &current_pitch);
            float current_altitude = BMP280_GetAltitude();

            // 2. Structural Safety Timeout Arm Interlock
            if (!is_armed) {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000); // Force Motors Off
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
                continue; // Skip PID execution if safety switch is down
            }

            // 3. Roll Axis PID Controller
            error_roll = roll_input - current_roll;
            integral_roll += error_roll * 0.005f;
            derivative_roll = (error_roll - last_error_roll) / 0.005f;
            out_roll = (Kp_roll * error_roll) + (Ki_roll * integral_roll) + (Kd_roll * derivative_roll);
            last_error_roll = error_roll;

            // 4. Pitch Axis PID Controller
            error_pitch = pitch_input - current_pitch;
            integral_pitch += error_pitch * 0.005f;
            derivative_pitch = (error_pitch - last_error_pitch) / 0.005f;
            out_pitch = (Kp_pitch * error_pitch) + (Ki_pitch * integral_pitch) + (Kd_pitch * derivative_pitch);
            last_error_pitch = error_pitch;

            // 5. Send Mixed Commands to Control Surfaces and Power Units
            Mix_Elevons(throttle_input, out_roll, out_pitch);
        }
    }
}

// --- MIXING & CONTROL SURFACE LOGIC ---
void Mix_Elevons(float throttle, float roll, float pitch) {
    // Elevon Aerodynamic Mixer Logic for Tailless Aircraft
    float s_left  = 1500 + pitch + roll;
    float s_right = 1500 - pitch + roll;

    // Hard Clip Servo Output Constraints (1000us to 2000us)
    s_left  = (s_left > 2000) ? 2000 : ((s_left < 1000) ? 1000 : s_left);
    s_right = (s_right > 2000) ? 2000 : ((s_right < 1000) ? 1000 : s_right);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)s_left);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)s_right);

    // Twin Engine Differential Thrust Mixing to assist Yaw Control
    float esc_l = throttle + (roll * 0.15f);
    float esc_r = throttle - (roll * 0.15f);

    // Protect against motor overflow parameters
    esc_l = (esc_l > 2000) ? 2000 : ((esc_l < 1000) ? 1000 : esc_l);
    esc_r = (esc_r > 2000) ? 2000 : ((esc_r < 1000) ? 1000 : esc_r);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)esc_l);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)esc_r);
}

// --- IMU TRANSLATION & CALIBRATION DRIVERS ---
void IMU_Calibrate(void) {
    uint8_t data[6];
    int32_t gx = 0, gy = 0;
    
    // Hold completely level and stable during initialization
    for (int i = 0; i < 400; i++) {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, data, 6, 10);
        gx += (int16_t)((data[0] << 8) | data[1]);
        gy += (int16_t)((data[2] << 8) | data[3]);
        HAL_Delay(2);
    }
    gyro_bias_x = (float)gx / 400.0f / 65.5f;
    gyro_bias_y = (float)gy / 400.0f / 65.5f;
}

void IMU_Read(float *r, float *p) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 6, 10);
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    // Compute Pitch/Roll orientation vectors from Accelerometer forces
    float acc_roll  = atan2f((float)ay, (float)az) * 57.2957f;
    float acc_pitch = atan2f(-(float)ax, sqrtf((float)ay*(float)ay + (float)az*(float)az)) * 57.2957f;

    // Complimentary Noise Filtering step
    *r = 0.98f * (*r) + 0.02f * acc_roll;
    *p = 0.98f * (*p) + 0.02f * acc_pitch;
}

// --- BAROMETER SENSOR INTERFACE ---
void BMP280_Init(void) {
    uint8_t calib[24], config[2] = {0xF4, 0x57};
    HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT, calib, 24, 100);
    
    dig_T1 = (calib[1] << 8) | calib[0]; dig_T2 = (calib[3] << 8) | calib[2];
    dig_P1 = (calib[7] << 8) | calib[6]; dig_P2 = (calib[9] << 8) | calib[8];
    
    HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDR, config, 2, 100);
}

float BMP280_GetAltitude(void) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR, 0xF7, I2C_MEMADD_SIZE_8BIT, data, 3, 100);
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    
    // Returns estimated altitude value in relative meters
    return 44330.0f * (1.0f - powf(((float)adc_P / 256.0f) / 101325.0f, 0.1903f));
}

// --- INTERRUPT BASED SBUS DATA RECEIVER ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t byte = rx_byte;
        
        if (sbus_index == 0 && byte != 0x0F) return; // Verify frame start bit
        
        sbus_buffer[sbus_index++] = byte;
        
        if (sbus_index == SBUS_FRAME_SIZE) {
            sbus_index = 0;
            if (sbus_buffer[24] == 0x00) { // Verify validity of end bit sequence
                sbus_channels[0] = ((sbus_buffer[1] | sbus_buffer[2] << 8) & 0x07FF);
                sbus_channels[1] = ((sbus_buffer[2] >> 3 | sbus_buffer[3] << 5) & 0x07FF);
                sbus_channels[2] = ((sbus_buffer[3] >> 6 | sbus_buffer[4] << 2) & 0x07FF);
                
                // Normalization mapping into standard PWM scale inputs
                roll_input     = (1000 + ((sbus_channels[0] - 172) * 1000 / 1639)) - 1500;
                pitch_input    = (1000 + ((sbus_channels[1] - 172) * 1000 / 1639)) - 1500;
                throttle_input = 1000 + ((sbus_channels[2] - 172) * 1000 / 1639);
                
                // Read specific bit flags to check the configuration state of safety switch
                is_armed = (sbus_buffer[23] & 0x01); 
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // Re-arm the interrupt listener
    }
}

// --- HARDWARE SYSTEM CLOCK CONFIGURATION (84 MHz via HSE) ---
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE_2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

// --- BASIC PERIPHERAL LOW LEVEL INITIALIZERS ---
static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c1);
}

static void MX_I2C2_Init(void) {
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    HAL_I2C_Init(&hi2c2);
}

static void MX_TIM1_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 84-1; // Clock tick resolution becomes 1 microsecond
    htim1.Init.Period = 20000-1; // 20ms Frame period -> 50Hz PWM Output frequency
    HAL_TIM_PWM_Init(&htim1);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000; // Initialize at safe idle pulse threshold
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
}

static void MX_TIM2_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84-1;
    htim2.Init.Period = 20000-1;
    HAL_TIM_PWM_Init(&htim2);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500; // Stabilize servos at exact midpoint baseline
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
}

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 100000; // Exact structural transmission frequency of SBUS
    huart1.Init.WordLength = UART_WORDLENGTH_9B;
    huart1.Init.StopBits = UART_STOPBITS_2;
    huart1.Init.Parity = UART_PARITY_EVEN;
    huart1.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart1);
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}