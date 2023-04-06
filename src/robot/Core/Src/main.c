/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUGGING
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch4_up;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t WS2812Buffer[24 * LED_COUNT] = {0};
unsigned short LEDStep = 0;

uint16_t ADCBuffer[100];

char StringBuffer[100];

unsigned int TIM4LastFallingEdge;
unsigned int TIM4FallingEdge;
unsigned int ReceivedPeriodBuffer;
unsigned int ReceivedPeriod;

unsigned int SonarRisingEdge;
unsigned int SonarFallingEdge;
unsigned int SonarOverflowCount = 0;
long SonarPulse;
unsigned int SonarPulsePolarity = 1;

int Mode = 0; // 0 = track, 1 = controlled
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void SetColor(unsigned int index, unsigned short r, unsigned short g, unsigned short b);
void SetColorAll(unsigned short r, unsigned short g, unsigned short b);
void Rainbow(int length, float phase, float brightness);
void LeftMotor(int a, int b);
void RightMotor(int a, int b);
void SetTailLights(unsigned short r, unsigned short g, unsigned short b);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    MX_TIM3_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    HAL_ADC_Start_DMA(&hadc1, ADCBuffer, ARRAYLEN(ADCBuffer));

    LEFTSTOP();
    RIGHTSTOP();

    SetColorAll(0, 0, 0);

    sprintf(StringBuffer, "Start\r\n");
    HAL_UART_Transmit(&huart1, StringBuffer, strlen(StringBuffer), 10);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        switch (Mode)
        {
        case 0: // tracking mode
            if (SonarPulse < 40)
            {
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 1);
            }
            else
            {
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
            }
#ifdef DEBUGGING
            sprintf(StringBuffer, "Mode 0 ADC: %4d %4d Sonar: %d OV: %d\r\n", LEFTADC, RIGHTADC, SonarPulse, SonarOverflowCount);
            HAL_UART_Transmit(&huart1, StringBuffer, strlen(StringBuffer), 10);
            HAL_Delay(10);
#endif

            if (LEFTADC < TARGETADCLEFT - DEADZONE)
            {
                LEFTFORWARD();
            }
            else if (LEFTADC > TARGETADCLEFT + DEADZONE)
            {
                LEFTREVERSE();
            }
            else
            {
                LEFTSTOP();
            }

            if (RIGHTADC < TARGETADCRIGHT - DEADZONE)
            {
                RIGHTFORWARD();
            }
            else if (RIGHTADC > TARGETADCRIGHT + DEADZONE)
            {
                RIGHTREVERSE();
            }
            else
            {
                RIGHTSTOP();
            }
            break;
        case 1: // controlled mode
#ifdef DEBUGGING
            sprintf(StringBuffer, "Mode 1 PeriodB: %4d Period: %4d\r\n", ReceivedPeriodBuffer, ReceivedPeriod);
            HAL_UART_Transmit(&huart1, StringBuffer, strlen(StringBuffer), 10);
            HAL_Delay(10);
#endif
            ReceivedPeriod = ReceivedPeriodBuffer;
            if (COMPARE(ReceivedPeriod, 600, 25)) // forward
            {
                if (SonarPulse > 30)
                {
                    LEFTFORWARD();
                    RIGHTFORWARD();
                    SetTailLights(0, 0, 0);
                    HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
                }
                else
                {
                    SetTailLights(30, 0, 0);
                    HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 1);
                }
            }
            else if (COMPARE(ReceivedPeriod, 650, 25)) // forward right
            {
                LEFTFORWARD();
                RIGHTSTOP();
                SetTailLights(0, 0, 0);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
            }
            else if (COMPARE(ReceivedPeriod, 700, 25)) // right
            {
                LEFTFORWARD();
                RIGHTREVERSE();
                SetTailLights(0, 0, 0);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
            }
            else if (COMPARE(ReceivedPeriod, 750, 25)) // reverse right
            {
                LEFTSTOP();
                RIGHTREVERSE();
                SetTailLights(10, 10, 10);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 1);
            }
            else if (COMPARE(ReceivedPeriod, 800, 25)) // reverse
            {
                LEFTREVERSE();
                RIGHTREVERSE();
                SetTailLights(10, 10, 10);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 1);
            }
            else if (COMPARE(ReceivedPeriod, 850, 25)) // reverse left
            {
                LEFTREVERSE();
                RIGHTSTOP();
                SetTailLights(10, 10, 10);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 1);
            }
            else if (COMPARE(ReceivedPeriod, 900, 25)) // left
            {
                LEFTREVERSE();
                RIGHTFORWARD();
                SetTailLights(0, 0, 0);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
            }
            else if (COMPARE(ReceivedPeriod, 950, 25)) // forward left
            {
                LEFTSTOP();
                RIGHTFORWARD();
                SetTailLights(0, 0, 0);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
            }
            else // stop
            {
                LEFTSTOP();
                RIGHTSTOP();
                SetTailLights(30, 0, 0);
                HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, 0);
            }
            break;
        default:
            break;
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 720 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 2000 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7200 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 5000 - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
    HAL_TIM_Base_Start_IT(&htim2);
    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 90 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 7200 - 1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
    /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LEFT_MOTOR_B_Pin | LEFT_MOTOR_A_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, BUZZER_PIN_Pin | RIGHT_MOTOR_B_Pin | RIGHT_MOTOR_A_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_BUILTIN_Pin */
    GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LEFT_MOTOR_B_Pin LEFT_MOTOR_A_Pin */
    GPIO_InitStruct.Pin = LEFT_MOTOR_B_Pin | LEFT_MOTOR_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : BUZZER_PIN_Pin RIGHT_MOTOR_B_Pin RIGHT_MOTOR_A_Pin */
    GPIO_InitStruct.Pin = BUZZER_PIN_Pin | RIGHT_MOTOR_B_Pin | RIGHT_MOTOR_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SetColor(unsigned int index, unsigned short r, unsigned short g, unsigned short b)
{
    for (int mask = 0b10000000, i = 0; mask > 0; mask >>= 1, i++)
    {
        if (g & mask)
            WS2812Buffer[index * 24 + i] = WS2812_HIGH_BIT;
        else
            WS2812Buffer[index * 24 + i] = WS2812_LOW_BIT;
        if (r & mask)
            WS2812Buffer[index * 24 + 8 + i] = WS2812_HIGH_BIT;
        else
            WS2812Buffer[index * 24 + 8 + i] = WS2812_LOW_BIT;
        if (b & mask)
            WS2812Buffer[index * 24 + 16 + i] = WS2812_HIGH_BIT;
        else
            WS2812Buffer[index * 24 + 16 + i] = WS2812_LOW_BIT;
    }
}

void SetColorAll(unsigned short r, unsigned short g, unsigned short b)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        SetColor(i, r, g, b);
    }
}

void Rainbow(int length, float phase, float brightness)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        SetColor(i,
                 (unsigned short)(brightness * (cos(2 * M_PI * (phase + (float)i / length)) * 0xff + 0x7f)),
                 (unsigned short)(brightness * (cos(2 * M_PI * (phase + (float)i / length - 1.0 / 3)) * 0xff + 0x7f)),
                 (unsigned short)(brightness * (cos(2 * M_PI * (phase + (float)i / length - 2.0 / 3)) * 0xff + 0x7f)));
    }
}

void SetTailLights(unsigned short r, unsigned short g, unsigned short b)
{
    SetColor(0, r, g, b);
    SetColor(1, r, g, b);
}

void LeftMotor(int a, int b)
{
    HAL_GPIO_WritePin(LEFT_MOTOR_A_GPIO_Port, LEFT_MOTOR_A_Pin, a);
    HAL_GPIO_WritePin(LEFT_MOTOR_B_GPIO_Port, LEFT_MOTOR_B_Pin, b);
}

void RightMotor(int a, int b)
{
    HAL_GPIO_WritePin(RIGHT_MOTOR_A_GPIO_Port, RIGHT_MOTOR_A_Pin, a);
    HAL_GPIO_WritePin(RIGHT_MOTOR_B_GPIO_Port, RIGHT_MOTOR_B_Pin, b);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
#ifdef DEBUGGING
        // sprintf(StringBuffer, "tim2 overflows\r\n");
        // HAL_UART_Transmit(&huart1, StringBuffer, strlen(StringBuffer), 10);
#endif
        if (HAL_GPIO_ReadPin(PULSE_IN_GPIO_Port, PULSE_IN_Pin) == 1)
            Mode = 0;
    }
    if (htim == &htim1)
    {
#ifdef DEBUGGING
        // sprintf(StringBuffer, "tim1 overflows %d\r\n", __HAL_TIM_GetCounter(&htim1));
        // HAL_UART_Transmit(&huart1, StringBuffer, strlen(StringBuffer), 10);
#endif
        SonarOverflowCount++;
        LEDStep += 1;
        switch (Mode)
        {
        case 0:
            SetColorAll(
                (unsigned short)(0.15 * (cos(2 * M_PI * (LEDStep / 255.0)) * 0xff + 0x7f)),
                (unsigned short)(0.15 * (cos(2 * M_PI * (LEDStep / 255.0 - 1.0 / 3)) * 0xff + 0x7f)),
                (unsigned short)(0.15 * (cos(2 * M_PI * (LEDStep / 255.0 - 2.0 / 3)) * 0xff + 0x7f)));
            break;
        case 1:
            SetColor(2,
                     (unsigned short)(0.15 * (cos(2 * M_PI * (LEDStep / 255.0)) * 0xff + 0x7f)),
                     (unsigned short)(0.15 * (cos(2 * M_PI * (LEDStep / 255.0 - 1.0 / 3)) * 0xff + 0x7f)),
                     (unsigned short)(0.15 * (cos(2 * M_PI * (LEDStep / 255.0 - 2.0 / 3)) * 0xff + 0x7f)));
            break;
        default:
            break;
        }
        HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)WS2812Buffer, ARRAYLEN(WS2812Buffer));
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            if (SonarPulsePolarity == 1)
            {
                SonarRisingEdge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
                SonarPulsePolarity = 0;
                SonarOverflowCount = 0;
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
#ifdef DEBUGGING
                HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, 1);
#endif
            }
            else
            {
                SonarFallingEdge = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
                SonarPulse = SonarOverflowCount * 2000 + SonarFallingEdge - SonarRisingEdge;
                SonarPulsePolarity = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
#ifdef DEBUGGING
                HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, 0);
#endif
            }
        }
    }
    if (htim == &htim4)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // falling edge
        {
            TIM4FallingEdge = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
            ReceivedPeriodBuffer = TIM4FallingEdge - TIM4LastFallingEdge;
            TIM4LastFallingEdge = TIM4FallingEdge;

#ifdef DEBUGGING
            // sprintf(StringBuffer, "tim4 ch2 %4d %4d\r\n", ReceivedPeriodBuffer, TIM4FallingEdge);
            // HAL_UART_Transmit(&huart1, StringBuffer, strlen(StringBuffer), 10);
#endif
            Mode = 1;
            __HAL_TIM_SetCounter(&htim2, 0);
        }
    }
}

// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
// {
// }

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
// }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
