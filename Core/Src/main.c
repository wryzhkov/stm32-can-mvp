/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ROLE_TRANSMITTER 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_last_id = 0;
volatile uint8_t can_last_data[8] = {0};

uint32_t last_tx_time = 0;
uint8_t tx_data[4] = {0xCA, 0xFE, 0x01, 0x02};

CAN_FilterTypeDef sFilterConfig;

#define CAN_LOG_QUEUE_SIZE 16

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CAN_LogFrame;

static CAN_LogFrame can_log_queue[CAN_LOG_QUEUE_SIZE];

static volatile uint8_t can_log_head = 0;
static volatile uint8_t can_log_tail = 0;
static volatile uint32_t can_log_dropped = 0;

volatile uint32_t can_tx_count = 0;

#define CAN_TX_LOG_QUEUE_SIZE 16

static CAN_LogFrame can_tx_log_queue[CAN_TX_LOG_QUEUE_SIZE];

static volatile uint8_t can_tx_log_head = 0;
static volatile uint8_t can_tx_log_tail = 0;
static volatile uint32_t can_tx_log_dropped = 0;

static CAN_LogFrame can_tx_pending[3];
static volatile uint8_t can_tx_pending_valid[3] = {0};
volatile uint32_t can_tx_tsr[3] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN_Log_Process(void);
static void CAN_Status_Report(void);
static void CAN_LogTx_Enqueue(uint8_t mailbox);
static void CAN_LogTx_Process(void);
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

HAL_Delay(2000);

  const char uart_test[] = "Blue Pill UART is alive!\r\n";

HAL_UART_Transmit(&huart1,
                  (uint8_t *)uart_test,
                  sizeof(uart_test) - 1,
                  HAL_MAX_DELAY);

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
      Error_Handler();
  }

  HAL_StatusTypeDef status = HAL_CAN_ActivateNotification(&hcan,    CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);

  if (status != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if CAN_ROLE_TRANSMITTER != 0
    if (HAL_GetTick() - last_tx_time >= 500)
    {
        CAN_TxHeaderTypeDef txHeader;
        uint32_t txMailbox;

        txHeader.StdId = 0x123; // CAN message id
        txHeader.ExtId = 0;
        txHeader.IDE = CAN_ID_STD;
        txHeader.RTR = CAN_RTR_DATA;
        txHeader.DLC = 4; // Payload bytes
        txHeader.TransmitGlobalTime = DISABLE;

        // We check for a free mailbox first because bxCAN has three hardware transmit mailboxes. We don't want to queue another frame if all are occupied.
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        {
            if (HAL_CAN_AddTxMessage(&hcan, &txHeader, tx_data, &txMailbox) == HAL_OK)
            {
                uint8_t mailbox = 0;

                if (txMailbox == CAN_TX_MAILBOX1)
                    mailbox = 1;
                else if (txMailbox == CAN_TX_MAILBOX2)
                    mailbox = 2;

                can_tx_pending[mailbox].id = txHeader.StdId;
                can_tx_pending[mailbox].dlc = txHeader.DLC;

                for (uint8_t i = 0; i < 8; i++)
                    can_tx_pending[mailbox].data[i] = tx_data[i];

                __DMB();
                can_tx_pending_valid[mailbox] = 1;

                tx_data[3]++;

            }
        }

        last_tx_time = HAL_GetTick();
    }
#else


#endif
    CAN_Log_Process();
    CAN_LogTx_Process();
    CAN_Status_Report();

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                             &rxHeader, rxData) != HAL_OK) {
        return;
    }

    can_rx_count++;
    can_last_id = rxHeader.StdId;

    for (uint8_t i = 0; i < 8; i++) {
        can_last_data[i] = rxData[i];
    }

    uint8_t next = (can_log_head + 1) % CAN_LOG_QUEUE_SIZE;

    if (next != can_log_tail) {
        can_log_queue[can_log_head].id = rxHeader.StdId;
        can_log_queue[can_log_head].dlc = rxHeader.DLC;

        for (uint8_t i = 0; i < 8; i++) {
            can_log_queue[can_log_head].data[i] = rxData[i];
        }

        can_log_head = next;
    } else {
        can_log_dropped++;
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
static void CAN_LogTx_Enqueue(uint8_t mailbox)
{
    if (!can_tx_pending_valid[mailbox])
        return;

    uint8_t next = (can_tx_log_head + 1) % CAN_TX_LOG_QUEUE_SIZE;

    if (next == can_tx_log_tail) {
        can_tx_log_dropped++;
        can_tx_pending_valid[mailbox] = 0;
        return;
    }

    can_tx_log_queue[can_tx_log_head] = can_tx_pending[mailbox];

    can_tx_log_head = next;
    can_tx_pending_valid[mailbox] = 0;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can_tx_count++;
    CAN_LogTx_Enqueue(0);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can_tx_count++;
    CAN_LogTx_Enqueue(1);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can_tx_count++;
    CAN_LogTx_Enqueue(2);
}
static void CAN_Log_Process(void)
{
    while (can_log_tail != can_log_head) {
        CAN_LogFrame frame;

        __disable_irq();

        frame = can_log_queue[can_log_tail];
        can_log_tail = (can_log_tail + 1) % CAN_LOG_QUEUE_SIZE;

        __enable_irq();

        char message[80];
        int len = snprintf(message, sizeof(message),
                           "[CAN RX] ID=%03lX DLC=%u DATA=",
                           (unsigned long)frame.id,
                           frame.dlc);

        for (uint8_t i = 0; i < frame.dlc; i++) {
            len += snprintf(message + len, sizeof(message) - len,
                            "%02X%s",
                            frame.data[i],
                            (i + 1 < frame.dlc) ? " " : "");
        }

        len += snprintf(message + len, sizeof(message) - len, "\r\n");

        HAL_UART_Transmit(&huart1, (uint8_t *)message,
                          len, HAL_MAX_DELAY);
    }
}

static void CAN_Status_Report(void)
{
    static uint32_t last_report = 0;

    uint32_t now = HAL_GetTick();

    if ((now - last_report) < 1000)
        return;

    last_report = now;

    uint32_t esr = CAN1->ESR;

    uint32_t tec = (esr >> 24) & 0xFF;
    uint32_t rec = (esr >> 16) & 0xFF;
    uint32_t lec = (esr >> 4) & 0x07;

    char message[196];

    int len = snprintf(message, sizeof(message),
        "[CAN] RX=%lu TX=%lu DROP=%lu "
        "ESR=%08lX TEC=%lu REC=%lu LEC=%lu "
        "MSR=%08lX TSR=%08lX MCR=%08lX"
        " TSR0=%08lX TSR1=%08lX TSR2=%08lX"
        "\r\n",
        (unsigned long)can_rx_count,
        (unsigned long)can_tx_count,
        (unsigned long)can_log_dropped,
        (unsigned long)esr,
        (unsigned long)tec,
        (unsigned long)rec,
        (unsigned long)lec,
        (unsigned long)CAN1->MSR,
        (unsigned long)CAN1->TSR,
        (unsigned long)CAN1->MCR,
        (unsigned long)can_tx_tsr[0],
        (unsigned long)can_tx_tsr[1],
        (unsigned long)can_tx_tsr[2]
      );

    HAL_UART_Transmit(&huart1,
        (uint8_t *)message, len, HAL_MAX_DELAY);
}

static void CAN_LogTx_Process(void)
{
    while (can_tx_log_tail != can_tx_log_head) {
        CAN_LogFrame frame;

        __disable_irq();

        frame = can_tx_log_queue[can_tx_log_tail];
        can_tx_log_tail =
            (can_tx_log_tail + 1) % CAN_TX_LOG_QUEUE_SIZE;

        __enable_irq();

        char message[80];

        int len = snprintf(message, sizeof(message),
            "[CAN TX] ID=%03lX DLC=%u DATA=",
            (unsigned long)frame.id,
            frame.dlc);

        for (uint8_t i = 0; i < frame.dlc; i++) {
            len += snprintf(message + len, sizeof(message) - len,
                "%02X%s",
                frame.data[i],
                (i + 1 < frame.dlc) ? " " : "");
        }

        len += snprintf(message + len, sizeof(message) - len, "\r\n");

        HAL_UART_Transmit(&huart1,
            (uint8_t *)message, len, HAL_MAX_DELAY);
    }
}
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
