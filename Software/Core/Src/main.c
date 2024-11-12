/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "6axis.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void check_device_communication();
//void Init_HighPerf_Mode_6_axis(void);
//HAL_StatusTypeDef Read_sensor_data();
//void Display_6_axis_data(void);
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&huart2);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

//#define SENSOR_ADDRESS  0x6B << 1
//
//void check_device_communication() {
//    uint8_t deviceID = 0;
//    HAL_StatusTypeDef status;
//
//    status = HAL_I2C_Mem_Read(&hi2c1, (uint8_t)(SENSOR_ADDRESS), 0x0F, I2C_MEMADD_SIZE_8BIT, &deviceID, 1, HAL_MAX_DELAY);
//
//    if (status == HAL_OK) {
//        printf("Communication réussie.\r\n");
//    } else {
//        printf("Échec de la communication.\r\n");
//    }
//}
//
//#define CTRL1_XL 0x10
//#define CTRL2_G  0x11
//#define CTRL3_C  0x12
//#define CTRL6_C  0x15
//#define CTRL7_G  0x16
//#define CTRL8_XL 0x17
//#define STATUS_REG 0x1E

// Fonction d'initialisation du capteur en mode haute performance
//void Init_HighPerf_Mode_6_axis(void)
//{
//    uint8_t data = 0;
//
//    // Activer le mode haute performance pour l'accéléromètre et le gyroscope
//    data = 0x54; // 208 Hz, ±16g pour l'accéléromètre
//    HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDRESS, CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//    data = 0x4C; // 208 Hz, ±2000 dps pour le gyroscope
//    HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDRESS, CTRL2_G, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//    // Activer l'incrémentation automatique des adresses et l'update des données
//    data = 0x00; // Incrémentation automatique activée, BDU activé
//    HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDRESS, CTRL3_C, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//    // Configurer la bande passante et autres options
//    data = 0x00; // Paramètre par défaut pour CTRL6_C
//    HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDRESS, CTRL6_C, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//    // Configuration supplémentaire du gyroscope
//    data = 0x00; // Paramètre par défaut pour CTRL7_G
//    HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDRESS, CTRL7_G, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//    // Configuration supplémentaire pour l'accéléromètre
//    data = 0x00; // Paramètre par défaut pour CTRL8_XL
//    HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDRESS, CTRL8_XL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//    uint8_t status = 0;
//
//        // Lire le registre de statut via STATUS_REG pour voir si les données sont prêtes (XLDA et GDA)
//        HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDRESS, STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
//
//        if (status & 0x01) {
//            printf("Les données de l'accéléromètre sont prêtes. q\r\n");
//        }
//        if (status & 0x02) {
//            printf("Les données du gyroscope sont prêtes.\r\n");
//        }
//
//}
//
//#define G_X_OUT_L 0x22             // Adresse du premier registre de sortie gyroscope (G_)
//#define XL_X_OUT_L 0x28            // Adresse du premier registre de sortie accéléromètre (XL_)
//
//
//// Sensibilités pour accéléromètre et gyroscope
//#define ACC_SENSITIVITY_16G 0.488 // Sensibilité pour ±16g
//#define GYRO_SENSITIVITY_2000DPS 70 // Sensibilité pour ±2000 dps
//
//HAL_StatusTypeDef Read_sensor_data(int16_t* accel_data, int16_t* gyro_data) {
//    HAL_StatusTypeDef status;
//    uint8_t status_reg;
//    uint8_t low_byte, high_byte;
//
//    // Étape 1 : Lire STATUS_REG pour vérifier XLDA et GDA
//    status = HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDRESS, STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, HAL_MAX_DELAY);
//    if (status != HAL_OK) return status;
//
//    // Vérifier si les bits XLDA (bit 0) et GDA (bit 1) sont à 1
//    if (!(status_reg & 0x01) || !(status_reg & 0x02)) {
//        // Pas de nouvelles données prêtes
//        return HAL_ERROR;
//    }
//	// Lire les données du gyroscope
//	for (int i = 0; i < 3; i++) {
//		// Adresses des registres pour chaque axe
//		uint8_t low_addr = G_X_OUT_L + i * 2;
//		uint8_t high_addr = low_addr + 1;
//
//		// Lire l'octet bas
//		status = HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDRESS, low_addr, I2C_MEMADD_SIZE_8BIT, &low_byte, 1, HAL_MAX_DELAY);
//		if (status != HAL_OK) return status;
//
//		// Lire l'octet haut
//		status = HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDRESS, high_addr, I2C_MEMADD_SIZE_8BIT, &high_byte, 1, HAL_MAX_DELAY);
//		if (status != HAL_OK) return status;
//
//		// Combiner les octets pour obtenir la valeur 16 bits
//		gyro_data[i] = (int16_t)((high_byte << 8) | low_byte);
//	}
//	// Lire les données de l'accéléromètre
//	for (int i = 0; i < 3; i++) {
//		// Adresses des registres pour chaque axe
//		uint8_t low_addr = XL_X_OUT_L + i * 2;
//		uint8_t high_addr = low_addr + 1;
//
//		// Lire l'octet bas
//		status = HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDRESS, low_addr, I2C_MEMADD_SIZE_8BIT, &low_byte, 1, HAL_MAX_DELAY);
//		if (status != HAL_OK) return status;
//
//		// Lire l'octet haut
//		status = HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDRESS, high_addr, I2C_MEMADD_SIZE_8BIT, &high_byte, 1, HAL_MAX_DELAY);
//		if (status != HAL_OK) return status;
//
//		// Combiner les octets pour obtenir la valeur 16 bits
//		accel_data[i] = (int16_t)((high_byte << 8) | low_byte);
//	}
//
//	return HAL_OK;
//}
//
//void Display_6_axis_data(void) {
//    int16_t accel_data[3], gyro_data[3]; // Tableau pour stocker les données d'accéléromètre et de gyroscope
//    HAL_StatusTypeDef status;
//
//    // Lire les données du capteur
//    status = Read_sensor_data(accel_data, gyro_data);
//
//    if (status == HAL_OK) {
//        // Afficher les valeurs dans le format demandé
//        printf("XL/(X,Y,Z) : %d ; %d ; %d\r\n", accel_data[0], accel_data[1], accel_data[2]);
//        printf("G/(X,Y,Z) : %d ; %d ; %d\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
//    } else {
//        printf("Erreur de lecture du capteur --> ");
//        check_device_communication();
//    }
//}

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
  check_device_communication();
  Init_HighPerf_Mode_6_axis();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Vérifier la communication avec le capteur
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //check_device_communication();
	  //Init_HighPerf_Mode_6_axis();
	  Display_6_axis_data();

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
