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
#include "i2c.h"
#include "icache.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lsm6dsv320x_reg.h"
#include <string.h>
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
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SENSOR_BUS hi2c1


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
#define    CNT_FOR_OUTPUT       100

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_motion[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static lsm6dsv320x_filt_settling_mask_t filt_settling_mask;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);




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
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	printf("HELLO!\n");
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SA0_GPIO_Port, SA0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	
	
  stmdev_ctx_t dev_ctx;
  double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3], temp_sum;
  uint16_t lowg_xl_cnt = 0, hg_xl_cnt = 0, gyro_cnt = 0, temp_cnt = 0;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
//  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);
	printf("LSM6DSV320X_ID=0x%x,whoamI=0x%x",LSM6DSV320X_ID,whoamI);
  if (whoamI != LSM6DSV320X_ID)
    while (1);

  /* Perform device power-on-reset */
  lsm6dsv320x_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  lsm6dsv320x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv320x_xl_setup(&dev_ctx, LSM6DSV320X_ODR_AT_60Hz, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
  lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_960Hz, 1);
  lsm6dsv320x_gy_setup(&dev_ctx, LSM6DSV320X_ODR_AT_120Hz, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);

  /* Set full scale */
  lsm6dsv320x_xl_full_scale_set(&dev_ctx, LSM6DSV320X_2g);
  lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g);
  lsm6dsv320x_gy_full_scale_set(&dev_ctx, LSM6DSV320X_2000dps);

  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv320x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv320x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV320X_GY_ULTRA_LIGHT);
  lsm6dsv320x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv320x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV320X_XL_STRONG);

  lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
  hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
  gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
  temp_sum = 0.0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		    lsm6dsv320x_data_ready_t drdy;

    /* Read output only if new xl value is available */
    lsm6dsv320x_flag_data_ready_get(&dev_ctx, &drdy);

    if (drdy.drdy_xl) {
      /* Read acceleration field data */
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      lsm6dsv320x_acceleration_raw_get(&dev_ctx, data_raw_motion);
      acceleration_mg[0] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[0]);
      acceleration_mg[1] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[1]);
      acceleration_mg[2] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[2]);

      lowg_xl_sum[0] += acceleration_mg[0];
      lowg_xl_sum[1] += acceleration_mg[1];
      lowg_xl_sum[2] += acceleration_mg[2];
      lowg_xl_cnt++;
    }

    if (drdy.drdy_hgxl) {
      /* Read acceleration field data */
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      lsm6dsv320x_hg_acceleration_raw_get(&dev_ctx, data_raw_motion);
      acceleration_mg[0] = lsm6dsv320x_from_fs256_to_mg(data_raw_motion[0]);
      acceleration_mg[1] = lsm6dsv320x_from_fs256_to_mg(data_raw_motion[1]);
      acceleration_mg[2] = lsm6dsv320x_from_fs256_to_mg(data_raw_motion[2]);

      hg_xl_sum[0] += acceleration_mg[0];
      hg_xl_sum[1] += acceleration_mg[1];
      hg_xl_sum[2] += acceleration_mg[2];
      hg_xl_cnt++;
    }

    /* Read output only if new xl value is available */
    if (drdy.drdy_gy) {
      /* Read angular rate field data */
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      lsm6dsv320x_angular_rate_raw_get(&dev_ctx, data_raw_motion);
      angular_rate_mdps[0] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[0]);
      angular_rate_mdps[1] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[1]);
      angular_rate_mdps[2] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[2]);

      gyro_sum[0] += angular_rate_mdps[0];
      gyro_sum[1] += angular_rate_mdps[1];
      gyro_sum[2] += angular_rate_mdps[2];
      gyro_cnt++;
    }

    if (drdy.drdy_temp) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm6dsv320x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lsm6dsv320x_from_lsb_to_celsius(data_raw_temperature);
      temp_sum += temperature_degC;
      temp_cnt++;
    }

    if (lowg_xl_cnt >= CNT_FOR_OUTPUT) {
      /* print avg low-g xl data */
      acceleration_mg[0] = lowg_xl_sum[0] / lowg_xl_cnt;
      acceleration_mg[1] = lowg_xl_sum[1] / lowg_xl_cnt;
      acceleration_mg[2] = lowg_xl_sum[2] / lowg_xl_cnt;

      printf("lg xl (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              lowg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
      lowg_xl_cnt = 0;

      /* print avg high-g xl data */
      acceleration_mg[0] = hg_xl_sum[0] / hg_xl_cnt;
      acceleration_mg[1] = hg_xl_sum[1] / hg_xl_cnt;
      acceleration_mg[2] = hg_xl_sum[2] / hg_xl_cnt;

      printf("hg xl (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              hg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
      hg_xl_cnt = 0;

      /* print avg gyro data */
      angular_rate_mdps[0] = gyro_sum[0] / gyro_cnt;
      angular_rate_mdps[1] = gyro_sum[1] / gyro_cnt;
      angular_rate_mdps[2] = gyro_sum[2] / gyro_cnt;

      printf("gyro (avg of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              gyro_cnt, angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
      gyro_cnt = 0;

      /* print avg temperature data */
      temperature_degC = temp_sum / temp_cnt;
      printf("Temperature (avg of %d samples) [degC]:%6.2f\r\n\r\n",
              temp_cnt, temperature_degC);
      temp_cnt = 0;
      temp_sum = 0.0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSV320X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSV320X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}



/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
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
