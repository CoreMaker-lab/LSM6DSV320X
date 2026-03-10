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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

#define SENSOR_BUS hi2c1


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
#define    FIFO_WATERMARK       128
#define    CNT_FOR_OUTPUT       100

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;
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

static   stmdev_ctx_t dev_ctx;
static double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3];
static uint16_t lowg_xl_cnt = 0, hg_xl_cnt = 0, gyro_cnt = 0;

static uint16_t gravity_cnt = 0, gbias_cnt = 0, rot_cnt = 0;
static float_t gravity_sum[3], gbias_sum[3];
static double_t rot_sum[4];

static   uint8_t fifo_thread_run = 0;

static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = lsm6dsv320x_from_f16_to_f32(h);
    return conv.ret;
}


typedef struct
{
  float yaw;
  float pitch;
  float roll;
} euler_angle_t;

typedef struct
{
  float quat_w;
  float quat_x;
  float quat_y;
  float quat_z;
} quaternion_t;




void quaternion_to_euler_angle(quaternion_t *q,euler_angle_t *euler)
{
		if (q->quat_w < 0.0f) {
			q->quat_x *= -1.0f;
			q->quat_y *= -1.0f;
			q->quat_z *= -1.0f;
			q->quat_w *= -1.0f;
		}

		float sqx = q->quat_x * q->quat_x;
		float sqy = q->quat_y * q->quat_y;
		float sqz = q->quat_z * q->quat_z;

		euler->yaw = -atan2f(2.0f * (q->quat_y * q->quat_w + q->quat_x * q->quat_z), 1.0f - 2.0f * (sqy + sqx));
		euler->pitch = -atan2f(2.0f * (q->quat_x * q->quat_y + q->quat_z * q->quat_w), 1.0f - 2.0f * (sqx + sqz));
		euler->roll = -asinf(2.0f * (q->quat_x * q->quat_w - q->quat_y * q->quat_z));

		if (euler->yaw < 0.0f)
		{
			euler->yaw += 2.0f * 3.1415926;
		}	
		
		euler->yaw = 57.29578 * euler->yaw;
		euler->pitch = 57.29578 * euler->pitch;
		euler->roll = 57.29578 * euler->roll;
}


static void lsm6dsv320x_fifo_thread(void)
{
  float_t acceleration_mg[3];
  float_t angular_rate_mdps[3];
  float_t gravity_mg[3], gbias_mdps[3], quat[4];
  uint8_t *axis;

  if (fifo_thread_run) {
    lsm6dsv320x_fifo_status_t fifo_status;
    uint16_t num;

    fifo_thread_run = 0;

    /* Read watermark flag */
    lsm6dsv320x_fifo_status_get(&dev_ctx, &fifo_status);

    num = fifo_status.fifo_level;

    while (num--) {
      lsm6dsv320x_fifo_out_raw_t f_data;

      /* Read FIFO sensor value */
      lsm6dsv320x_fifo_out_raw_get(&dev_ctx, &f_data);
      datax = (int16_t *)&f_data.data[0];
      datay = (int16_t *)&f_data.data[2];
      dataz = (int16_t *)&f_data.data[4];
      ts = (int32_t *)&f_data.data[0];

      switch (f_data.tag) {
      case LSM6DSV320X_XL_NC_TAG:
        /* Read acceleration field data */
        acceleration_mg[0] = lsm6dsv320x_from_fs2_to_mg(*datax);
        acceleration_mg[1] = lsm6dsv320x_from_fs2_to_mg(*datay);
        acceleration_mg[2] = lsm6dsv320x_from_fs2_to_mg(*dataz);

        lowg_xl_sum[0] += acceleration_mg[0];
        lowg_xl_sum[1] += acceleration_mg[1];
        lowg_xl_sum[2] += acceleration_mg[2];
        lowg_xl_cnt++;
        break;

      case LSM6DSV320X_XL_HG_TAG:
        acceleration_mg[0] = lsm6dsv320x_from_fs320_to_mg(*datax);
        acceleration_mg[1] = lsm6dsv320x_from_fs320_to_mg(*datay);
        acceleration_mg[2] = lsm6dsv320x_from_fs320_to_mg(*dataz);

        hg_xl_sum[0] += acceleration_mg[0];
        hg_xl_sum[1] += acceleration_mg[1];
        hg_xl_sum[2] += acceleration_mg[2];
        hg_xl_cnt++;
        break;

      case LSM6DSV320X_GY_NC_TAG:
        angular_rate_mdps[0] = lsm6dsv320x_from_fs2000_to_mdps(*datax);
        angular_rate_mdps[1] = lsm6dsv320x_from_fs2000_to_mdps(*datay);
        angular_rate_mdps[2] = lsm6dsv320x_from_fs2000_to_mdps(*dataz);

        gyro_sum[0] += angular_rate_mdps[0];
        gyro_sum[1] += angular_rate_mdps[1];
        gyro_sum[2] += angular_rate_mdps[2];
        gyro_cnt++;
        break;

      case LSM6DSV320X_SFLP_GYROSCOPE_BIAS_TAG:
        axis = &f_data.data[0];
        gbias_mdps[0] = lsm6dsv320x_from_fs125_to_mdps(axis[0] | (axis[1] << 8));
        gbias_mdps[1] = lsm6dsv320x_from_fs125_to_mdps(axis[2] | (axis[3] << 8));
        gbias_mdps[2] = lsm6dsv320x_from_fs125_to_mdps(axis[4] | (axis[5] << 8));

        gbias_sum[0] += gbias_mdps[0];
        gbias_sum[1] += gbias_mdps[1];
        gbias_sum[2] += gbias_mdps[2];
        gbias_cnt++;
        break;

      case LSM6DSV320X_SFLP_GRAVITY_VECTOR_TAG:
        axis = &f_data.data[0];
        gravity_mg[0] = lsm6dsv320x_from_sflp_to_mg(axis[0] | (axis[1] << 8));
        gravity_mg[1] = lsm6dsv320x_from_sflp_to_mg(axis[2] | (axis[3] << 8));
        gravity_mg[2] = lsm6dsv320x_from_sflp_to_mg(axis[4] | (axis[5] << 8));

        gravity_sum[0] += gravity_mg[0];
        gravity_sum[1] += gravity_mg[1];
        gravity_sum[2] += gravity_mg[2];
        gravity_cnt++;
        break;

      case LSM6DSV320X_SFLP_GAME_ROTATION_VECTOR_TAG:
      {
        uint16_t *sflp = (uint16_t *)&f_data.data[2];

        if (f_data.data[0] == 0x00) {
          /* Game Rotation first word */
          quat[0] = lsm6dsv320x_from_quaternion_lsb_to_float(sflp[0]);
          quat[1] = lsm6dsv320x_from_quaternion_lsb_to_float(sflp[1]);
        } else if (f_data.data[0] == 0x01) {
          /* Game Rotation second word */
          quat[2] = lsm6dsv320x_from_quaternion_lsb_to_float(sflp[0]);
          quat[3] = lsm6dsv320x_from_quaternion_lsb_to_float(sflp[1]);

          rot_sum[0] += quat[0];
          rot_sum[1] += quat[1];
          rot_sum[2] += quat[2];
          rot_sum[3] += quat[3];

          rot_cnt++;
        } else {
          /* error */
          printf("[%02x - %02x] wrong word \r\n", f_data.data[0], f_data.data[1]);
       }

        break;
      }

      case LSM6DSV320X_TIMESTAMP_TAG:
        printf("TIMESTAMP [ms] %d\r\n", *ts);

        /* print avg low-g xl data */
        if (lowg_xl_cnt > 0) {
          acceleration_mg[0] = lowg_xl_sum[0] / lowg_xl_cnt;
          acceleration_mg[1] = lowg_xl_sum[1] / lowg_xl_cnt;
          acceleration_mg[2] = lowg_xl_sum[2] / lowg_xl_cnt;

          printf("lg xl (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  lowg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
          lowg_xl_cnt = 0;
        }

        /* print avg high-g xl data */
        if (hg_xl_cnt > 0) {
          acceleration_mg[0] = hg_xl_sum[0] / hg_xl_cnt;
          acceleration_mg[1] = hg_xl_sum[1] / hg_xl_cnt;
          acceleration_mg[2] = hg_xl_sum[2] / hg_xl_cnt;

          printf("hg xl (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  hg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
          hg_xl_cnt = 0;
        }

        /* print avg gyro data */
        if (gyro_cnt > 0) {
          angular_rate_mdps[0] = gyro_sum[0] / gyro_cnt;
          angular_rate_mdps[1] = gyro_sum[1] / gyro_cnt;
          angular_rate_mdps[2] = gyro_sum[2] / gyro_cnt;

          printf("gyro (avg of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                  gyro_cnt, angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
          gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
          gyro_cnt = 0;
        }

         /* print SFLP gbias data */
        if (gbias_cnt > 0) {
          gbias_mdps[0] = gbias_sum[0] / gbias_cnt;
          gbias_mdps[1] = gbias_sum[1] / gbias_cnt;
          gbias_mdps[2] = gbias_sum[2] / gbias_cnt;

          printf("SFLP gbias (avg of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                  gbias_cnt, gbias_mdps[0], gbias_mdps[1], gbias_mdps[2]);
          gbias_sum[0] = gbias_sum[1] = gbias_sum[2] = 0.0;
          gbias_cnt = 0;
        }

         /* print SFLP gravity data */
        if (gravity_cnt > 0) {
          gravity_mg[0] = gravity_sum[0] / gravity_cnt;
          gravity_mg[1] = gravity_sum[1] / gravity_cnt;
          gravity_mg[2] = gravity_sum[2] / gravity_cnt;

          printf("SFLP gravity (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  gravity_cnt, gravity_mg[0], gravity_mg[1], gravity_mg[2]);
          gravity_sum[0] = gravity_sum[1] = gravity_sum[2] = 0.0;
          gravity_cnt = 0;
        }

         /* print SFLP game rotation data */
        if (rot_cnt > 0) {
          quat[0] = rot_sum[0] / rot_cnt;
          quat[1] = rot_sum[1] / rot_cnt;
          quat[2] = rot_sum[2] / rot_cnt;
          quat[3] = rot_sum[3] / rot_cnt;

          printf("SFLP rotation (avg of %d samples) [quaternions]:X: %2.3f\tY: %2.3f\tZ: %2.3f\tW: %2.3f\r\n\r\n",
                  rot_cnt, quat[0], quat[1], quat[2], quat[3]);
          rot_sum[0] = rot_sum[1] = rot_sum[2] = rot_sum[3] = 0.0;
          rot_cnt = 0;
					
		

        /* 轴变换：保持你“能跑通欧拉角”工程的映射 */
        quaternion_t q;
        q.quat_w =  quat[3];
        q.quat_x = -quat[1];
        q.quat_y =  quat[2];
        q.quat_z = -quat[0];

        euler_angle_t e;
        quaternion_to_euler_angle(&q, &e);

        /* 可选：Yaw 显示到 [-180, 180] */
        float yaw_print = e.yaw;
        if (yaw_print > 180.0f) {
          yaw_print -= 360.0f;
        }

        printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n",
               e.roll, e.pitch, yaw_print);


					
					
					
        }

        break;

      default:
        printf("[%02x] UNHANDLED TAG \r\n", f_data.tag);
        break;
      }
    }
  }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT1_Pin) {
    fifo_thread_run = 1;
  }
}
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
	
	
	
	
  lsm6dsv320x_pin_int1_route_t pin_int = { 0 };

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

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsv320x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /* Set FIFO batch XL/Gyro ODR */
  lsm6dsv320x_fifo_xl_batch_set(&dev_ctx, LSM6DSV320X_XL_BATCHED_AT_60Hz);
  lsm6dsv320x_fifo_hg_xl_batch_set(&dev_ctx, 1);
  lsm6dsv320x_fifo_gy_batch_set(&dev_ctx, LSM6DSV320X_GY_BATCHED_AT_120Hz);

  /* Set FIFO batch SFLP */
  lsm6dsv320x_sflp_data_rate_set(&dev_ctx, LSM6DSV320X_SFLP_120Hz);

  lsm6dsv320x_fifo_sflp_raw_t sflp =
        {
          .game_rotation = 1,
          .gravity = 1,
          .gbias = 1,
        };
  lsm6dsv320x_fifo_sflp_batch_set(&dev_ctx, sflp);

  lsm6dsv320x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv320x_fifo_mode_set(&dev_ctx, LSM6DSV320X_STREAM_MODE);
  lsm6dsv320x_fifo_timestamp_batch_set(&dev_ctx, LSM6DSV320X_TMSTMP_DEC_32);
  lsm6dsv320x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv320x_xl_setup(&dev_ctx, LSM6DSV320X_ODR_AT_60Hz, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
  lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_480Hz, 0);
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

  /* enable fifo_th on High-G XL (sensor at highest frequency) */
  pin_int.fifo_th = PROPERTY_ENABLE;
  lsm6dsv320x_pin_int1_route_set(&dev_ctx, &pin_int);
  //lsm6dsv320x_pin_int2_route_set(&dev_ctx, &pin_int);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		lsm6dsv320x_fifo_thread();
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
