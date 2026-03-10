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
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}


#define SENSOR_BUS hi2c1

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
#define    CNT_FOR_OUTPUT       10

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


static   stmdev_ctx_t dev_ctx;
volatile  uint8_t thread_wake = 0;

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

static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = lsm6dsv320x_from_f16_to_f32(h);
    return conv.ret;
}

static void sflp2q(float_t quat[4], uint16_t sflp[3])
{
  float_t sumsq = 0;

  quat[0] = npy_half_to_float(sflp[0]);
  quat[1] = npy_half_to_float(sflp[1]);
  quat[2] = npy_half_to_float(sflp[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f) {
    float_t n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT1_Pin) {
    thread_wake = 1;
  }
}

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
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("HELLO!\n");
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SA0_GPIO_Port, SA0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	


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
  printf("WHO_AM_I=0x%02X (expect 0x%02X)\r\n", whoamI, LSM6DSV320X_ID);
  if (whoamI != LSM6DSV320X_ID) {
    printf("ERROR: device id mismatch!\r\n");
    while (1) { }
  }

  /* Perform device power-on-reset */
  lsm6dsv320x_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  lsm6dsv320x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv320x_xl_setup(&dev_ctx, LSM6DSV320X_ODR_AT_120Hz, LSM6DSV320X_XL_HIGH_PERFORMANCE_MD);
  lsm6dsv320x_gy_setup(&dev_ctx, LSM6DSV320X_ODR_AT_120Hz, LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
	lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_480Hz, 1);
	lsm6dsv320x_gy_setup(&dev_ctx,LSM6DSV320X_ODR_AT_120Hz,LSM6DSV320X_GY_HIGH_PERFORMANCE_MD);
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


  /* --------- SFLP: 120Hz + Game rotation --------- */
  lsm6dsv320x_sflp_data_rate_set(&dev_ctx, LSM6DSV320X_SFLP_120Hz);
  lsm6dsv320x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

  /* 推荐：写入 gbias 初值（与你能跑通版本一致） */
  lsm6dsv320x_sflp_gbias_t gbias = {0};
  gbias.gbias_x = 0.0f;
  gbias.gbias_y = 0.0f;
  gbias.gbias_z = 0.0f;
  lsm6dsv320x_sflp_game_gbias_set(&dev_ctx, &gbias);

  /* --------- INT1 route: 用 DRDY_XL 做唤醒（你现在中断链路已通） --------- */
  lsm6dsv320x_pin_int1_route_t pin_int1 = {0};
  pin_int1.drdy_xl = PROPERTY_ENABLE;
  lsm6dsv320x_pin_int1_route_set(&dev_ctx, &pin_int1);
	
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;		
	
	uint8_t data_angular_rate_raw[16]={0};
	data_angular_rate_raw[0]=0xAB;//帧头
	data_angular_rate_raw[1]=0xFD;//源地址
	data_angular_rate_raw[2]=0xFE;//目标地址		
	data_angular_rate_raw[3]=0x03;//功能码ID	
	data_angular_rate_raw[4]=0x08;//数据长度LEN
	data_angular_rate_raw[5]=0x00;//数据长度LEN 8
	data_angular_rate_raw[6]=0x01;//mode = 1	

	data_angular_rate_raw[13]=0x00;//FUSION _STA：融合状态
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		  if (thread_wake) {
      thread_wake = 0;
      /* 关键：必须先确认 XL new data（你验证过必须这样才有数据） */
      lsm6dsv320x_data_ready_t status;
      lsm6dsv320x_flag_data_ready_get(&dev_ctx, &status);
      if (status.drdy_xl) {

        /* 建议：读一次 XL 输出，等价“消费/清源”，保证下一次中断继续来 */
        lsm6dsv320x_acceleration_raw_get(&dev_ctx, data_raw_motion);

        /* 如果你希望再稳一点，可以加 SFLP ready 门控 */
        lsm6dsv320x_all_sources_t all_sources;
        lsm6dsv320x_all_sources_get(&dev_ctx, &all_sources);
        if (!all_sources.emb_func_stand_by) {
          continue;
        }

        /* 读 SFLP 四元数 */
        lsm6dsv320x_quaternion_t q_st;
        lsm6dsv320x_sflp_quaternion_get(&dev_ctx, &q_st);

        /* 轴变换：保持你“能跑通欧拉角”工程的映射 */
        quaternion_t q;
        q.quat_w =  q_st.quat_w;
        q.quat_x = -q_st.quat_y;
        q.quat_y =  q_st.quat_z;
        q.quat_z = -q_st.quat_x;

        euler_angle_t e;
        quaternion_to_euler_angle(&q, &e);

        /* 可选：Yaw 显示到 [-180, 180] */
        float yaw_print = e.yaw;
        if (yaw_print > 180.0f) {
          yaw_print -= 360.0f;
        }

//        printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n",
//               e.roll, e.pitch, yaw_print);
				
			// 定义用于存放欧拉角的整数变量（放大100倍，用于发送）
			int16_t	Roll_int16;
			int16_t	Pitch_int16;					
			int16_t	Yaw_int16;	
			// 将浮点欧拉角转成定点整数，单位：度 × 100
			Roll_int16 = (int16_t)((e.roll) * 100);
			Pitch_int16 = (int16_t)((e.pitch) * 100);
			Yaw_int16 = (int16_t)((e.yaw) * 100 - 18000);
			
//			printf("2:Roll=%d,Pitch=%d,Yaw=%d \n",
//											Roll_int16,Pitch_int16,Yaw_int16);	
			
			
			data_angular_rate_raw[7] = (uint8_t)(Roll_int16 >> 8);//roll
			data_angular_rate_raw[8] = (uint8_t)(Roll_int16 );
			data_angular_rate_raw[9] = (uint8_t)(Pitch_int16 >> 8);//pitch
			data_angular_rate_raw[10] = (uint8_t)(Pitch_int16 );
			data_angular_rate_raw[11] = (uint8_t)(Yaw_int16 >> 8);//yaw
			data_angular_rate_raw[12] = (uint8_t)(Yaw_int16 );
								
			data_angular_rate_raw[13] = 0;
			sumcheck = 0;
			addcheck = 0;
			for(uint16_t i=0; i < 14; i++)
			{
				sumcheck += data_angular_rate_raw[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
				addcheck += sumcheck; //每一字节的求和操作，进行一次 sumcheck 的累加
			}
			data_angular_rate_raw[14]=sumcheck;
			data_angular_rate_raw[15]=addcheck;	
			
			HAL_UART_Transmit(&huart1 , (uint8_t *)&data_angular_rate_raw, 16, 0xFFFF);				

      }
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
