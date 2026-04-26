/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define BMI088_INT1_Pin GPIO_PIN_10
#define BMI088_INT1_GPIO_Port GPIOE
#define BMI088_INT1_EXTI_IRQn EXTI15_10_IRQn
#define BMI088_INT3_Pin GPIO_PIN_12
#define BMI088_INT3_GPIO_Port GPIOE
#define BMI088_INT3_EXTI_IRQn EXTI15_10_IRQn
#define VALVE_CHASSIS_2_Pin GPIO_PIN_13
#define VALVE_CHASSIS_2_GPIO_Port GPIOE
#define VALVE_CHASSIS_1_Pin GPIO_PIN_14
#define VALVE_CHASSIS_1_GPIO_Port GPIOE
#define RS485_REDE2_Pin GPIO_PIN_14
#define RS485_REDE2_GPIO_Port GPIOB
#define PUMP_CHASSIS_Pin GPIO_PIN_14
#define PUMP_CHASSIS_GPIO_Port GPIOD
#define PUMP_ARM_Pin GPIO_PIN_15
#define PUMP_ARM_GPIO_Port GPIOD
#define RS485_REDE1_Pin GPIO_PIN_4
#define RS485_REDE1_GPIO_Port GPIOD
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOE
#define CSN_Pin GPIO_PIN_1
#define CSN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
