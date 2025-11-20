//#include "../../actuators/pwm.h"
//#include "../../krnl/scheduler.h"
//#include "../../communication/usart.h"
//#include "./Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"
//#include "stm32f4xx_hal_tim.h"
//
//TIM_HandleTypeDef htim3;
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(htim->Instance==TIM3)
//  {
//    /* USER CODE BEGIN TIM3_MspPostInit 0 */
//
//    /* USER CODE END TIM3_MspPostInit 0 */
//
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    /**TIM3 GPIO Configuration
//    PB4     ------> TIM3_CH1
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//    /* USER CODE BEGIN TIM3_MspPostInit 1 */
//
//    /* USER CODE END TIM3_MspPostInit 1 */
//  }
//
//}
//
//void Error_Handler() {os_printf("HAL error ! \n");}
//static void MX_TIM3_Init(void)
//{
//
//  /* USER CODE BEGIN TIM3_Init 0 */
//
//  /* USER CODE END TIM3_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM3_Init 1 */
//
//  /* USER CODE END TIM3_Init 1 */
//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 168;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 100;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 50;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM3_Init 2 */
//
//  /* USER CODE END TIM3_Init 2 */
//  HAL_TIM_MspPostInit(&htim3);
//
//}
//
//void PWM_init() {
//    MX_TIM3_Init();
//}
//
////void PWM_start() {
////    //HAL_TIM_Base_Start(&htim3);
////    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
////}
////
////void PWM_set_duty_cycle(float duty_cycle) {
////    int cnt = duty_cycle * 65535;
////    os_printf("Set duty cycle: %d \n", cnt);
////    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, cnt);
////}
//
//void PWM_thread() {
//
//    //sleep(10 * MILLISECONDS);
//    PWM_init();
//    //PWM_start();
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          // Start PWM on TIM1_CH1
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500); // 50% duty cycle
//    while (1) {
//        uint32_t counter = __HAL_TIM_GET_COUNTER(&htim3);
//         os_printf("TIM3 counter = %d\n", (int) counter);
//        //
//        // sleep(2 * SECONDS);
//        // PWM_set_duty_cycle(0.25);
//        // sleep(2 * SECONDS);
//        // PWM_set_duty_cycle(0.5);
//        // sleep(2 * SECONDS);
//        // PWM_set_duty_cycle(0.8);
//    }
//}
