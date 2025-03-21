# 6AxisWurth
save perso pour des tests

### Pour la première initialisation :
Dans `stm32l4xx_it.c` décommenter pour le premier lancement le `Init_HighPerf_Mode_6_axis();` pour avoir :
```c
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  Init_HighPerf_Mode_6_axis(); //à appeler au premier lancement
  Display_6_axis_data();
  /* USER CODE END TIM2_IRQn 1 */
}
```

Problème réglé -> due à l'initialisation de `Init_HighPerf_Mode_6_axis();` avant l'initialisation de l'i2c.
