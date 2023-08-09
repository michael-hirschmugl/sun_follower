# sun_follower
Digital compass controlled sun follower for PV modules.

## Modifications
- Added `HAL_TIM_Base_Start_IT(&htim16);` at `/* USER CODE BEGIN 2 */`. This ensures, that the timer 16 will generate an interrupt. Timer 16 is a general purpose timer. It is set to toggle at 1 sec intervals.
- Added `void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)` at `/* USER CODE BEGIN 4 */`. This is the interrupt service routine for the timer interrupt. It cointains the GPIO toggle command.