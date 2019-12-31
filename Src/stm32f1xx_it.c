/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_MODO_OPER 2                // kte p/ testar val max modo_oper
#define MAX_MODO_RELOG 3                // kte p/ testar val max modo_oper
#define DT_DEBOUNCING 250              // tempo delay para debouncing

#define MAX_MM 60
#define MAX_HH 24
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t modo_oper = 0;       // VAR modo_oper LOCAL
volatile uint32_t tIN_IRQ1 = 0;        // tempo entrada na última IRQ6
volatile uint32_t tIN_IRQ2 = 0;        // tempo entrada na última IRQ6
volatile uint32_t tIN_IRQ3 = 0;        // tempo entrada na última IRQ6

volatile uint8_t modo_relogio = 0;
volatile uint8_t setup_hh = 0;
volatile uint8_t setup_mm = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void) {
	/* USER CODE BEGIN EXTI1_IRQn 0 */

	if ((HAL_GetTick() - tIN_IRQ1) > DT_DEBOUNCING) {
		tIN_IRQ1 = HAL_GetTick();                // tIN (ms) da última IRQ1
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0 && modo_relogio == 0) {
			++modo_oper;                          // incrementa modo operação
			if (modo_oper > MAX_MODO_OPER)
				modo_oper = 0;                     // se >MAX voltar modo_oper=0
		}
	}
	/* USER CODE END EXTI1_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	/* USER CODE BEGIN EXTI1_IRQn 1 */

	/* USER CODE END EXTI1_IRQn 1 */
}

/**
 * @brief This function handles EXTI line2 interrupt.
 */
void EXTI2_IRQHandler(void) {
	/* USER CODE BEGIN EXTI2_IRQn 0 */
	if ((HAL_GetTick() - tIN_IRQ2) > DT_DEBOUNCING) {
		tIN_IRQ2 = HAL_GetTick();                // tIN (ms) da última IRQ1
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0 && modo_oper == 0) {
			++modo_relogio;                          // incrementa modo operação
			if (modo_relogio > MAX_MODO_RELOG)
				modo_relogio = 0;                  // se >MAX voltar modo_oper=0
		}
	}
	/* USER CODE END EXTI2_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
	/* USER CODE BEGIN EXTI2_IRQn 1 */

	/* USER CODE END EXTI2_IRQn 1 */
}

/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler(void) {
	/* USER CODE BEGIN EXTI3_IRQn 0 */
	if ((HAL_GetTick() - tIN_IRQ3) > DT_DEBOUNCING) {
		tIN_IRQ3 = HAL_GetTick();                // tIN (ms) da última IRQ1
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0 && modo_relogio == 1) {
			++setup_hh;                          // incrementa modo operação
			if (setup_hh >= MAX_HH)
				setup_hh = 0;                     // se >MAX voltar modo_oper=0
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0 && modo_relogio == 2) {
			++setup_mm;                          // incrementa modo operação
			if (setup_mm >= MAX_MM)
				setup_mm = 0;                     // se >MAX voltar modo_oper=0
		}
	}
	/* USER CODE END EXTI3_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
	/* USER CODE BEGIN EXTI3_IRQn 1 */

	/* USER CODE END EXTI3_IRQn 1 */
}

/**
 * @brief This function handles ADC1 and ADC2 global interrupts.
 */
void ADC1_2_IRQHandler(void) {
	/* USER CODE BEGIN ADC1_2_IRQn 0 */

	/* USER CODE END ADC1_2_IRQn 0 */
	HAL_ADC_IRQHandler(&hadc1);
	/* USER CODE BEGIN ADC1_2_IRQn 1 */

	/* USER CODE END ADC1_2_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// fn que permite setar o valor da var modo_oper
void set_modo_oper(int m) {
	// OBS: seção crítica, desabilitamos todas as IRQs p/ atualizar var
	__disable_irq();                     // desabilita IRQs
	if (m > MAX_MODO_OPER)                // se x maior MAX permitido
	{
		modo_oper = MAX_MODO_OPER;           // set apenas com max
	} else if (m < 0)                      // se x menor que 0
			{
		modo_oper = 0;                       // set com 0
	} else                                // valor no intervalo 0-MAX
	{
		modo_oper = m;                       // modifica modo_oper
	}
	__enable_irq();                      // volta habilitar IRQs
}

// fn que qpenas retorna o valor da var modo_oper
int get_modo_oper(void) {
	static int x;                        // var local recebe modo_oper
	// OBS: seção crítica, desabilitamos todas as IRQs p/ atualizar var
	__disable_irq();                     // desabilita IRQs
	x = modo_oper;                       // faz x = modo_oper
	__enable_irq();                      // volta habilitar IRQs
	return x;                            // retorna x (=modo_oper)
}

// fn que permite setar o valor da var modo_oper
void set_modo_relogio(int m) {
	// OBS: seção crítica, desabilitamos todas as IRQs p/ atualizar var
	__disable_irq();                     // desabilita IRQs
	if (m > MAX_MODO_RELOG)                // se x maior MAX permitido
	{
		modo_relogio = MAX_MODO_RELOG;           // set apenas com max
	} else if (m < 0)                      // se x menor que 0
			{
		modo_relogio = 0;                       // set com 0
	} else                                // valor no intervalo 0-MAX
	{
		modo_relogio = m;                       // modifica modo_oper
	}
	__enable_irq();                      // volta habilitar IRQs
}

// fn que qpenas retorna o valor da var modo_oper
int get_modo_relogio(void) {
	static int x;                        // var local recebe modo_oper
	// OBS: seção crítica, desabilitamos todas as IRQs p/ atualizar var
	__disable_irq();                     // desabilita IRQs
	x = modo_relogio;                       // faz x = modo_oper
	__enable_irq();                      // volta habilitar IRQs
	return x;                            // retorna x (=modo_oper)
}

int get_horas(void) {
	static int x;                        // var local recebe modo_oper
	// OBS: seção crítica, desabilitamos todas as IRQs p/ atualizar var
	__disable_irq();                     // desabilita IRQs
	x = setup_hh;                       // faz x = modo_oper
	__enable_irq();                      // volta habilitar IRQs
	return x;                            // retorna x (=modo_oper)
}

int get_minutos(void) {
	static int x;                        // var local recebe modo_oper
	// OBS: seção crítica, desabilitamos todas as IRQs p/ atualizar var
	__disable_irq();                     // desabilita IRQs
	x = setup_mm;                       // faz x = modo_oper
	__enable_irq();                      // volta habilitar IRQs
	return x;                            // retorna x (=modo_oper)
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
