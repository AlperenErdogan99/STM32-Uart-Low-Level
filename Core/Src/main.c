#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"


volatile uint8_t temp;

/* Kullanilan INIT yol haritasi 
  1. Enable UART CLOCK and GPIO CLOCK
  2. Configure the UART pins Alternate Functions
  3. Enable UART by writing the UE bit in USART_CR1 register to 1 
  4. Program the M bit in USART_CR1 to define the word lenght
  5. Select the desire baud rate using the USART_BRR register 
  6. Enable the Transmitter/ Receiver by settings the TE and RE bits in USART_CR1 register 

*/

/*  
USART2 çevrebirimini kullaniyorum. 
UART'i PA2 ve PA3 pinleri üzerinden kullaniyorum.

*/

/* Makrolar*/

#define RCC_BaseAddr                            0x40023800
#define AHB1ENR_Reg_Offset                      0x30 
#define RCC_AHB1ENR_Reg_Addr                    RCC_BaseAddr + AHB1ENR_Reg_Offset
#define APB1ENR_Reg_Offset                      0x40 
#define RCC_APB1ENR_Reg_Addr                    RCC_BaseAddr + APB1ENR_Reg_Offset

#define GPIO_BaseAddr                           0x40020000
#define PORT_MODE_Reg_Offset                    0x00
#define GPIO_MODER_Reg_Addr                     GPIO_BaseAddr + PORT_MODE_Reg_Offset
#define PORT_OUTSPEED_Reg_Offset                0x08
#define GPIO_OUTSPEED_Reg_Addr                  GPIO_BaseAddr + PORT_OUTSPEED_Reg_Offset
#define ALTERNATE_FUNCTION_Reg_Offset           0x20
#define GPIO_ALTERNATE_FUNCTION_Reg_Addr        GPIO_BaseAddr + ALTERNATE_FUNCTION_Reg_Offset

#define USART2_BaseAddr                          0x40004400
#define CR1_Reg_Offset                           0x0C
#define USART2_CR1_Reg_Addr                      USART2_BaseAddr + CR1_Reg_Offset
#define BRR_Reg_Offset                           0x08 
#define USART2_BRR_Reg_Addr                      USART2_BaseAddr + BRR_Reg_Offset
#define DR_Reg_Offset                            0x04
#define USART2_DR_Reg_Addr                       USART2_BaseAddr + DR_Reg_Offset
#define SR_Reg_Offset                            0x00 
#define USART2_SR_Reg_Addr                       USART2_BaseAddr + SR_Reg_Offset


void UART_init( ){

 *(uint32_t *)(RCC_APB1ENR_Reg_Addr ) |= 0x00020000; // usart2 clock enable 
 *(uint32_t *)(RCC_AHB1ENR_Reg_Addr) |= 0x00000001;  // gpioa clock enable 
 
 
 *(uint32_t *)(GPIO_MODER_Reg_Addr) |=  0x00000020 ; // pa2 set alternate function mode 
 *(uint32_t *)(GPIO_MODER_Reg_Addr) |=  0x00000080 ; // pa3 set alternate function mode 
 
 *(uint32_t *)(GPIO_OUTSPEED_Reg_Addr) |= 0x00000030  ; // pa2 set high speed mode
 *(uint32_t *)(GPIO_OUTSPEED_Reg_Addr) |= 0x000000C0  ; // pa3 set high speed mode

 *(uint32_t *)(GPIO_ALTERNATE_FUNCTION_Reg_Addr) |=  0x00000700  ; // pa2 set AF7 function mode   
 *(uint32_t *)(GPIO_ALTERNATE_FUNCTION_Reg_Addr) |=  0x00007000  ; // pa3 set AF7 function mode 
 
 *(uint32_t *)(USART2_CR1_Reg_Addr) = 0x00000000 ; // clear CR1 Reg
 *(uint32_t *)(USART2_CR1_Reg_Addr) |=  0x00002000 ; // Uart Enable on CR1 reg
 
 *(uint32_t *)(USART2_CR1_Reg_Addr) &= ~(0x00001000) ; // config word lenght 8 bit data 
 
 *(uint32_t *)(USART2_BRR_Reg_Addr) |= (0x000001A0) ; // 115200 baudrate for 48Mhz USART clock 
 
 *(uint32_t *)(USART2_CR1_Reg_Addr) |=  0x0000000C ; // Transmitter and receiver enable. TE and RE bits set 
 
}

void UART_SendChar(uint8_t c){
*(uint32_t *)(USART2_DR_Reg_Addr) = c ; // load data to transmit buffer. Data register. 
while (! (*(uint32_t *)(USART2_SR_Reg_Addr)) & (0x00000040)); // wait for transmition complete
*(uint32_t *)(USART2_SR_Reg_Addr) &= ~(0xFFFFFFFF);
}

uint8_t UART_GetChar ( ){
uint8_t temp;
while (! (*(uint32_t *)(USART2_SR_Reg_Addr)) & (0x00000020)); // wait for receive buffer 
temp = *(uint32_t *)(USART2_DR_Reg_Addr); // read data 
return temp ; 
}

void UART_init(void);
void UART_SendChar (uint8_t c);
uint8_t UART_GetChar (void);
void SystemClock_Config(void);


int main(void)
{
 
  SystemClock_Config();
  UART_init();
  
  
 
  
  
  while (1)
  {
    UART_SendChar(temp);
    for (int i = 0 ; i<500000; i++);
    temp = UART_GetChar();
    
    
  }
  
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
