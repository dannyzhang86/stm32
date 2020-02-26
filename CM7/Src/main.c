/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HSEM_ID_0 (0U) /* HW semaphore 0*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig; 

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#define I2C_ADDRESS        0x50  //0xA0
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;
/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling****  ****I2C_TwoBoards communication based on Polling**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
static void CPU_CACHE_Enable(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF); 

	return ch;
}

char *strx,*extstrx,*Readystrx;
char RxBuffer[200],Rxcouter;

void Clear_Buffer(void)//????
{
	uint8_t i;
	HAL_UART_Transmit(&huart2,(uint8_t *)RxBuffer,20,10);
	for(i=0;i<Rxcouter;i++)
		RxBuffer[i]=0;//??
	Rxcouter=0;
}
/* USER CODE END 0 */

/**
* @brief  The application entry point.
* @retval int
*/
HAL_StatusTypeDef I2C_WRITE_ADDR16(I2C_HandleTypeDef *hi2c, uint16_t DevAddress_7bit, uint16_t Address,  uint8_t Data)
{
	HAL_StatusTypeDef CurStatus;
	uint8_t aTxBuffer[3];
	aTxBuffer[0] = Address>>8;
	aTxBuffer[1] = Address&0xff;
	aTxBuffer[2] = Data&0xff;

	CurStatus = HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)DevAddress_7bit, aTxBuffer, sizeof(aTxBuffer), 10000);
	printf("I2C_WRITE_ADDR16 CurStatus1 = 0x%2X",CurStatus);
	if(CurStatus != HAL_OK)
	{
		if(CurStatus == HAL_ERROR)
		{
			printf("HAL ERROR\n");
			return CurStatus;
		}
		else if(CurStatus == HAL_TIMEOUT)
		{
			printf("HAL_TIMEOUT\n");
			return CurStatus;
		}
		else if(CurStatus == HAL_BUSY)
		{
			unsigned int result;
			while((result = HAL_I2C_GetError(&I2cHandle)) != HAL_I2C_ERROR_NONE)
			{
				printf("HAL_I2C_GetError:%X",result);

				if( result == HAL_I2C_ERROR_BERR      ||  \
					result == HAL_I2C_ERROR_ARLO      ||  \
					result == HAL_I2C_ERROR_AF        ||  \
					result == HAL_I2C_ERROR_OVR       ||  \
					result == HAL_I2C_ERROR_DMA       ||  \
					result == HAL_I2C_ERROR_TIMEOUT   ||  \
					result == HAL_I2C_ERROR_SIZE      ||  \
					result == HAL_I2C_ERROR_DMA_PARAM )
				{
					CurStatus = HAL_ERROR;
					printf("HAL_I2C_Get Errors, Break!\n");
					return CurStatus;
				}
			}
			printf("I2C_WRITE_ADDR16 CurStatus2 = 0x%2X",CurStatus);
			CurStatus = HAL_OK;
			printf("I2C_WRITE_ADDR16 CurStatus3 = 0x%2X",CurStatus);
		}
		else
		{
			printf("ERROR\n");
			return CurStatus;
		}
	}
	return CurStatus;
}

HAL_StatusTypeDef I2C_WRITE_ADDR8(I2C_HandleTypeDef *hi2c, uint16_t DevAddress_7bit, uint8_t Address,  uint8_t Data)
{
	HAL_StatusTypeDef CurStatus;
	uint8_t aTxBuffer[2];
	aTxBuffer[0] = Address&0xff;
	aTxBuffer[1] = Data&0xff;

	CurStatus = HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)DevAddress_7bit, aTxBuffer, sizeof(aTxBuffer), 10000);
	printf("I2C_WRITE_ADDR8 CurStatus1 = 0x%2X",CurStatus);
	if(CurStatus != HAL_OK)
	{
		if(CurStatus == HAL_ERROR)
		{
			printf("HAL ERROR\n");
			return CurStatus;
		}
		else if(CurStatus == HAL_TIMEOUT)
		{
			printf("HAL_TIMEOUT\n");
			return CurStatus;
		}
		else if(CurStatus == HAL_BUSY)
		{
			unsigned int result;
			while((result = HAL_I2C_GetError(&I2cHandle)) != HAL_I2C_ERROR_NONE)
			{
				printf("HAL_I2C_GetError:%X",result);

				if( result == HAL_I2C_ERROR_BERR      ||  \
					result == HAL_I2C_ERROR_ARLO      ||  \
					result == HAL_I2C_ERROR_AF        ||  \
					result == HAL_I2C_ERROR_OVR       ||  \
					result == HAL_I2C_ERROR_DMA       ||  \
					result == HAL_I2C_ERROR_TIMEOUT   ||  \
					result == HAL_I2C_ERROR_SIZE      ||  \
					result == HAL_I2C_ERROR_DMA_PARAM )
				{
					CurStatus = HAL_ERROR;
					printf("HAL_I2C_Get Errors, Break!\n");
					return CurStatus;
				}
			}
			printf("WRITE CurStatus2 = 0x%2X",CurStatus);
			CurStatus = HAL_OK;
			printf("WRITE CurStatus3 = 0x%2X",CurStatus);
		}
		else
		{
			printf("ERROR\n");
			return CurStatus;
		}
	}
	return CurStatus;
}

uint8_t I2C_READ_ADDR16(I2C_HandleTypeDef *hi2c, uint16_t DevAddress_7bit, uint16_t Address)
{
	HAL_StatusTypeDef CurStatus;
	uint8_t aTxBuffer[2];
	aTxBuffer[0] = Address>>8;
	aTxBuffer[1] = Address&0xff;

	uint8_t aRxBuffer[1];
	memset(aRxBuffer, 0, 1);
	
	CurStatus = HAL_I2C_Master_Transmit(&I2cHandle,  DevAddress_7bit&0xff, aTxBuffer, sizeof(Address), 10000);
	printf("I2C_READ_ADDR16 CurStatus1 = 0x%2X",CurStatus);
	if(CurStatus != HAL_OK)
	{
		if(CurStatus == HAL_ERROR)
		{
			printf("HAL ERROR\n");
			return -1;
		}
		else if(CurStatus == HAL_TIMEOUT)
		{
			printf("HAL_TIMEOUT\n");
			return -1;
		}
		else if(CurStatus == HAL_BUSY)
		{
			unsigned int result;
			while((result = HAL_I2C_GetError(&I2cHandle)) != HAL_I2C_ERROR_NONE)
			{
				printf("HAL_I2C_GetError:%X",result);

				if( result == HAL_I2C_ERROR_BERR      ||  \
					result == HAL_I2C_ERROR_ARLO      ||  \
					result == HAL_I2C_ERROR_AF        ||  \
					result == HAL_I2C_ERROR_OVR       ||  \
					result == HAL_I2C_ERROR_DMA       ||  \
					result == HAL_I2C_ERROR_TIMEOUT   ||  \
					result == HAL_I2C_ERROR_SIZE      ||  \
					result == HAL_I2C_ERROR_DMA_PARAM )
				{
					CurStatus = HAL_ERROR;
					printf("HAL_I2C_Get Errors, Break!\n");
					return -1;
				}
			}
			printf("I2C_READ_ADDR16 CurStatus2 = 0x%2X",CurStatus);
			CurStatus = HAL_OK;
			printf("I2C_READ_ADDR16 CurStatus3 = 0x%2X",CurStatus);
		}
		else
		{
			printf("ERROR\n");
			return -1;
		}
	}

	CurStatus = HAL_I2C_Master_Receive(&I2cHandle,  DevAddress_7bit&0xff, aRxBuffer, sizeof(aRxBuffer), 10000);
	printf("READ CurStatus1 = 0x%2X",CurStatus);
	if(CurStatus != HAL_OK)
	{
		if(CurStatus == HAL_ERROR)
		{
			printf("HAL ERROR\n");
			return -1;
		}
		else if(CurStatus == HAL_TIMEOUT)
		{
			printf("HAL_TIMEOUT\n");
			return -1;
		}
		else if(CurStatus == HAL_BUSY)
		{
			unsigned int result;
			while((result = HAL_I2C_GetError(&I2cHandle)) != HAL_I2C_ERROR_NONE)
			{
				printf("HAL_I2C_GetError:%X",result);

				if( result == HAL_I2C_ERROR_BERR      ||  \
					result == HAL_I2C_ERROR_ARLO      ||  \
					result == HAL_I2C_ERROR_AF        ||  \
					result == HAL_I2C_ERROR_OVR       ||  \
					result == HAL_I2C_ERROR_DMA       ||  \
					result == HAL_I2C_ERROR_TIMEOUT   ||  \
					result == HAL_I2C_ERROR_SIZE      ||  \
					result == HAL_I2C_ERROR_DMA_PARAM )
				{
					CurStatus = HAL_ERROR;
					printf("HAL_I2C_Get Errors, Break!\n");
					return -1;
				}
			}
			printf("READ CurStatus2 = 0x%2X",CurStatus);
			CurStatus = HAL_OK;
			printf("READ CurStatus3 = 0x%2X",CurStatus);
		}
		else
		{
			printf("ERROR\n");
		}
	}
	return aRxBuffer[0];
}

uint8_t I2C_READ_ADDR8(I2C_HandleTypeDef *hi2c, uint16_t DevAddress_7bit, uint8_t Address)
{
	HAL_StatusTypeDef CurStatus;
	uint8_t aTxBuffer[1];
	aTxBuffer[0] = Address;

	uint8_t aRxBuffer[1];
	memset(aRxBuffer, 0, 1);
	
	CurStatus = HAL_I2C_Master_Transmit(&I2cHandle,  DevAddress_7bit&0xff, aTxBuffer, sizeof(aTxBuffer), 10000);
	printf("I2C_READ_ADDR8_WRITE CurStatus1 = 0x%2X",CurStatus);
	if(CurStatus != HAL_OK)
	{
		if(CurStatus == HAL_ERROR)
		{
			printf("HAL ERROR\n");
			return -1;
		}
		else if(CurStatus == HAL_TIMEOUT)
		{
			printf("HAL_TIMEOUT\n");
			return -1;
		}
		else if(CurStatus == HAL_BUSY)
		{
			unsigned int result;
			while((result = HAL_I2C_GetError(&I2cHandle)) != HAL_I2C_ERROR_NONE)
			{
				printf("HAL_I2C_GetError:%X",result);

				if( result == HAL_I2C_ERROR_BERR      ||  \
					result == HAL_I2C_ERROR_ARLO      ||  \
					result == HAL_I2C_ERROR_AF        ||  \
					result == HAL_I2C_ERROR_OVR       ||  \
					result == HAL_I2C_ERROR_DMA       ||  \
					result == HAL_I2C_ERROR_TIMEOUT   ||  \
					result == HAL_I2C_ERROR_SIZE      ||  \
					result == HAL_I2C_ERROR_DMA_PARAM )
				{
					CurStatus = HAL_ERROR;
					printf("HAL_I2C_Get Errors, Break!\n");
					return -1;
				}
			}
			printf("I2C_READ_ADDR8_WRITE CurStatus2 = 0x%2X",CurStatus);
			CurStatus = HAL_OK;
			printf("I2C_READ_ADDR8_WRITE CurStatus3 = 0x%2X",CurStatus);
		}
		else
		{
			printf("ERROR\n");
			return -1;
		}
	}

	CurStatus = HAL_I2C_Master_Receive(&I2cHandle, DevAddress_7bit&0xff, aRxBuffer, sizeof(aRxBuffer), 10000);
	printf("READ CurStatus1 = 0x%2X",CurStatus);
	if(CurStatus != HAL_OK)
	{
		if(CurStatus == HAL_ERROR)
		{
			printf("HAL ERROR\n");
			return -1;
		}
		else if(CurStatus == HAL_TIMEOUT)
		{
			printf("HAL_TIMEOUT\n");
			return -1;
		}
		else if(CurStatus == HAL_BUSY)
		{
			unsigned int result;
			while((result = HAL_I2C_GetError(&I2cHandle)) != HAL_I2C_ERROR_NONE)
			{
				printf("HAL_I2C_GetError:%X",result);

				if( result == HAL_I2C_ERROR_BERR      ||  \
					result == HAL_I2C_ERROR_ARLO      ||  \
					result == HAL_I2C_ERROR_AF        ||  \
					result == HAL_I2C_ERROR_OVR       ||  \
					result == HAL_I2C_ERROR_DMA       ||  \
					result == HAL_I2C_ERROR_TIMEOUT   ||  \
					result == HAL_I2C_ERROR_SIZE      ||  \
					result == HAL_I2C_ERROR_DMA_PARAM )
				{
					CurStatus = HAL_ERROR;
					printf("HAL_I2C_Get Errors, Break!\n");
					return -1;
				}
			}
			printf("READ CurStatus2 = 0x%2X",CurStatus);
			CurStatus = HAL_OK;
			printf("READ CurStatus3 = 0x%2X",CurStatus);
		}
		else
		{
			printf("ERROR\n");
		}
	}
	return aRxBuffer[0];
}
/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Supply configuration update enable 
	*/
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	/** Configure the main internal regulator output voltage 
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
	/** Macro to configure the PLL clock source 
	*/
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
		|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
		|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
		|RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
	PeriphClkInitStruct.PLL3.PLL3M = 4; //1;
	PeriphClkInitStruct.PLL3.PLL3N = 400; //24;
	PeriphClkInitStruct.PLL3.PLL3P = 2;
	PeriphClkInitStruct.PLL3.PLL3Q = 4;
	PeriphClkInitStruct.PLL3.PLL3R = 2;
	PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
	PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
	PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
	PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Enable USB Voltage detector 
	*/
	HAL_PWREx_EnableUSBVoltageDetector();
}

/**
* @brief ETH Initialization Function
* @param None
* @retval None
*/
static void MX_ETH_Init(void)
{

	/* USER CODE BEGIN ETH_Init 0 */

	/* USER CODE END ETH_Init 0 */

	uint8_t MACAddr[6] ;

	/* USER CODE BEGIN ETH_Init 1 */

	/* USER CODE END ETH_Init 1 */
	heth.Instance = ETH;
	MACAddr[0] = 0x00;
	MACAddr[1] = 0x80;
	MACAddr[2] = 0xE1;
	MACAddr[3] = 0x00;
	MACAddr[4] = 0x00;
	MACAddr[5] = 0x00;
	heth.Init.MACAddr = &MACAddr[0];
	heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
	heth.Init.TxDesc = DMATxDscrTab;
	heth.Init.RxDesc = DMARxDscrTab;
	heth.Init.RxBuffLen = 1524;

	/* USER CODE BEGIN MACADDRESS */

	/* USER CODE END MACADDRESS */

	if (HAL_ETH_Init(&heth) != HAL_OK)
	{
		Error_Handler();
	}

	memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
	TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
	TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
	/* USER CODE BEGIN ETH_Init 2 */

	/* USER CODE END ETH_Init 2 */

}

/**
* @brief I2C1 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0x0A;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0xFF;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter 
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter 
	*/
	// if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	// {
	//   Error_Handler();
	// }
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
* @brief USART2 Initialization Function
* @param None
* @retval None
*/
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
* @brief USART3 Initialization Function
* @param None
* @retval None
*/
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
* @brief USB_OTG_FS Initialization Function
* @param None
* @retval None
*/
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

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

	/* USER CODE END Error_Handler_Debug */
}
static void CPU_CACHE_Enable(void)
{
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
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

//########################################################################
//############### main() #################################################
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();
	/* USER CODE END 1 */

	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	//   int32_t timeout; 
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	// timeout = 0xFFFF;
	// while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	// if ( timeout < 0 )
	//  {
	//Error_Handler();
	// }
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	HSEM notification */
	/*HW semaphore Clock enable*/
	//__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	//HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	//HAL_HSEM_Release(HSEM_ID_0,0);
	/* wait until CPU2 wakes up from stop mode */
	//timeout = 0xFFFF;
	//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	//if ( timeout < 0 )
	//{
	//	 printf("timeout1111");
	//Error_Handler();

	//}
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	//  MX_GPIO_Init();
	//  MX_ETH_Init();
	MX_I2C1_Init();
	MX_USART3_UART_Init();
	//  MX_USB_OTG_FS_PCD_Init();
	//  MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	//while(1)
	//{
	//printf("1111/n");
	// }
	printf("1111/n");
	//printf("1111/n");
	//printf("1111/n");

	//while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 10000) != HAL_OK)
	//while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer[0], 1, 10000) != HAL_OK)
	//  {
	/* Error_Handler() function is called when Timeout error occurs.
	When Acknowledge failure occurs (Slave don't acknowledge it's address)
	Master restarts communication */
	//		HAL_StatusTypeDef result1;
	//		result1 = HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer[0], 1, 10000);
	//		printf("result1:%X",result1);
	//		unsigned int result2;
	//		result2 = HAL_I2C_GetError(&I2cHandle);
	//		printf("result2:%X",result2);

	//    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
	//    {
	//			printf("error111111");
	//      //Error_Handler();
	//  }
	// }
	//printf("HAL_I2C_Master_Receive11111111111111");
	//printf("%2X",aRxBuffer[0]); */
	/* while(1)
	{
	printf("HAL_I2C_Master_Receive1111/n");
	}*/

	uint16_t ADDR = 0x00; 
	uint8_t TX_DATA = 0xAA;
	printf("before:TX_DATA = 0x%2X",TX_DATA);  
	//printf("after1:aRxBuffer[1]=%2X",aRxBuffer[1]);
	//while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t*)aTxBuffer, TXBUFFERSIZE, 10000)!= HAL_OK)
	
	HAL_StatusTypeDef CurStatus;
	CurStatus = I2C_WRITE_ADDR16(&I2cHandle, I2C_ADDRESS, ADDR, TX_DATA);
	printf("WRITE CurStatus4 = 0x%2X",CurStatus);
	printf("HAL_I2C_Master_Transmit22222222222");

	/* Turn LED1 on: Transfer in Transmission process is correct */
	//  BSP_LED_On(LED1);
	HAL_Delay(1000);

	//while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 10000) != HAL_OK)
	uint16_t RX_DATA = 0x00; 
	RX_DATA = I2C_READ_ADDR16(&I2cHandle, I2C_ADDRESS, ADDR);
	printf("HAL_I2C_Master_Receive33333333333333");
	printf("RX_DATA = 0x%2X", RX_DATA);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
