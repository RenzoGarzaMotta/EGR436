/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
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

/************ASCII CHARACTERS**********/
#define DEL				0x08
#define EOT				0x04
#define NL				0x0D
#define CR				0x0A
#define SPACE			0x20


/************FILE MANAGEMENT***********/

//Chip Info
//22:46:51  $GPRMC,224651.00,A,4259.03282,N,08541.36921,W,0.033,,110223,,,A*61
//22:46:51  $GPVTG,,T,,M,0.033,N,0.062,K,A*27
//22:46:51  $GPGGA,224651.00,4259.03282,N,08541.36921,W,1,05,1.84,209.9,M,-34.5,M,,*6B
//22:46:51  $GPGSA,A,3,19,03,06,17,14,,,,,,,,3.65,1.84,3.15*06
//22:46:51  $GPGSV,2,1,08,01,27,045,16,03,26,089,14,06,39,204,20,11,02,212,*78
//22:46:51  $GPGSV,2,2,08,14,59,126,29,17,73,010,16,19,57,281,29,24,21,310,14*7A
//22:46:51  $GPGLL,4259.03282,N,08541.36921,W,224651.00,A,A*7B

//
#define DIR_FIL_ADDR	0x000000	//Initial addr of the Directory File
#define BLOCK_1_ADDR	0x001000	//The address of the first block in memory (excluding The Directory File)
#define	BLOCK_OFFSET	0x001000	//Add this value to the initial 4KB block addr to get the next block's initial address
#define END_ADDR_OFF	0x000FFF	//Add this value to the initial 4KB block addr value to get the end address
#define PAGE_END_OFF	0x0000FF
#define PAGE_OFFSET		0x000100
#define LAST_PAGE_OFF	0x000F00
#define TOC_ADDR		0x000000
#define TOC_OFFSET		0x000020

//Directory File Macros
#define BLOCKS_4KB		127
#define BLOCKS_32KB		16
#define BLOCKS_64KB		8
#define PAGE_COUNT		16

/**********SPI FLASH COMMANDS**********/
const uint8_t WR_EN = 0x06;			//Write Enable
const uint8_t WR_EN_VSR = 0x50;		//Write Enable for Volatile Status Register
const uint8_t WR_DIS = 0x04;		//Write Disable
const uint8_t RD_SR = 0x05;			//Read Status Register
const uint8_t WR_SR = 0x01;			//Write Status Register
const uint8_t RD_DATA = 0x03;		//Read Data
const uint8_t RD_FAST = 0x0B;		//Read Fast
const uint8_t RD_FAST_DO = 0x3B;	//Read Fast Dual Output
const uint8_t RD_FAST_DIO = 0xBB;	//Read Fast Dual Input/Output
const uint8_t PG_PROG = 0x02;		//Page Program
const uint8_t ERASE_4KB = 0x20;		//Sector Erase 4KB
const uint8_t ERASE_32KB = 0x52;	//Sector Erase 32KB
const uint8_t ERASE_64KB = 0xD8;	//Sector Erase 64KB
const uint8_t ERASE_CHIP = 0x60;	//Erase Chip Memory
const uint8_t PWR_DWN = 0xB9;		//Power Down
const uint8_t REL_PWR_DWN = 0xAB;	//Release Power Down

///**********WRONG SPI FLASH COMMANDS**********/
////Read
//const uint8_t READ_ARRAY_0 = 0x0B;		//Up to 104 MHz
//const uint8_t READ_ARRAY_1 = 0x03;		//Up to 33 MHz
//const uint8_t READ_DUAL_OUTPUT = 0x03;		//Up to 50 MHz
//
////Program and Erase Commands
//const uint8_t ERASE_PAGE = 0x81;		//Up to 104 MHz
//const uint8_t ERASE_4KBLK = 0x20;		//Up to 104 MHz
//const uint8_t ERASE_32KBLK = 0x52;		//Up to 104 MHz
//const uint8_t ERASE_64KBLK = 0xD8;		//Up to 104 MHz
//const uint8_t ERASE_CHIP = 0x60;		//Up to 104 MHz
//const uint8_t ERASE_BYTE = 0xC7;		//Up to 104 MHz
//const uint8_t PROG_BYTE = 0x02;			//Up to 104 MHz
//const uint8_t PROG_PAGE = 0x02;			//Up to 104 MHz
//const uint8_t PROG_SEQ_0 = 0xAD;  		//Up to 104 MHz
//const uint8_t PROG_SEQ_1 = 0xAF;  		//Up to 104 MHz
//const uint8_t PROG_DUAL_INPUT = 0xA2;	//Up to 104 MHz
//
//
////Protection Commands
//const uint8_t WR_EN = 0x06;  			//Up to 104 MHz
//const uint8_t WR_DIS = 0x04;  			//Up to 104 MHz
//const uint8_t PROT_SECT = 0x36;  		//Up to 104 MHz
//const uint8_t UNPROT_SECT = 0x39;  		//Up to 104 MHz
//const uint8_t RD_SECT_PROT_REG = 0x3C;  //Up to 104 MHz
//
////Security Commands
//const uint8_t PROG_OTP = 0x9B;			//Up to 104 MHz
//const uint8_t READ_OTP = 0x77;			//Up to 104 MHz
//
////Status Register Commands
//const uint8_t READ_STAT_REG = 0x05;  	//Up to 104 MHz
//const uint8_t ACTIVE_STAT_INT = 0x25;  	//Up to 104 MHz
//const uint8_t WR_STAT_B1 = 0x01;  		//Up to 104 MHz
//const uint8_t WR_STAT_B2 = 0x31;  		//Up to 104 MHz
//
////Misc Commands
//const uint8_t FLASH_RESET = 0xF0;  		//Up to 104 MHz
//const uint8_t MFT_ID = 0x9F;  			//Up to 104 MHz
//const uint8_t DEEP_PWR_DWN = 0xB9;  	//Up to 104 MHz
//const uint8_t RESUME = 0xAB;  			//Up to 104 MHz
//const uint8_t ULTRA_DPD = 0x79;  		//Up to 104 MHz

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void writeEnable(void);
void writeDisable(void);
void readStatusRegister(void);
void pageProgram(uint32_t address, uint8_t *data, uint32_t size);
void readData(uint32_t address, uint8_t *data, uint32_t size);
void erase4KB(uint32_t address);
void erase32KB(uint32_t address);
void erase64KB(uint32_t address);
void eraseChip(void);

void confirmCmd(void);
uint8_t fileCensus(void);
uint8_t pageCensus(int blockStartAddr);
int getFileStart(uint8_t fileNum);
int getFileEnd(uint8_t fileNum);
void deleteFile(uint8_t fileNum);
void readFile(uint8_t fileNum);
int getFileSize(uint8_t fileNum);
int getUnavailableMemory(void);
int getAvailableMemory(void);
void storeFile(void);
void getTitle(uint8_t fileNum);
void printTOC(void);
void formatTOC(void);
void defragment(int address);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Global flags
volatile uint8_t spi_xmit_flag = 0;
volatile uint8_t spi_recv_flag = 0;

uint8_t rxData[20], cmd[50], cmdArg[50];
uint8_t txData[20];
uint8_t executeCmd, i, a, word;//Execute Command
uint8_t cmdResponse[100];

//char file[1028] = "Baby Baby\n\r\n\rBaby, baby, naughty baby,\n\rHush, you squalling thing, I say.\n\rPeace this moment, peace, or maybe\n\rBonaparte will pass this way.\n\r\n\rBaby, baby, he's a giant,\n\rTall and black as Rouen steeple,\n\rAnd he breakfasts, dines, rely on't,\n\rEvery day on naughty people.\n\r\n\rBaby, baby, if he hears you\n\rAs he gallops past the house,\n\rLimb from limb at once he'll tear you,\n\rJust as pussy tears a mouse.\n\r\n\rAnd he'll beat you, beat you, beat you,\n\rAnd he'll beat you into pap,\n\rAnd he'll eat you, eat you, eat you,\n\rEvery morsel snap, snap, snap.";

char file[1028];
char title[100] = {0};
char uart_buf[50];
int uart_buf_len;
uint8_t spi_buf[256];
uint8_t fileCount, pageCount;
uint32_t addr;


uint8_t* statReg;
uint8_t SRP, TB, BP2, BP1, BP0, WEL, BUSY;

uint8_t store, fileReceived;

int n;
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	//HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
	HAL_UART_Receive_IT(&huart1, rxData, 1);

	//HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
	HAL_UART_Transmit_IT(&huart1, txData, 1);

	// CS pin should default high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

//	/*****************COMMENT BELOW********************/
//	// Say something
//	uart_buf_len = sprintf(uart_buf, "SPI Test\r\n");
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//
//	addr = BLOCK_1_ADDR;
////	erase4KB(addr);

//	spi_buf[0] = 0xFF;
//	spi_buf[1] = 0xFF;
//	spi_buf[2] = 0xFF;
//
//	addr = BLOCK_1_ADDR;
//	readData(addr, spi_buf, 200);
//
//	readStatusRegister();
//	writeEnable();
//	readStatusRegister();
//
//	// Test bytes to write to EEPROM
//	spi_buf[0] = 0x12;
//	spi_buf[1] = 0x34;
//	spi_buf[2] = 0x56;
//
//	// Set starting address
//	addr = BLOCK_1_ADDR;
//
//	pageProgram(addr, spi_buf, 3);
//
//	spi_buf[0] = 0xFF;
//	spi_buf[1] = 0xFF;
//	spi_buf[2] = 0xFF;
//
//	addr = BLOCK_1_ADDR;
//	readData(addr, spi_buf, 3);
//	//buffer should read 12, 34, 56
//
//	readFile(1);

//	eraseChip();

//	pageProgram(0x000000 + BLOCK_OFFSET, (uint8_t*)file, 255);	//Store the incoming file at the next available location.
//	readData(0x000000 + BLOCK_OFFSET, spi_buf, 255);
//	HAL_UART_Transmit_IT(&huart1, spi_buf, 255);
//	HAL_UART_Transmit_IT(&huart1, "Hello World\n\r", 15);
//	getTitle(1);



//	int firstAddr, lastAddr, fileSize;
//	firstAddr = getFileStart(2);
//	lastAddr = getFileEnd(2);
//	fileSize = getFileSize(1);



	strcpy((char*)cmdResponse,"\r\n-->");
	HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
	HAL_Delay(20);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1){
		confirmCmd();
		if(fileReceived){
			storeFile();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*WRITE ENABLE*/  //WORKS
void writeEnable(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&WR_EN, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/*WRITE DISABLE*/  //WORKS
void writeDisable(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&WR_DIS, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/*READ STATUS REGISTER*/  //WORKS
void readStatusRegister(void){
	//Read Status Register
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&RD_SR, 1, 100);
	HAL_SPI_Receive(&hspi1, statReg, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	SRP = (*statReg 	& 0b10000000) >> 7;	//Status Register Protect
	TB  = (*statReg 	& 0b00100000) >> 5;	//Top/Bottom Protect
	BP2 = (*statReg 	& 0b00010000) >> 4;	//Block Protect 2
	BP1 = (*statReg 	& 0b00001000) >> 3;	//Block Protect 1
	BP0 = (*statReg 	& 0b00000100) >> 2;	//Block Protect 0
	WEL = (*statReg 	& 0b00000010) >> 1;	//Write Enable Latch
	BUSY = (*statReg & 0b00000001) >> 0;	//Erase/Write In Progress

	//	Print out status register
//	uart_buf_len = sprintf(uart_buf, "SRP: 0x%02x\r\n", SRP);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//	uart_buf_len = sprintf(uart_buf, "TB: 0x%02x\r\n", SRP);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//	uart_buf_len = sprintf(uart_buf, "BP2: 0x%02x\r\n", SRP);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//	uart_buf_len = sprintf(uart_buf, "BP1: 0x%02x\r\n", SRP);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//	uart_buf_len = sprintf(uart_buf, "BP0: 0x%02x\r\n", SRP);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//	uart_buf_len = sprintf(uart_buf, "WEL: 0x%02x\r\n", SRP);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//	uart_buf_len = sprintf(uart_buf, "BSY: 0x%02x\r\n", SRP);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

/*PAGE PROGRAM*/  //WORKS
void pageProgram(uint32_t address, uint8_t *data, uint32_t size){
	uint8_t command[4] = {PG_PROG, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_SPI_Transmit(&hspi1, data, size, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*READ DATA*/  //WORKS
void readData(uint32_t address, uint8_t *data, uint32_t size){
	uint8_t command[4] = {RD_DATA, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_SPI_Receive(&hspi1, data, size, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/*ERASE 4K*/  //WORKS
void erase4KB(uint32_t address){
	uint8_t command[4] = {ERASE_4KB, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*ERASE 32K*/  //WORKS
void erase32KB(uint32_t address){
	uint8_t command[4] = {ERASE_32KB, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*ERASE 64K*/  //WORKS
void erase64KB(uint32_t address){
	uint8_t command[4] = {ERASE_64KB, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*ERASE CHIPK*/  //WORKS
void eraseChip(void){
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&ERASE_CHIP, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}


void storeFile(void){
	uint8_t fileCount, pageCount = 0;
	fileCount = fileCensus();
	char filePacket[255];
	int address = fileCount * BLOCK_OFFSET;	//Get the address of the last file in memory
	int fileSize = strlen((char *)file);
	if(fileSize > 255){
		for(int startLocation = 0; startLocation < fileSize; startLocation += PAGE_OFFSET){
			int tempAddr = address + BLOCK_OFFSET + (PAGE_OFFSET * pageCount);
			memset((char *)filePacket, '\0', strlen((char *)filePacket));
			memcpy(filePacket, file + startLocation, PAGE_OFFSET);
			pageProgram(tempAddr, (uint8_t*)filePacket, PAGE_OFFSET);	//Store the incoming file at the next available location.
			HAL_Delay(500);
			pageCount++;
		}
	}else{
		pageProgram(address + BLOCK_OFFSET, (uint8_t*)file, 255);	//Store the incoming file at the next available location.
	}
	//Format the TOC after storing the data.
	formatTOC();
	memset((char *)file, '\0', strlen((char *)file));/*<--------------------------------------------------------------------------------------------------------------------*/
	strcpy((char*)cmdResponse,"\r\n-->");
	HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));

	memset((char *)cmd, '\0', strlen((char *)cmd));
	memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
	fileReceived = 0;
}

void confirmCmd(void){
	if(executeCmd){
		if (strcmp((char*)cmd, "STORE") == 0){							//DONE
			//Send receipt of command
//			strcpy((char*)cmdResponse,"STORE EXEC\r\n");
//			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
//			memset((char *)cmd, '\0', strlen((char *)cmd));
//			memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
			store = 1;
//			executeCmd = 0;
		}else if(strcmp((char*)cmd, "DELETE") == 0){					//DONE
			deleteFile(atoi((char *)cmdArg));
			HAL_Delay(1000);
			formatTOC();
			HAL_Delay(1000);
			strcpy((char*)cmdResponse,"DELETE EXEC\r\n-->");
			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
			memset((char *)cmd, '\0', strlen((char *)cmd));
			memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
			executeCmd = 0;
		}else if(strcmp((char*)cmd, "READ") == 0){						//DONE
			readFile(atoi((char *)cmdArg));
//			HAL_Delay(1000);
//			strcpy((char*)cmdResponse,"\r\nREAD EXEC\r\n-->");
//			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
//			memset((char *)cmd, '\0', strlen((char *)cmd));
//			memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
			executeCmd = 0;
		}else if(strcmp((char*)cmd, "DIR") == 0){
//			formatTOC();
			printTOC();
//			strcpy((char*)cmdResponse,"DIR EXEC\r\n");
//			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
			strcpy((char*)cmdResponse,"\r\n-->");
			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
			memset((char *)cmd, '\0', strlen((char *)cmd));
			memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
			executeCmd = 0;
		}else if(strcmp((char*)cmd, "MEM") == 0){						//Double-Check functionality when STORE works
//			getUnavailableMemory();
			getAvailableMemory();
			strcpy((char*)cmdResponse,"\r\n-->");
			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
			memset((char *)cmd, '\0', strlen((char *)cmd));
			memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
			executeCmd = 0;
		}else if(strcmp((char*)cmd, "CLEAR") == 0){						//DONE
			eraseChip();
			HAL_Delay(1000);
			strcpy((char*)cmdResponse,"Chip erased successfully\r\n-->");
			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
			memset((char *)cmd, '\0', strlen((char *)cmd));
			memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
			executeCmd = 0;
		}else{															//DONE
			//INVALID COMMAND
			strcpy((char*)cmdResponse,"INVALID\r\n-->");
			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
			memset((char *)cmd, '\0', strlen((char *)cmd));
			memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
			executeCmd = 0;
		}
	}
}

void formatTOC(void){
	int address = 0;
	int f = 1, fileCount = 0;
	erase4KB(TOC_ADDR);
	HAL_Delay(100);
	//	Get a census of the files in memory and update the table.
	fileCount = fileCensus();

	for(f = 1; f <= fileCount; f++){
		memset(title, '\0', strlen(title));
		getTitle(f);

		for(address = 0; address < 0x1E0; address += TOC_OFFSET){	//Find the address to store the filename
			if(((address >> 5) + 1) == f){
				break;
			}
		}

		pageProgram(address, (uint8_t *)title, strlen(title)); //File in TOC take up 32 bytes.
		HAL_Delay(100);
//		readData(address, spi_buf, PAGE_OFFSET);
	}
}

void printTOC(void){
	int address = 0;
	uint8_t f, fileCount;
	fileCount = fileCensus();
	memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
	for(f = 1; f <= fileCount; f++){
		for(address = 0; address < 0x1E0; address += TOC_OFFSET){	//Find the address to store the filename
			if(((address >> 5) + 1) == f){
				break;
			}
		}
		readData(address, spi_buf, 32);
		HAL_UART_Transmit_IT(&huart1, spi_buf, strlen((char *)spi_buf));
		HAL_Delay(500);
	}




//	readData(TOC_ADDR, spi_buf, 255);
//	HAL_UART_Transmit_IT(&huart1, spi_buf, strlen((char *)spi_buf));
//	HAL_Delay(500);
}

uint8_t fileCensus(void){
	//Go through all the starting addresses of ALL 4KB
	//sectors and keep track of all which don't start with 0xFF
	//need to get their location (Known) and filename.
	int address;
	fileCount = 0;
	for(address = BLOCK_1_ADDR; address < (BLOCK_1_ADDR * BLOCKS_4KB); address += BLOCK_OFFSET){
		memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
		readData(address, spi_buf, 1);
		if(spi_buf[0] != 0xFF){
			fileCount++;
		}else{
			break;
		}
	}
	return fileCount;
}

uint8_t pageCensus(int blockStartAddr){
	//Go through all the starting addresses of ALL 4KB
	//sectors and keep track of all which don't start with 0xFF
	//need to get their location (Known) and filename.
	int address;
	pageCount = 0;
	for(address = blockStartAddr; address <= (blockStartAddr + LAST_PAGE_OFF); address += PAGE_OFFSET){
		memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
		readData(address, spi_buf, 1);
		if(spi_buf[0] != 0xFF){
			pageCount++;
		}else{
			break;
		}
	}
	return pageCount;
}

void getTitle(uint8_t fileNum){
	int addr = 0, i = 4, fileSize;
	char fileNumString[30] = {0}, fileSizeString[20] = {0};
	memset((char *)fileNumString, '\0', strlen((char *)fileNumString));
	sprintf(fileNumString, "%d", (int)fileNum);

	title[0] = '\r';
	title[1] = '\n';
//	strcpy(&title[2], fileNumString);
//	title[3] = '-';

	i = 2 + strlen(fileNumString);
	for (int j = 2; j < i; j++) {
		title[j] = fileNumString[j - 2];
	}
	title[i] = '-';
	i+=1;
	addr = getFileStart(fileNum);
	readData(addr, spi_buf, 1);
	title[i] = spi_buf[0];
	for(addr = getFileStart(fileNum); spi_buf[0] != '\r'; addr++){
		readData(addr, spi_buf, 1);
		HAL_Delay(5);
		if(spi_buf[0] != '\r'){
			title[i] = spi_buf[0];
			i++;
		}else{
			break;
		}
	}
	fileSize = getFileSize(fileNum);
	sprintf(fileSizeString, " %d B", fileSize);
	strcpy(&title[i], fileSizeString);
	i += strlen(fileSizeString);

	do{
		title[i] = '\x20';
		i++;
	}while(i < 32);
}

int getFileStart(uint8_t fileNum){
	int address;
	for(address = BLOCK_1_ADDR; address < (BLOCK_1_ADDR * BLOCKS_4KB); address += BLOCK_OFFSET){
		if((address >> 12) == fileNum){
			memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
			readData(address, spi_buf, 1);
			if(spi_buf[0] != 0xFF){
				return address;
			}else{
				return 0;
			}
		}
	}
}

int getFileEnd(uint8_t fileNum){
	//Two Options

	//Option ONE:
	//read in page by page starting at the starting address until 0xFF char is found
	int address, pageAddr, charAddr;
	for(address = BLOCK_1_ADDR; address < (BLOCK_1_ADDR * BLOCKS_4KB); address += BLOCK_OFFSET){		//Find the desired 4KB Mem Block
		if((address >> 12) == fileNum){																	//Determine if the address corresponds to the file
			for(pageAddr = address + LAST_PAGE_OFF; pageAddr >= address; pageAddr -= PAGE_OFFSET){		//Determine which page has data in decending order
				memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
				readData(pageAddr, spi_buf, 1);
				if(spi_buf[0] != 0xFF){
					for(charAddr = pageAddr; charAddr  < (pageAddr + PAGE_END_OFF); charAddr++){
						memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
						readData(charAddr, spi_buf, 1);
						if(spi_buf[0] == '\0'){
							return charAddr;
						}else{
							continue;
						}
					}
				}else{
					continue;
				}
			}
		}
	}

	//Option TWO:
	//Count from the end of the 4KB sector until non-0xFF data is found

	return 0;
}

void deleteFile(uint8_t fileNum){
	//Find the file to delete
	//Delete File
	//Move last file in memory up to recently-deleted file.
	//Choose whether to move the entire file at once of breakdown file into data packets.

	int address;
	for(address = BLOCK_1_ADDR; address < (BLOCK_1_ADDR * BLOCKS_4KB); address += BLOCK_OFFSET){
		if((address >> 12) == fileNum){
			//DELETE 4KB sector
			erase4KB(address);
			HAL_Delay(1000);
			break;
		}
	}
	defragment(address);
}

void readFile(uint8_t fileNum){
	//Find the file to delete
	//Delete File
	//Move last file in memory up to recently-deleted file.
	//Choose whether to move the entire file at once of breakdown file into data packets.
//	uint8_t pageCount = 0;
	int address;
	for(address = BLOCK_1_ADDR; address < (BLOCK_1_ADDR * BLOCKS_4KB); address += BLOCK_OFFSET){
		if((address >> 12) == fileNum){
			pageCount = pageCensus(address);
			if(pageCount > 1){
				for(int startLocation = 0; startLocation < (PAGE_OFFSET * pageCount); startLocation += PAGE_OFFSET){
					int tempAddr = address + startLocation;
					readData(tempAddr, spi_buf, PAGE_OFFSET);	//Store the incoming file at the next available location.
					HAL_UART_Transmit_IT(&huart1, spi_buf, PAGE_OFFSET);
					HAL_Delay(500);
				}
			}else{
				readData(address, spi_buf,PAGE_OFFSET);
				HAL_UART_Transmit_IT(&huart1, spi_buf, PAGE_OFFSET);
				HAL_Delay(500);
			}

			//	pageProgram(address + BLOCK_OFFSET, (uint8_t*)file, strlen(file));	//Store the incoming file at the next available location.
			//Format the TOC after storing the data.
			//	formatTOC();
			memset((char *)file, '\0', strlen((char *)file));
			strcpy((char*)cmdResponse,"\r\n-->");
			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
			break;
		}
	}
}

int getFileSize(uint8_t fileNum){
	//Given the starting address and the last address of the file, determine the file-size
	int startAddr, endAddr;
	startAddr = getFileStart(fileNum);
	endAddr = getFileEnd(fileNum);
	return endAddr - startAddr; //returns number of bytes in the file
}

int getUnavailableMemory(void){
//	char buffer[20];
	int j, totalMemUsed = 0;
	for(j = 1; j < BLOCKS_4KB; j++){
		totalMemUsed += getFileSize(j);
	}
//	sprintf(buffer, "%d bytes\r\n", totalMemUsed);
//	HAL_UART_Transmit_IT(&huart1, (uint8_t *) buffer, strlen(buffer));
//	HAL_Delay(500);
//	memset((char *)cmd, '\0', strlen((char *)cmd));
//	memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
	return totalMemUsed;
}

int getAvailableMemory(void){
	char buffer[20];
	int totalMem = 524287 /*Bytes*/, availableMem, unavailableMem;
	unavailableMem = getUnavailableMemory();
	availableMem = totalMem - unavailableMem;
	sprintf(buffer, "%d Bytes Available\r\n", availableMem);
	HAL_UART_Transmit_IT(&huart1, (uint8_t *) buffer, strlen(buffer));
	HAL_Delay(500);
	memset((char *)cmd, '\0', strlen((char *)cmd));
	memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
	return availableMem;
}

void defragment(int address){
	int tempAddr;
//	for(address = address; address < (BLOCK_1_ADDR * BLOCKS_4KB); address += BLOCK_OFFSET){
//		readData(address, spi_buf, 1);
//		if(spi_buf[0] != 0xFF){
//			memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
//			continue;
//		}else{
			for(tempAddr = address; tempAddr < (BLOCK_1_ADDR * BLOCKS_4KB); tempAddr += PAGE_OFFSET){
				if((tempAddr % 0x1000) == 0){
					erase4KB(tempAddr);
					HAL_Delay(100);
				}
				memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
				readData(tempAddr + BLOCK_OFFSET, spi_buf, PAGE_OFFSET);
//				HAL_Delay(250);
				pageProgram(tempAddr, spi_buf, PAGE_OFFSET);
//				HAL_Delay(250);
				memset((char *)spi_buf, '\0', strlen((char *)spi_buf));
				readData(tempAddr, spi_buf, PAGE_OFFSET);
			}
//			break;
//		}
//	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	int length = (sizeof(cmd)/sizeof(cmd[0]));
	//Re-enable UART RX interrupt
	//HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
	HAL_UART_Receive_IT(&huart1, rxData, 1);
//	HAL_UART_Transmit_IT(&huart1, rxData, 1);


	if(executeCmd){
		if(store){
			if(rxData[0] != EOT){
				file[n] = rxData[0];
				n++;
			}else{
				file[n] = rxData[0];	//Store the EOT from the incoming file
				n = 0;
				executeCmd = 0;
				store = 0;
				fileReceived = 1;
			}
		}
	}else{
		if(word == 0){
//			if(rxData[0] == DEL){
//				cmd[i] = '\0';
//				i--;
			/*}else */if(rxData[0] == NL){
				//Do nothing
			}else if(rxData[0] == CR){
				executeCmd = 1;//Execute Command
				i = 0;
				a = 0;
			}else if(rxData[0] != SPACE){
				cmd[i] = rxData[0];
				i++;
				if(i > length){
					i = 0;
				}
			}else{
				word = 1;
			}
		}else if(word == 1){
//			if(rxData[0] == DEL){
//				cmdArg[a] = '\0';
//				a--;
			/*}else*/ if(rxData[0] == NL){
				//Do nothing
			}else if(rxData[0] == CR){
				executeCmd = 1;//Execute Command
				word = 0;
				i = 0;
				a = 0;
			}else if(rxData[0] != SPACE){
				cmdArg[a] = rxData[0];
				a++;
				if(a > length){
					a = 0;
				}
			}else{
				//Do nothing
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//Do nothing
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
