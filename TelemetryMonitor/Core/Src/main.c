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
#include "bno055_stm32.h"
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




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


uint8_t* statReg;
uint8_t SRP, TB, BP2, BP1, BP0, WEL, BUSY;

//BNO055 I2C Address
//#define BNO055_ADDR_LO	0x28
//#define BNO055_ADDR		BNO055_ADDR_LO<<1


///**************PAGE 0*********************/
//#define BNO_CHIP_ID				0x00
//#define BNO_ACC_ID				0x01
//#define BNO_MAG_ID				0x02
//#define BNO_GYR_ID				0x03
//#define BNO_SW_REV_ID_LSB		0x04
//#define BNO_SW_REV_ID_MSB		0x05
//#define BNO_BL_REV_ID			0x06
//#define BNO_PAGE_ID				0x07
//#define BNO_ACC_DATA_X_LSB		0x08
//#define BNO_ACC_DATA_X_MSB		0x09
//#define BNO_ACC_DATA_Y_LSB		0x0A
//#define BNO_ACC_DATA_Y_MSB		0x0B
//#define BNO_ACC_DATA_Z_LSB		0x0C
//#define BNO_ACC_DATA_Z_MSB		0x0D
//#define BNO_MAG_DATA_X_LSB		0x0E
//#define BNO_MAG_DATA_X_MSB		0x0F
//#define BNO_MAG_DATA_Y_LSB		0x10
//#define BNO_MAG_DATA_Y_MSB		0x11
//#define BNO_MAG_DATA_Z_LSB		0x12
//#define BNO_MAG_DATA_Z_MSB		0x13
//#define BNO_GYR_DATA_X_LSB		0x14
//#define BNO_GYR_DATA_X_MSB		0x15
//#define BNO_GYR_DATA_Y_LSB		0x16
//#define BNO_GYR_DATA_Y_MSB		0x17
//#define BNO_GYR_DATA_Z_LSB		0x18
//#define BNO_GYR_DATA_Z_MSB		0x19
//#define BNO_EUL_HEADING_LSB		0x1A
//#define BNO_EUL_HEADING_MSB		0x1B
//#define BNO_EUL_ROLL_LSB			0x1C
//#define BNO_EUL_ROLL_MSB			0x1D
//#define BNO_EUL_PITCH_LSB		0x1E
//#define BNO_EUL_PITCH_MSB		0x1F
//#define BNO_QUA_DATA_W_LSB		0x20
//#define BNO_QUA_DATA_W_MSB		0x21
//#define BNO_QUA_DATA_X_LSB		0x22
//#define BNO_QUA_DATA_X_MSB		0x23
//#define BNO_QUA_DATA_Y_LSB		0x24
//#define BNO_QUA_DATA_Y_MSB		0x25
//#define BNO_QUA_DATA_Z_LSB		0x26
//#define BNO_QUA_DATA_Z_MSB		0x27
//#define BNO_LIA_DATA_X_LSB		0x28
//#define BNO_LIA_DATA_X_MSB		0x29
//#define BNO_LIA_DATA_Y_LSB		0x2A
//#define BNO_LIA_DATA_Y_MSB		0x2B
//#define BNO_LIA_DATA_Z_LSB		0x2C
//#define BNO_LIA_DATA_Z_MSB		0x2D
//#define BNO_GRV_DATA_X_LSB		0x2E
//#define BNO_GRV_DATA_X_MSB		0x2F
//#define BNO_GRV_DATA_Y_LSB		0x30
//#define BNO_GRV_DATA_Y_MSB		0x31
//#define BNO_GRV_DATA_Z_LSB		0x32
//#define BNO_GRV_DATA_Z_MSB		0x33
//#define BNO_TEMP					0x34
//#define BNO_CALIB_STAT			0x35
//#define BNO_ST_RESULT			0x36
//#define BNO_INT_STA				0x37
//#define BNO_SYS_CLK_STATUS		0x38
//#define BNO_SYS_STATUS          0x39
//#define BNO_SYS_ERR             0x3A
//#define BNO_UNIT_SEL            0x3B
//
//#define BNO_OPR_MODE            0x3D
//#define BNO_PWR_MODE            0x3E
//#define BNO_SYS_TRIGGER         0x3F
//#define BNO_TEMP_SOURCE         0x40
//#define BNO_AXIS_MAP_CONFIG     0x41
//#define BNO_AXIS_MAP_SIGN       0x42
//#define BNO_SIC_MATRIX_LSB0     0x43
//#define BNO_SIC_MATRIX_MSB0     0x44
//#define BNO_SIC_MATRIX_LSB1     0x45
//#define BNO_SIC_MATRIX_MSB1     0x46
//#define BNO_SIC_MATRIX_LSB2     0x47
//#define BNO_SIC_MATRIX_MSB2     0x48
//#define BNO_SIC_MATRIX_LSB3     0x49
//#define BNO_SIC_MATRIX_MSB3     0x4A
//#define BNO_SIC_MATRIX_LSB4     0x4B
//#define BNO_SIC_MATRIX_MSB4     0x4C
//#define BNO_SIC_MATRIX_LSB5     0x4D
//#define BNO_SIC_MATRIX_MSB5     0x4E
//#define BNO_SIC_MATRIX_LSB6     0X4F
//#define BNO_SIC_MATRIX_MSB6     0x50
//#define BNO_SIC_MATRIX_LSB7     0x51
//#define BNO_SIC_MATRIX_MSB7     0x52
//#define BNO_SIC_MATRIX_LSB8     0x53
//#define BNO_SIC_MATRIX_MSB8     	0x54
//#define BNO_ACC_OFFSET_X_LSB		0x55
//#define BNO_ACC_OFFSET_X_MSB		0x56
//#define BNO_ACC_OFFSET_Y_LSB		0x57
//#define BNO_ACC_OFFSET_Y_MSB		0x58
//#define BNO_ACC_OFFSET_Z_LSB		0x59
//#define BNO_ACC_OFFSET_Z_MSB		0x5A
//#define BNO_MAG_OFFSET_X_LSB		0x5B
//#define BNO_MAG_OFFSET_X_MSB		0x5C
//#define BNO_MAG_OFFSET_Y_LSB		0x5D
//#define BNO_MAG_OFFSET_Y_MSB		0x5E
//#define BNO_MAG_OFFSET_Z_LSB		0X5F
//#define BNO_MAG_OFFSET_Z_MSB    	0x60
//#define BNO_GYR_OFFSET_X_LSB		0x61
//#define BNO_GYR_OFFSET_X_MSB		0x62
//#define BNO_GYR_OFFSET_Y_LSB		0x63
//#define BNO_GYR_OFFSET_Y_MSB		0x64
//#define BNO_GYR_OFFSET_Z_LSB		0x65
//#define BNO_GYR_OFFSET_Z_MSB    0x66
//#define BNO_ACC_RADIUS_LSB		0x67
//#define BNO_ACC_RADIUS_MSB		0x68
//#define BNO_MAG_RADIUS_LSB		0x69
//#define BNO_MAG_RADIUS_MSB		0x6A
//
///******************PAGE 1*********************/
//#define BNO_PAGE_ID				0x07
//#define BNO_ACC_CONFIG			0x08
//#define BNO_MAG_CONFIG			0x09
//#define BNO_GYR_CONFIG_0			0x0A
//#define BNO_GYR_CONFIG_1			0x0B
//#define BNO_ACC_SLEEP_CONFIG		0x0C
//#define BNO_GYR_SLEEP_CONFIG		0x0D
//
//#define BNO_INT_MSK				0x0F
//#define BNO_INT_EN				0x10
//#define BNO_ACC_AM_THRES			0x11
//#define BNO_ACC_INT_SETTINGS		0x12
//#define BNO_ACC_HG_DURATION		0x13
//#define BNO_ACC_HG_THRES			0x14
//#define BNO_ACC_NM_THRES			0x15
//#define BNO_ACC_NM_SET			0x16
//#define BNO_GYR_INT_SETTING		0x17
//#define BNO_GYR_HR_X_SET			0x18
//#define BNO_GYR_DUR_X			0x19
//#define BNO_GYR_HR_Y_SET			0x1A
//#define BNO_GYR_GYR_DUR_Y		0x1B
//#define BNO_GYR_HR_Z_SET			0x1C
//#define BNO_GYR_DUR_Z			0x1D
//#define BNO_GYR_AM_THRES			0x1E
//#define BNO_GR_AM_SET			0x1F



uint8_t I2C_Data[200];
uint8_t i;

uint8_t *w_MSB, *w_LSB;
uint8_t *x_MSB, *x_LSB;
uint8_t *y_MSB, *y_LSB;
uint8_t *z_MSB, *z_LSB;
uint16_t wQuat, xQuat, yQuat, zQuat;
char quatStr[50];

//BLE Module
uint8_t rxData1[20];
uint8_t txData1[20] = "B";

//USB_Serial
uint8_t rxData2[20];
uint8_t txData2[20] = "U";

//GPS Module
uint8_t rxData3[20];
uint8_t txData3[20] = "G";


uint8_t btnPress, serialOutput, dataOut;

char QuatW[20], QuatX[20], QuatY[20], QuatZ[20];
bno055_vector_t v;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
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

void storeFile(void);
void readFile(uint8_t fileNum);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

//	//HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
//	HAL_UART_Receive_IT(&huart1, rxData1, 1);
	HAL_UART_Receive_IT(&huart2, rxData2, 1);
	HAL_UART_Receive_IT(&huart3, rxData3, 1);

//	//HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
//	HAL_UART_Transmit_IT(&huart1, txData1, 1);
	HAL_UART_Transmit_IT(&huart2, txData2, 1);
	HAL_UART_Transmit_IT(&huart3, txData3, 1);

//	HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
//	                                    uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//
//
//	HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
//	                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)


//	HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
//	                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
//
//
//	HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
//	                                      uint16_t MemAddSize, uint8_t *pData, uint16_t Size)

	//Read Operation Mode
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x01, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x02, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x04, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x05, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1, 100);

//	for(i = 0; i < 200; i++){
//		HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, i, I2C_MEMADD_SIZE_8BIT, &I2C_Data[i], 1, 100);
//	}


//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, w_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x21, I2C_MEMADD_SIZE_8BIT, w_MSB, 1, 100);
//	wQuat = (*w_MSB << 8) & (*w_LSB);
//
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x22, I2C_MEMADD_SIZE_8BIT, x_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, x_MSB, 1, 100);
//	xQuat = (*x_MSB << 8) & (*x_LSB);
//
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x24, I2C_MEMADD_SIZE_8BIT, y_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x25, I2C_MEMADD_SIZE_8BIT, y_MSB, 1, 100);
//	yQuat = (*y_MSB << 8) & (*y_LSB);
//
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x26, I2C_MEMADD_SIZE_8BIT, z_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, 0x27, I2C_MEMADD_SIZE_8BIT, z_MSB, 1, 100);
//	zQuat = (*z_MSB << 8) & (*z_LSB);
//
//	sprintf(quatStr, "w:%u, x:%u, y:%u, z:%u", wQuat, xQuat, yQuat, zQuat);



//	HAL_I2C_Mem_Read_IT(&hi2c1, BNO055_ADDR, 0x04, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1);
//	HAL_I2C_Mem_Read_IT(&hi2c1, BNO055_ADDR, 0x05, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1);
//	HAL_I2C_Mem_Read_IT(&hi2c1, BNO055_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, I2C_Data, 1);
/**************************SPI_FLASH_TEST******************************/
//char uart_buf[50];
//int uart_buf_len;
//uint8_t spi_buf[256];
//uint32_t addr;
//
//	// Say something
//	uart_buf_len = sprintf(uart_buf, "SPI Test\r\n");
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//
//	addr = BLOCK_1_ADDR;
//	//	erase4KB(addr);
//
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

	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	v = bno055_getVectorQuaternion();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	if(dataOut == 0){//SERIAL IMU DATA
		v = bno055_getVectorQuaternion();
		sprintf(QuatW, "%.2lf,", v.w); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)QuatW, strlen(QuatW));
		HAL_Delay(10);
		sprintf(QuatX, "%.2lf,", v.x); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)QuatX, strlen(QuatX));
		HAL_Delay(10);
		sprintf(QuatY, "%.2lf,", v.y); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)QuatY, strlen(QuatY));
		HAL_Delay(10);
		sprintf(QuatZ, "%.2lf\n", v.z); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)QuatZ, strlen(QuatZ));
		HAL_Delay(10);
	}else if(dataOut == 1){//SERIAL GPS DATA
		serialOutput = 1;
	}else if(dataOut == 2){//BLUETOOTH IMU DATA
		v = bno055_getVectorQuaternion();
		sprintf(QuatW, "%.2lf,", v.w); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)QuatW, strlen(QuatW));
		HAL_Delay(10);
		sprintf(QuatX, "%.2lf,", v.x); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)QuatX, strlen(QuatX));
		HAL_Delay(10);
		sprintf(QuatY, "%.2lf,", v.y); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)QuatY, strlen(QuatY));
		HAL_Delay(10);
		sprintf(QuatZ, "%.2lf\n", v.z); // format the double with two decimal places
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)QuatZ, strlen(QuatZ));
		HAL_Delay(10);
	}else if(dataOut == 3){//BLUETOOTH GPS DATA
		serialOutput = 0;
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
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
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 57600;
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USR_BTN_Pin */
  GPIO_InitStruct.Pin = USR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USR_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	btnPress = 0;
	dataOut++;
	if(dataOut > 3){
		dataOut = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//Re-enable UART RX interrupt
	//HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
//	HAL_UART_Receive_IT(&huart1, rxData1, 1);	//Bluetooth Receiver
//	HAL_UART_Receive_IT(&huart2, rxData2, 1);	//Serial Receiver
	HAL_UART_Receive_IT(&huart3, rxData3, 1);	//GPS Receiver


	//ECHO TEST
//	HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)




	if(serialOutput == 0){
		HAL_UART_Transmit_IT(&huart1, rxData3, 1);	//Bluetooth Transmission
	}else{
		HAL_UART_Transmit_IT(&huart2, rxData3, 1);	//Serial Transmission
	}
	btnPress = 1;
//	HAL_UART_Transmit_IT(&huart1, rxData3, 1);	//Bluetooth Transmission
//	HAL_UART_Transmit_IT(&huart2, rxData3, 1);	//Serial Transmission
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//Do nothing
}

//uint8_t BNO_StatusCheck(void){
//	uint8_t *BNOStatus;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_SYS_STATUS, I2C_MEMADD_SIZE_8BIT, &BNOStatus, 1, 100);
//	return *BNOStatus;
//}
//
//void BNO_ErrorCheck(void){
//	uint8_t *BNOError;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_SYS_ERR, I2C_MEMADD_SIZE_8BIT, &BNOError, 1, 100);
//	return *BNOError;
//}

//uint8_t BNO_GetIDs(void){
//	BNO_GetChipID();
//	BNO_GetAccID();
//	BNO_GetMagID();
//	BNO_GetGyroID();
//}
//
//uint8_t BNO_GetChipID(void){
//	uint8_t Output;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_CHIP_ID, I2C_MEMADD_SIZE_8BIT, Output, 1, 100);
//	return Output;
//}
//
//uint8_t BNO_GetAccID(void){
//	uint8_t Output;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_ACC_ID, I2C_MEMADD_SIZE_8BIT, Output, 1, 100);
//	return Output;
//}
//
//uint8_t BNO_GetMagID(void){
//	uint8_t Output;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_MAG_ID, I2C_MEMADD_SIZE_8BIT, Output, 1, 100);
//	return Output;
//}
//
//uint8_t BNO_GetGyroID(void){
//	uint8_t Output;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_GYR_ID, I2C_MEMADD_SIZE_8BIT, Output, 1, 100);
//	return Output;
//}
//
//uint8_t BNO_GetData(void){
//	BNO_GetAccData();
//	BNO_GetMagData();
//	BNO_GetGyroData();
//}

//void BNO_GetAccData(uint16_t *accX, uint16_t *accY, uint16_t *accZ){
//	uint8_t accX_LSB, accX_MSB, accY_LSB, accY_MSB, accZ_LSB, accZ_MSB;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, &accX_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_ACC_DATA_X_MSB, I2C_MEMADD_SIZE_8BIT, &accX_MSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_ACC_DATA_Y_LSB, I2C_MEMADD_SIZE_8BIT, &accY_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_ACC_DATA_Y_MSB, I2C_MEMADD_SIZE_8BIT, &accY_MSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_ACC_DATA_Z_LSB, I2C_MEMADD_SIZE_8BIT, &accZ_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_ACC_DATA_Z_MSB, I2C_MEMADD_SIZE_8BIT, &accZ_MSB, 1, 100);
//	*accX = (accX_MSB << 8) | accX_LSB;
//	*accY = (accY_MSB << 8) | accY_LSB;
//	*accZ = (accZ_MSB << 8) | accZ_LSB;
//}
//
//void BNO_GetMagData(uint16_t *magX, uint16_t *magY, uint16_t *magZ){
//	uint8_t magX_LSB, magX_MSB, magY_LSB, magY_MSB, magZ_LSB, magZ_MSB;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_MAG_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, &magX_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_MAG_DATA_X_MSB, I2C_MEMADD_SIZE_8BIT, &magX_MSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_MAG_DATA_Y_LSB, I2C_MEMADD_SIZE_8BIT, &magY_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_MAG_DATA_Y_MSB, I2C_MEMADD_SIZE_8BIT, &magY_MSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_MAG_DATA_Z_LSB, I2C_MEMADD_SIZE_8BIT, &magZ_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_MAG_DATA_Z_MSB, I2C_MEMADD_SIZE_8BIT, &magZ_MSB, 1, 100);
//	*magX = (magX_MSB << 8) | magX_LSB;
//	*magY = (magY_MSB << 8) | magY_LSB;
//	*magZ = (magZ_MSB << 8) | magZ_LSB;
//}
//
//void BNO_GetGyroData(uint16_t *gyroX, uint16_t *gyroY, uint16_t *gyroZ){
//	uint8_t gyroX_LSB, gyroX_MSB, gyroY_LSB, gyroY_MSB, gyroZ_LSB, gyroZ_MSB;
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_GYR_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, &gyroX_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_GYR_DATA_X_MSB, I2C_MEMADD_SIZE_8BIT, &gyroX_MSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_GYR_DATA_Y_LSB, I2C_MEMADD_SIZE_8BIT, &gyroY_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_GYR_DATA_Y_MSB, I2C_MEMADD_SIZE_8BIT, &gyroY_MSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_GYR_DATA_Z_LSB, I2C_MEMADD_SIZE_8BIT, &gyroZ_LSB, 1, 100);
//	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDR, BNO_GYR_DATA_Z_MSB, I2C_MEMADD_SIZE_8BIT, &gyroZ_MSB, 1, 100);
//	*gyroX = (gyroX_MSB << 8) | gyroX_LSB;
//	*gyroY = (gyroY_MSB << 8) | gyroY_LSB;
//	*gyroZ = (gyroZ_MSB << 8) | gyroZ_LSB;
//}

//uint8_t BNO_GetEulerData(void){
//	BNO_GetEulerHeading();
//	BNO_GetEulerRoll();
//	BNO_GetEulerPitch();
//}
//
//uint8_t BNO_GetEulerHeading(void){
//
//}
//
//uint8_t BNO_GetEulerRoll(void){
//
//}
//
//uint8_t BNO_GetEulerPitch(void){
//
//}

//uint8_t BNO_GetQuat(void){
//	BNO_GetQuatW();
//	BNO_GetQuatX();
//	BNO_GetQuatY();
//	BNO_GetQuatZ();
//}
//
//uint8_t BNO_GetQuatW(void){
//
//}
//
//uint8_t BNO_GetQuatX(void){
//
//}
//
//uint8_t BNO_GetQuatY(void){
//
//}
//
//uint8_t BNO_GetQuatZ(void){
//
//}

//uint8_t BNO_GetLinData(void){
//	BNO_GetLinearX();
//	BNO_GetLinearY();
//	BNO_GetLinearZ();
//}
//
//uint8_t BNO_GetLinearX(void){
//
//}
//
//uint8_t BNO_GetLinearY(void){
//
//}
//
//uint8_t BNO_GetLinearZ(void){
//
//}
//
//uint8_t BNO_GetGravData(void){
//	BNO_GetGravX();
//	BNO_GetGravY();
//	BNO_GetGravZ();
//}
//
//uint8_t BNO_GetGravX(void){
//
//}
//
//uint8_t BNO_GetGravY(void){
//
//}
//
//uint8_t BNO_GetGravZ(void){
//
//}
//
//uint8_t BNO_GetTemp(void){
//
//}


















/*WRITE ENABLE*/  //WORKS
void writeEnable(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&WR_EN, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/*WRITE DISABLE*/  //WORKS
void writeDisable(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&WR_DIS, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/*READ STATUS REGISTER*/  //WORKS
void readStatusRegister(void){
	//Read Status Register
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&RD_SR, 1, 100);
	HAL_SPI_Receive(&hspi1, statReg, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

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
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_SPI_Transmit(&hspi1, data, size, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*READ DATA*/  //WORKS
void readData(uint32_t address, uint8_t *data, uint32_t size){
	uint8_t command[4] = {RD_DATA, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_SPI_Receive(&hspi1, data, size, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/*ERASE 4K*/  //WORKS
void erase4KB(uint32_t address){
	uint8_t command[4] = {ERASE_4KB, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*ERASE 32K*/  //WORKS
void erase32KB(uint32_t address){
	uint8_t command[4] = {ERASE_32KB, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*ERASE 64K*/  //WORKS
void erase64KB(uint32_t address){
	uint8_t command[4] = {ERASE_64KB, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, command, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

/*ERASE CHIPK*/  //WORKS
void eraseChip(void){
	writeEnable();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&ERASE_CHIP, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	do{
		readStatusRegister();
	}while(BUSY && WEL);
}

//void storeFile(void){
//	uint8_t fileCount, pageCount = 0;
//	fileCount = fileCensus();
//	char filePacket[255];
//	int address = fileCount * BLOCK_OFFSET;	//Get the address of the last file in memory
//	int fileSize = strlen((char *)file);
//	if(fileSize > 255){
//		for(int startLocation = 0; startLocation < fileSize; startLocation += PAGE_OFFSET){
//			int tempAddr = address + BLOCK_OFFSET + (PAGE_OFFSET * pageCount);
//			memset((char *)filePacket, '\0', strlen((char *)filePacket));
//			memcpy(filePacket, file + startLocation, PAGE_OFFSET);
//			pageProgram(tempAddr, (uint8_t*)filePacket, PAGE_OFFSET);	//Store the incoming file at the next available location.
//			HAL_Delay(500);
//			pageCount++;
//		}
//	}else{
//		pageProgram(address + BLOCK_OFFSET, (uint8_t*)file, 255);	//Store the incoming file at the next available location.
//	}
//	//Format the TOC after storing the data.
//	formatTOC();
//	memset((char *)file, '\0', strlen((char *)file));/*<--------------------------------------------------------------------------------------------------------------------*/
//	strcpy((char*)cmdResponse,"\r\n-->");
//	HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
//
//	memset((char *)cmd, '\0', strlen((char *)cmd));
//	memset((char *)cmdArg, '\0', strlen((char *)cmdArg));
//	fileReceived = 0;
//}
//
//void readFile(uint8_t fileNum){
//	//Find the file to delete
//	//Delete File
//	//Move last file in memory up to recently-deleted file.
//	//Choose whether to move the entire file at once of breakdown file into data packets.
////	uint8_t pageCount = 0;
//	int address;
//	for(address = BLOCK_1_ADDR; address < (BLOCK_1_ADDR * BLOCKS_4KB); address += BLOCK_OFFSET){
//		if((address >> 12) == fileNum){
//			pageCount = pageCensus(address);
//			if(pageCount > 1){
//				for(int startLocation = 0; startLocation < (PAGE_OFFSET * pageCount); startLocation += PAGE_OFFSET){
//					int tempAddr = address + startLocation;
//					readData(tempAddr, spi_buf, PAGE_OFFSET);	//Store the incoming file at the next available location.
//					HAL_UART_Transmit_IT(&huart1, spi_buf, PAGE_OFFSET);
//					HAL_Delay(500);
//				}
//			}else{
//				readData(address, spi_buf,PAGE_OFFSET);
//				HAL_UART_Transmit_IT(&huart1, spi_buf, PAGE_OFFSET);
//				HAL_Delay(500);
//			}
//
//			//	pageProgram(address + BLOCK_OFFSET, (uint8_t*)file, strlen(file));	//Store the incoming file at the next available location.
//			//Format the TOC after storing the data.
//			//	formatTOC();
//			memset((char *)file, '\0', strlen((char *)file));
//			strcpy((char*)cmdResponse,"\r\n-->");
//			HAL_UART_Transmit_IT(&huart1, cmdResponse, strlen((char *)cmdResponse));
//			break;
//		}
//	}
//}










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
