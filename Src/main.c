/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "dwt_stm32_delay.h"
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_16   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_16 + FLASH_PAGE_SIZE*8 - 1   /* End @ of user Flash area */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int adcValue[5];
uint32_t time=0;

uint32_t indX = 0;
int32_t dataRead[4096];
int enc1=0,enc2 =0;

int fotoValue[4];
int dataMeasured[40];
int sum[4];

uint8_t pressed = 0;
uint8_t writeFlash = 0;
uint8_t userButtonCounter = 0;
uint32_t lastChangeTime = 0;
uint16_t userButtonGesture = 0;
uint8_t lastState = 0; //0 = nincs lenyomva, 1 = lenyomva

int before = 0;
int next = 0;

uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PAGEError = 0, MemoryProgramStatus = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
void UserButtonHandler(int button_state);
void beep(int note, int duration);
int ReadFoto(int pin);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int64_t dataFlash = 0;

uint8_t data[2];
uint8_t buffer[14];
int16_t value[7];
int i=0;
int fotoindex=0;

//star wars start
const int c = (int)1000000/261; //261Hz
const int d = (int)1000000/294;  //294Hz
const int e = (int)1000000/329; //329Hz
const int f = (int)1000000/349; //349Hz
const int g = (int)1000000/391; //391Hz
const int gS = (int)1000000/415; //415Hz
const int a = (int)1000000/440; //440Hz
const int aS = (int)1000000/455; //455Hz
const int b = (int)1000000/466; //466Hz
const int cH = (int)1000000/523; //523Hz
const int cSH = (int)1000000/554; //554Hz
const int dH = (int)1000000/587; //587Hz
const int dSH = (int)1000000/622; //622Hz
const int eH = (int)1000000/659; //659Hz
const int fH = (int)1000000/698; //698Hz
const int fSH = (int)1000000/740; //740Hz
const int gH = (int)1000000/784; //784Hz
const int gSH = (int)1000000/830; //830Hz
const int aH = (int)1000000/880; //880Hz

void firstSection()
{
  beep(a, 500);
  beep(a, 500);    
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);  
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  HAL_Delay(500);
 
  beep(eH, 500);
  beep(eH, 500);
  beep(eH, 500);  
  beep(fH, 350);
  beep(cH, 150);
  beep(gS, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  HAL_Delay(500);
}
 
void secondSection()
{
  beep(aH, 500);
  beep(a, 300);
  beep(a, 150);
  beep(aH, 500);
  beep(gSH, 325);
  beep(gH, 175);
  beep(fSH, 125);
  beep(fH, 125);    
  beep(fSH, 250);
 
  HAL_Delay(325);
 
  beep(aS, 250);
  beep(dSH, 500);
  beep(dH, 325);  
  beep(cSH, 175);  
  beep(cH, 125);  
  beep(b, 125);  
  beep(cH, 250);  
 
  HAL_Delay(350);
}
//star wars end

void ok()
{
for (int i=1000; i<2000; i=i*1.02) beep(500000/i,10);  
for (int i=2000; i>1000; i=i*.98) beep(500000/i,10);
for (int i=1000; i<2000; i=i*1.02) beep(500000/i,10); 
}


void ohno()
{
for (int i=1000; i<1244; i=i*1.01)  beep(1000000/i,30);
for (int i=1244; i>1000; i=i*.99)beep(1000000/i,30);
}
	
void beep1()
{
	beep(b,100);
	HAL_Delay(50);
	beep(a,100);
	HAL_Delay(50);
	beep(g,100);
	HAL_Delay(50);
	beep(f,100);
	HAL_Delay(50);
	beep(e,100);
	HAL_Delay(50);
	beep(d,100);
	HAL_Delay(50);
	beep(c,100);
	HAL_Delay(50);
}

void beep2()
{
	beep(c,200);
	HAL_Delay(100);
	beep(d,200);
	HAL_Delay(100);
	beep(e,200);
	HAL_Delay(100);
	beep(f,200);
	HAL_Delay(100);
	beep(g,200);
	HAL_Delay(100);
	beep(a,200);
	HAL_Delay(100);
	beep(b,200);
	HAL_Delay(100);
}

void beep3()
{
	beep(cH,200);
	HAL_Delay(100);
	beep(dH,200);
	HAL_Delay(100);
	beep(eH,200);
	HAL_Delay(100);
	beep(fH,200);
	HAL_Delay(100);
	beep(gH,200);
	HAL_Delay(100);
	beep(aH,200);
	HAL_Delay(100);
}

void beep4()
{
	beep(500,50);
	HAL_Delay(100);
	beep(500,50);
	HAL_Delay(100);
	beep(500,50);
	HAL_Delay(100);
	
	beep(500,200);
	HAL_Delay(100);
	beep(500,200);
	HAL_Delay(100);
	beep(500,200);
	HAL_Delay(100);
	
	beep(500,50);
	HAL_Delay(100);
	beep(500,50);
	HAL_Delay(100);
	beep(500,50);
	HAL_Delay(100);
}
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	//microsec
	DWT_Delay_Init ();
	
	//BUZZER
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	TIM16->CCR1=0;
//	TIM16->PSC=100;
	
//	//GYRO
//	if(HAL_I2C_IsDeviceReady(&hi2c1, 0xD0, 2, 10) == HAL_OK)
//	{
//		ok();
//	}
//	else{beep4(); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);}
//	HAL_Delay(300);
//	
//	//Transmit via I2C
//	data[0]=0x6B;
//	data[1]=0x00;
//	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, data, 2, 10);
	
	
	//ENCODER
//	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	
	//Timer interrupt
	HAL_TIM_Base_Start_IT(&htim7);
	//set data
	for(int i=0;i< 4096;i++){
		dataRead[i]=0;
	}
	for(int i=0;i< 4;i++){
		fotoValue[i]=0;
		sum[i]=0;
	}
	for(int i=0;i< 40;i++){
		dataMeasured[i]=0;
	}
	
	/*dataRead[indX++] = 62;
	for(int i=0;i< 256;i++){
		if(i%2 == 0) dataRead[i]=i/2;
		else dataRead[i] = i*501;
	}*/
		/*for(int  i =0; i<400;i++)
		dataRead[indX++] = i*1000+1;*/
	/*for(int i=0;i<100;i++){
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		HAL_Delay(100);
	}*/
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(writeFlash == 1) break;
//		TIM16->PSC=2000-0.415*adcValue[fotoindex];
//		for(int i=0; i<4; i++)dataRead[indX++]=adcValue[i];
//		HAL_Delay(200);
	
		//IRLED + FOTO
		
	if(next>before) {
		for (int i=0; i<10; i++) {
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
					DWT_Delay_us(100);
					HAL_ADC_Start(&hadc1);
					for(int i =0;i < 5;i++){
						HAL_ADC_PollForConversion(&hadc1,100);
						fotoValue[i] = HAL_ADC_GetValue(&hadc1);
					}
					HAL_ADC_Stop(&hadc1);
					fotoValue[1]= fotoValue[1]-1300;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
					DWT_Delay_us(100000);
					dataMeasured[i] = fotoValue[0];
					
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
					DWT_Delay_us(100);
							HAL_ADC_Start(&hadc1);
					for(int i =0;i < 5;i++){
						HAL_ADC_PollForConversion(&hadc1,100);
						fotoValue[i] = HAL_ADC_GetValue(&hadc1);
					}
					HAL_ADC_Stop(&hadc1);
					fotoValue[1]= fotoValue[1]-1300;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
					DWT_Delay_us(100000);
					dataMeasured[i+10] = fotoValue[1];
			
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
					DWT_Delay_us(100);
							HAL_ADC_Start(&hadc1);
					for(int i =0;i < 5;i++){
						HAL_ADC_PollForConversion(&hadc1,100);
						fotoValue[i] = HAL_ADC_GetValue(&hadc1);
					}
					HAL_ADC_Stop(&hadc1);
					fotoValue[1]= fotoValue[1]-1300;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
					DWT_Delay_us(100000);
					dataMeasured[i+20] = fotoValue[2];
					
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
					DWT_Delay_us(100);
					HAL_ADC_Start(&hadc1);
					for(int i =0;i < 5;i++){
						HAL_ADC_PollForConversion(&hadc1,100);
						fotoValue[i] = HAL_ADC_GetValue(&hadc1);
					}
					HAL_ADC_Stop(&hadc1);
					fotoValue[1]= fotoValue[1]-1300;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
					DWT_Delay_us(100000);
					dataMeasured[i+30] = fotoValue[3];
		}
		for(int i=0; i<10;i++){
			sum[0] += dataMeasured[i];
			sum[1] += dataMeasured[i+10];
			sum[2] += dataMeasured[i+20];
			sum[3] += dataMeasured[i+30];
		}
		for(int i =0;i<4;i++){
			dataRead[indX++] = sum[i]/10;
			sum[i]=0;
		}
		before=next;
		beep(500, 100);
	}
	if(pressed == 1){
		pressed=0;
		beep(500, 50);
		HAL_Delay(50);
		beep(500, 50);
	}
	
	
		HAL_ADC_Start(&hadc1);
		for(int i =0;i < 5;i++){
			HAL_ADC_PollForConversion(&hadc1,100);
			adcValue[i] = HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1);
		HAL_Delay(50);
		UserButtonHandler(adcValue[4]);
		
		
//		//ENCODER
//		//if(adcValue[4] < 10) break;
//		enc1=TIM1->CNT;
//		dataRead[indX++] = enc1;
//		enc2=TIM2->CNT;
//		dataRead[indX++] = enc2;
//		HAL_Delay(100);
//		if(enc1 != 0 || enc2 != 0)
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		
//		//GYRO
//	data[0]=0x3B;
//	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, data, 1, 10);
//	HAL_Delay(10);
//	HAL_I2C_Master_Receive(&hi2c1, 0xD1, buffer, 14, 10);
//	for(int i=0;i<7;i++)
//		{
//			value[i]=(buffer[2*i]<<8) + buffer[2*i+1];
//		}
//	value[3]=value[3]/340.00+36.53;
//	HAL_Delay(1000);
//	for(int  i =0; i<7;i++)
//		dataRead[indX++] = value[i];
	
//		//BUZZER
//	
//	//STAR_WARS START
//	//Play first section
//  firstSection();
//  //Play second section
//  secondSection();
//  //Variant 1
//  beep(f, 250);  
//  beep(gS, 500);  
//  beep(f, 350);  
//  beep(a, 125);
//  beep(cH, 500);
//  beep(a, 375);  
//  beep(cH, 125);
//  beep(eH, 650);
//  HAL_Delay(500);
//  //Repeat second section
//  secondSection();
//  //Variant 2
//  beep(f, 250);  
//  beep(gS, 500);  
//  beep(f, 375);  
//  beep(cH, 125);
//  beep(a, 500);  
//  beep(f, 375);  
//  beep(cH, 125);
//  beep(a, 650);  
//  HAL_Delay(650);
//  //STAR_WARS END

//	ok();
//	HAL_Delay(500);
//	ohno();
//	HAL_Delay(500);
//	beep1();
//	HAL_Delay(500);
//	beep2();
//	HAL_Delay(500);
//	beep3();
//	HAL_Delay(500);
//	beep4();
//	HAL_Delay(500);
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
	if(1) // 0 - doesn't write to flash // 1 - writes to flash
	{
	/* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
	
	/* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
  /* Get the 1st page to erase */
  FirstPage = GetPage(FLASH_USER_START_ADDR);
  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
  /* Get the bank */
  BankNumber = GetBank(FLASH_USER_START_ADDR);
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;
	
	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PAGEError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
    /* Infinite loop */
    while (1)
    {
      //TODO: hiba visszajelz�s buzzeren
    }
  }
	/* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  Address = FLASH_USER_START_ADDR;

	indX = 0;
  while (Address < FLASH_USER_END_ADDR && indX < 4096)
  {
		if (dataRead[indX+1] < 0) dataFlash = ((int64_t)dataRead[indX+1] + 1<<32) + (int64_t)dataRead[indX];
		else dataFlash = ((int64_t)dataRead[indX+1]<<32) + (int64_t)dataRead[indX];
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, dataFlash) == HAL_OK)
    {
      Address = Address + 8;
			indX = indX + 2;
    }
   else
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
      while (1)
      {
			//TODO: hiba visszajelz�s buzzeren
      }
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	ok();
}
	
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 80;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim15);

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 80;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}

/**
  * @brief  Felismeri a gombbal lenyomott mint�kat
  * @param  userButton: a gombra k�t�tt pin ADC �rt�ke, GND = gomb lenyomva
  */
void UserButtonHandler(int userButton)
{
	if(time - lastChangeTime > 50) //DEBOUNCE
	{
	if(userButton < 10 && lastState ==0){
		lastState = 1;
		lastChangeTime = time;
	}
		
	else if(userButton > 500 && lastState ==1){
		lastState = 0;
		//dataRead[indX++] = time - lastChangeTime; //DEBUG
		if(time - lastChangeTime < 500) userButtonGesture += 2* (1 << userButtonCounter++); //r�vid
		else if(time - lastChangeTime >= 500) userButtonGesture += 3* (1 << userButtonCounter++); //hossz�
		lastChangeTime = time;
	}
	
	else if (userButtonGesture != 0 && userButton > 500 && time - lastChangeTime > 1000){
		//dataRead[indX++] = 666; //DEBUG
			switch(userButtonGesture){
				case 2: //r�vid
//					fotoindex++;
//					if(fotoindex>3)fotoindex=0;
					next++;
					break;
				case 3: //hossz�
					pressed = 1;
					fotoindex++;
					if(fotoindex>3)fotoindex=0;
					break;
				case 6: //r�vid-r�vid
					break;
				case 7: //hossz�-r�vid
					break;
				case 8: //r�vid-hossz�
					if(TIM16->CCR1 == 0) TIM16->CCR1 = 40;
					else TIM16->CCR1 = 0;
					break;
				case 9: //hossz�-hossz�
					break;
				case 21: //hossz�-hossz�-hossz�
					writeFlash = 1;
					break;
			}
		userButtonGesture = 0;
		userButtonCounter = 0;
	}
	}
}

//buzzer
/**
  * @brief  A megadott hangon, megadott ideig sipol.
  * @param  note: a prescaler �rt�ke, duration: a delay ms-ban
  */
void beep(int note, int duration)
{
  //Play tone on buzzerPin
	TIM16->CCR1 = 40;
  TIM16->PSC=note;
	HAL_Delay(duration);

  //Stop tone on buzzerPin
  TIM16->CCR1 = 0;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
