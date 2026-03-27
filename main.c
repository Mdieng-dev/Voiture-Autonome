 /**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   STM32F7xx HAL API Template project 
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Driver_CAN.h"                 // ::CMSIS Driver:CAN
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_sdram.h"
#include <stdio.h>
#include "AppWizard.h"



//extern void BSP_SDRAM_Init(void);
extern int Init_GUIThread (void);
// -------RECEIVE MESSAGE PROGRAM-------

extern ARM_DRIVER_CAN Driver_CAN1;

 // -------Global variable-------
ARM_CAN_MSG_INFO rx_msg_info;
char data_buf[8];
char receive = 0;
int distance;

DMA2D_HandleTypeDef hdma2d;

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS2                  // when RTE component CMSIS RTOS2 is used
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#endif

#ifdef RTE_CMSIS_RTOS2_RTX5
/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}

/**
  * Override default HAL_InitTick function
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  
  UNUSED(TickPriority);

  return HAL_OK;
}
#endif

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/* Private functions ---------------------------------------------------------*/
  
void CAN_callback(uint32_t obj_idx, uint32_t event);
void Initialisation_CAN();



// ------------Fonction de decodage-------------


//    Ecran 1 : Gestion de l'Énergie et Autonomie
void UserSOC(char data[8]) ;
void RST_Displayed_Autonomy(char data[8]) ;
void AvailableEnergy(char data[8]) ;
void HVBatLevel1Failure(char data[8]) ;
void HVBatLevel2Failure(char data[8]) ;


//    Ecran 2 : Information sur la Recharge
void RST_BCB_State(char data[8]) ;
void ChargingPower(char data[8]) ;
void RST_ChargeCableTrapDoor(char data[8]) ;
void DomesticNetworkState(char data[8])  ;

//    Ecran 3 : Santé et Sécurité du Système (Alertes)
void BCBTempAlert(char data[8]) ;
void HVBatOverTemp(char data[8]) ;
void RST_InternalBCBTemp(char data[8]) ;
void HVIsolationImpedance(char data[8]) ;


//    Ecran 4 : Conduite et Performance
void VehicleSpeed(char data[8]) ;
void RST_InstantPEBPower(char data[8]) ;
void ElecMachineSpeed(char data[8]) ;
void RST_MotorDriveAuthorization(char data[8]) ;


//    Ecran 5 : Maintenance et Diagnostic
int DistanceTotalizer(char data[8]);
void HVBatHealth(char data[8]) ;
void RST_BMS_Soft_Number(char data[8]) ;
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* This project template calls firstly two functions in order to configure MPU feature 
     and to enable the CPU Cache, respectively MPU_Config() and CPU_CACHE_Enable().
     These functions are provided as template implementation that User may integrate 
     in his application, to enhance the performance in case of use of AXI interface 
     with several masters. */ 
  
  /* Configure the MPU attributes as Write Through */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
	
  /* Initialize BSP SDRAM */
  BSP_SDRAM_Init();
	
	//MX_DMA2D_Init();

  /* Configure the System clock to have a frequency of 216 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();
	
	


  /* Add your application code here
     */

#ifdef RTE_CMSIS_RTOS2
  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();
	Initialisation_CAN();
	
  /* Create thread functions that start executing, 
  Example: osThreadNew(app_main, NULL, NULL); */
	Init_GUIThread();

  /* Start thread execution */
  osKernelStart();
#endif

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25U;
  RCC_OscInitStruct.PLL.PLLN = 400U;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9U;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the Embedded SRAM1 as (default): cacheable, write-back, 
     allocate on read or write miss, non-shareable normal memory, execute never

     Note:
     DTCM starting at address 0x20000000 with size of 64 kB, regardless of 
     MPU settings behaves as non-cacheable, non-shareable normal memory */

  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress      = 0x20000000U;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00U;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the External SDRAM as: non-cacheable, non-shareable normal memory, execute never

     Note:
     External SDRAM starting at address 0xC0000000, regardless of MPU settings 
     behaves as non-cacheable memory */

  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress      = 0xC0000000U;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_16MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00U;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 



// -------DEFINITION DES FONCTIONS-------

void CAN_callback(uint32_t obj_idx, uint32_t event)
{
    switch (event)
    {
    case ARM_CAN_EVENT_RECEIVE:
        //  Message was received successfully by the obj_idx object.
       //traiter les donn?e dans data_buf
				receive = 1;
				Driver_CAN1.MessageRead(0, &rx_msg_info, data_buf, 8); //
//TODO, vérifier l'ID dans  rx_msg_info
        switch (rx_msg_info.id)  {
		
					case 0x155:
						
						
						break;
					
					
					
					case 0x599:
						
						break;
					
					
					case 0x425:
						
						break;					
					case 0x597:
						
						break;


					case 0x196:
						
						break;

          case 0x5D7 :
						
						distance =	DistanceTotalizer(data_buf);
					  //APPW_SetValue(ID_VAR_distancetotalizer,distance);
					
						break;
					
					
					case 0x59B:
						
						break;

					case 0x19F:
						
						break;


					case 0x59F:
						
						break;


					case 0x424:
						
						break;					

					case 0x55F:
						
						break;


               // DecodeCANMessage(uint32_t frame_id, char data[8]);

        }

				
				
				
				
        break;
    }
}

void Initialisation_CAN()
{
	volatile int32_t                  status;
	 
	status=Driver_CAN1.Initialize(NULL, CAN_callback);
	status=Driver_CAN1.PowerControl(ARM_POWER_FULL);
	
	status=Driver_CAN1.SetMode(ARM_CAN_MODE_INITIALIZATION);
	status=Driver_CAN1.SetBitrate(
												ARM_CAN_BITRATE_NOMINAL, // d?bit fixe
												500000, // 500 kbits/s (HS)
												ARM_CAN_BIT_PROP_SEG(3U) | // prop. seg = 3 TQ
												ARM_CAN_BIT_PHASE_SEG1(1U) | // phase seg1 = 1 TQ
												ARM_CAN_BIT_PHASE_SEG2(1U) | // phase seg2 = 1 TQ
												ARM_CAN_BIT_SJW(1U) // Resync. Seg = 1 TQ
										);
	


	
	status=Driver_CAN1.ObjectConfigure(0,ARM_CAN_OBJ_RX); // Objet 0 pour r?ception
	status=Driver_CAN1.ObjectConfigure(2,ARM_CAN_OBJ_TX); // Objet 2 pour send
	
	status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x599),0); // the last arg is Mask or end of ID range (depending on filter type)
		
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x155),0);
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x599),0);
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x425),0);
        
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x597),0);

    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x196),0);
        
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x5D7),0);
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x59B),0);
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x19F),0);
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x59F),0);
        
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x424),0);
    status=Driver_CAN1.ObjectSetFilter( 0, ARM_CAN_FILTER_ID_EXACT_ADD , ARM_CAN_STANDARD_ID(0x55F),0);

	status=Driver_CAN1.SetMode(ARM_CAN_MODE_NORMAL); // fin initialisation
	
}







//    Ecran 1 : Gestion de l'Énergie et Autonomie
void UserSOC(char data[8]) {
	// variable
	char val_soc[15];
	float soc = 0;
	
	// calcul
	soc = ((data[4]<<5) + (data[5]>>3))*0.02; //%
}



void RST_Displayed_Autonomy(char data[8]) {
	// variable
	char val_auto[15];
	float autono = 0;
	
	// calcul
	autono = (data[5]<<0); //km


	// à afficher
}



void AvailableEnergy(char data[8]) {
	// variable
	char val_avNRJ[15];
	float avNRJ = 0;
	
	// calcul
	avNRJ = (((data[0]&0x01)<<8) + (data[1]))*0.1; //kwh

	// à afficher
}




void HVBatLevel1Failure(char data[8]) {
	// variable
	short batfail11 = 0;
	char texte[30];
	// calcul
	batfail11 = (data[3]>>2)&0x03; 
		switch (batfail11) {
			case 0x00: 
				sprintf(texte, "Not used"); 
			break;
			case 0x01: 
				sprintf(texte, "No default"); 
			break;
			case 0x02: 
				sprintf(texte,  "Avertissement niveau 1"); 
			break;
			default: 
				sprintf(texte, "Valeur non disponible"); 
			break;
	}

	// à afficher
}



void HVBatLevel2Failure(char data[8]) {
	// variable
	short batfail11 = 0;
	char texte[30];
	// calcul
	batfail11 = (data[3]>>4)&0x03; 
	switch (batfail11) {
		case 0x00: 
			sprintf(texte, "Non utilisé"); 
		break;
		case 0x01: 
			sprintf(texte, "No default"); 
		break;
		case 0x02: 
			sprintf(texte,  "Défaillance niveau 2"); 
		break;
		default: 
			sprintf(texte, "Valeur non disponible"); 
		break;
		}

	// à afficher


}


//    Ecran 2 : Information sur la Recharge
void RST_BCB_State(char data[8]) {
	// variable
	char texte[30];
	short state = 0;
	
	// calcul
	state = (data[1]>>5)&0x03 ;
	
	switch (state) {
	case 0x00: 
		sprintf(texte, "Non utilisé"); 
	break;
	case 0x01: 
		sprintf(texte, "No default"); 
	break;
	case 0x02: 
		sprintf(texte,  "Défaillance niveau 2"); 
	break;
	default: 
		sprintf(texte, "Valeur non disponible"); 
	break;
	}

	// à afficher


}

void ChargingPower(char data[8]) {
	// variable
	char val_charge[15];
	float charge = 0;
	
	// calcul
	charge = (data[0])*0.3; //kW

	// à afficher


}



void RST_ChargeCableTrapDoor(char data[8]) {
	// variable
	char texte[30];
	short trapDoor = 0;
	
	// calcul
	trapDoor = (data[1])&0x01; 
	switch (trapDoor) {
		case 0x00: 
			sprintf(texte, "Trap Door opened"); 
		break;
		case 0x01: 
			sprintf(texte, "Trap Door closed"); 
		break;
	}
	// à afficher

}


void DomesticNetworkState(char data[8])  {
	// variable
	char texte[30];
	short netState = 0;
	
	// calcul
	netState = (data[0]>>5)&0x01; 
	switch (netState) {
		case 0x00: 
			sprintf(texte, "Domestic Network is not present"); 
		break;
		case 0x01: 
			sprintf(texte, "Domestic Network is present"); 
		break;
	}
	// à afficher

}

//    Ecran 3 : Santé et Sécurité du Système (Alertes)
void BCBTempAlert(char data[8]) {
	// variable
	char texte[30];
	short tempAlert = 0;
	
	// calcul
	tempAlert = (data[0])&0x03; 
	switch (tempAlert) {
		case 0x00: 
			sprintf(texte, "No Temperature Alert"); 
		break;
		case 0x01: 
			sprintf(texte, "High Temp Alert"); 
		break;
		case 0x02: 
			sprintf(texte,  "Over Temp Alert"); 
		break;
		default: 
			sprintf(texte, "Not used"); 
		break;
	}
	// à afficher




}
void HVBatOverVolt(char data[8]) {
	// variable
	char texte[30];
	short overVolt = 0;
	
	// calcul
	overVolt = (data[1]>>2)&0x03; 
	switch (overVolt) {
		case 0x00: 
			sprintf(texte, "No failure"); 
		break;
		case 0x01: 
			sprintf(texte, "Level 1 Overvoltage"); 
		break;
		case 0x02: 
			sprintf(texte,  "Level 2 Overvoltage"); 
		break;
		default: 
			sprintf(texte, "Unavailable value"); 
		break;
	}
	// à afficher

}


void RST_InternalBCBTemp(char data[8]) {

	// variable
	char temp = 0;
	
	// calcul
	temp = (data[7]); //°C
	// à afficher



}



void HVIsolationImpedance(char data[8]) {
	// variable
	float impedence = 0;
	
	// calcul
	impedence = ((data[3]<<6)+ (data[4]>>2))*100; // Ohm
	// à afficher


}


//    Ecran 4 : Conduite et Performance
void VehicleSpeed(char data[8]) {
	// variable
	float speed = 0;
	
	// calcul
	speed = ((data[0]<<8) + (data[1]))*0.01; //km/h
	// à afficher




}
void RST_InstantPEBPower(char data[8]) {
	// variable
	short power = 0;
	
	// calcul
	power = (data[2]); //%
	// à afficher
}


void ElecMachineSpeed(char data[8]) {
	// variable
	float MachSpeed = 0;
	
	// calcul
	MachSpeed = ((data[2]<<4)+(data[3]>>4)); //%
	// à afficher

}
void RST_MotorDriveAuthorization(char data[8]) {
	// variable
	char texte[30];
	short driveAuthorization = 0;
	
	// calcul
	driveAuthorization = (data[3])&0x01; 
	switch (driveAuthorization) {
		case 0x00: 
			sprintf(texte, "MotorDriveAuthorized"); 
		break;
		case 0x01: 
			sprintf(texte, "MotorDriveForbidden"); 
		break;
	}
	// à afficher


}


//    Ecran 5 : Maintenance et Diagnostic

// distance
int DistanceTotalizer(char data[8]){
	// variable
	char val_dist[15];
	int dist = 0;
	
	// calcul
	dist = (data[1]<<16) + (data[2]<<8)+(data[3]<<0);
	// à afficher
	return dist;
}


void HVBatHealth(char data[8]) {
	// variable
	short batHealth = 0;
	
	// calcul
	batHealth = (data[5])&0x7F;  // %

	// à afficher




}


void RST_BMS_Soft_Number(char data[8]) {
	// variable
	char texte[30];
	short overTemp = 0;
	
	// calcul
	overTemp = data[2]; 

	sprintf(texte, "Software number"); 
	
	// à afficher
}