#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "stm32f4xx.h"                  // Device header
#include "Driver_I2C.h"                 // CMSIS Driver:I2C
#include "Driver_CAN.h"                 // CMSIS Driver:CAN
#include "RTE_Device.h"                 // Device:STM32Cube Framework:Classic
#include "rtx_os.h"                     // CMSIS:RTOS2:Keil RTX5&&Source
#include "EventRecorder.h"              // CMSIS-View:Event Recorder&&DAP

extern ARM_DRIVER_I2C        Driver_I2C1;
#define I2C_A                (&Driver_I2C1)

extern ARM_DRIVER_CAN Driver_CAN2;

osThreadId_t ID_tacheI2C,ID_tacheCAN;
osThreadId_t ID_tacheLED; 

osMutexId_t ID_Mutex_Data; 


#define I2C_EVENT_DONE 0x01
#define CAN_EVENT_DONE 0x01


#define NB_CAPTEURS 4 // Modifié à 3 selon ta description textuelle
const uint8_t ADRESSES_I2C[NB_CAPTEURS] = {(0xE0>>1), (0xE2>>1), (0xE4>>1),(0xE8>>1)};

uint16_t distance_global[NB_CAPTEURS];
/*----------------------------------------------------------------------------
 * Fonction de Callback I2C
 *---------------------------------------------------------------------------*/


void callbackI2C(uint32_t event) {
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        osThreadFlagsSet(ID_tacheI2C, I2C_EVENT_DONE);
    }
}

void callbackCAN(uint32_t obj_idx, uint32_t event) {
	if (event & ARM_CAN_EVENT_SEND_COMPLETE) {
		osThreadFlagsSet(ID_tacheCAN, CAN_EVENT_DONE); 
	}
}

/*----------------------------------------------------------------------------
 * Fonctions d'Initialisation
 *---------------------------------------------------------------------------*/
void init_LED(void){
    __HAL_RCC_GPIOD_CLK_ENABLE(); 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
    GPIO_InitStruct.Pull = GPIO_NOPULL;            
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void init_I2C(void){
    Driver_I2C1.Initialize(callbackI2C); 
    Driver_I2C1.PowerControl(ARM_POWER_FULL);
    Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD); 
}

void init_CAN(void){
	Driver_CAN2.Initialize(NULL, callbackCAN);
	Driver_CAN2.PowerControl(ARM_POWER_FULL);
	Driver_CAN2.SetMode(ARM_CAN_MODE_INITIALIZATION);
	Driver_CAN2.SetBitrate( ARM_CAN_BITRATE_NOMINAL, // débit fixe
													125000, // 125 kbits/s (LS)
													ARM_CAN_BIT_PROP_SEG(3U) | // prop. seg = 3 TQ
													ARM_CAN_BIT_PHASE_SEG1(1U) | // phase seg1 = 1 TQ
													ARM_CAN_BIT_PHASE_SEG2(1U) | // phase seg2 = 1 TQ
													ARM_CAN_BIT_SJW(1U) // Resync. Seg = 1 TQ
													);
	
	Driver_CAN2.ObjectConfigure(2,ARM_CAN_OBJ_TX); 
	Driver_CAN2.SetMode(ARM_CAN_MODE_NORMAL); 
}

/*----------------------------------------------------------------------------
* Tache I2C 
 *---------------------------------------------------------------------------*/
static void tache_I2C (void *argument) {
    uint8_t addr1 = ADRESSES_I2C[1]; 
    uint8_t addr2 = ADRESSES_I2C[3];
    uint8_t reg_cmd[2] = {0x00, 0x51};
    uint8_t reg_msb = 0x02, reg_lsb = 0x03;
    uint8_t vMSB, vLSB;
    uint16_t dist;
		
    while(1) {
        // --- LECTURE CAPTEUR 1 ---
        Driver_I2C1.MasterTransmit(addr1, reg_cmd, 2, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        osDelay(70);
        
        Driver_I2C1.MasterTransmit(addr1, &reg_msb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr1, &vMSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        
        Driver_I2C1.MasterTransmit(addr1, &reg_lsb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr1, &vLSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);

        dist = (vMSB << 8) | vLSB;
				
				//ecriture le la valeur mesurée, du capteur 1, a l'aide du mutex
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        distance_global[1] = dist; 
        osMutexRelease(ID_Mutex_Data);
				
			
        // --- LECTURE CAPTEUR 2 ---
        Driver_I2C1.MasterTransmit(addr2, reg_cmd, 2, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        osDelay(70);
        
        Driver_I2C1.MasterTransmit(addr2, &reg_msb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr2, &vMSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        
        Driver_I2C1.MasterTransmit(addr2, &reg_lsb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr2, &vLSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);

        dist = (vMSB << 8) | vLSB; // On réutilise dist
        
				//ecriture le la valeur mesurée, du capteur 2, a l'aide du mutex
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        distance_global[3] = dist;  
        osMutexRelease(ID_Mutex_Data);

        osDelay(100);
    }
}

/*----------------------------------------------------------------------------
 * Tache LED 
 *---------------------------------------------------------------------------*/
static void tache_LED (void *argument) {
    uint16_t msg_recu; // Réception de la structure
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    
    while(1) {
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        msg_recu = distance_global[1];  
        osMutexRelease(ID_Mutex_Data);
            
				// Eteint les LEDs dédiées au capteur 1 (ex: Rouge et Orange)
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
				
				if(msg_recu > 0x0010) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);// Rouge
				if(msg_recu > 0x0020) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);// Orange
             
            
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        msg_recu = distance_global[3];  
        osMutexRelease(ID_Mutex_Data);

				// Eteint les LEDs dédiées au capteur 2 (ex: Verte et Bleue)
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);
				
				if(msg_recu > 0x0010) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);// Verte
				if(msg_recu > 0x0020) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // Bleue
            
    osDelay(100); 
		} 
}

/*----------------------------------------------------------------------------
 * Tache CAN 
 *---------------------------------------------------------------------------*/
static void tache_CAN (void *argument) {
	ARM_CAN_MSG_INFO tx_msg_info;
	
	uint16_t msg_recu;
	uint8_t data[2];
	
	tx_msg_info.id = ARM_CAN_STANDARD_ID(0x110); // ID CAN
	tx_msg_info.rtr = 0; // 0 = trame DATA
	
	while (1) {
		// Envoie de la valeur mesuré du capteur 1
		osMutexAcquire(ID_Mutex_Data, osWaitForever);
		msg_recu = distance_global[1];  
		osMutexRelease(ID_Mutex_Data);
		
		data[0]=(msg_recu>>8);
		data[1]=msg_recu;
		
		// envoi CAN
		Driver_CAN2.MessageSend(2, &tx_msg_info, data, 2);
		// Sommeil attente fin envoi sur EV1
		osThreadFlagsWait(CAN_EVENT_DONE,osFlagsWaitAny, osWaitForever);
		
		//envoie periodique toute les 50ms
		osDelay(500);
	}
}

osThreadAttr_t configT1 = {.priority = osPriorityAboveNormal};
osThreadAttr_t configT2 = {.priority = osPriorityBelowNormal};

int main (void) {
    SystemCoreClockUpdate();
    
		//Initialisations
		init_CAN();
    init_I2C();
		init_LED();
    
    osKernelInitialize();
		
		ID_Mutex_Data = osMutexNew(NULL);
		
    //Initialisation des tache
    ID_tacheI2C = osThreadNew(tache_I2C, NULL, &configT1);
		ID_tacheCAN = osThreadNew(tache_CAN, NULL, &configT1);
    ID_tacheLED = osThreadNew(tache_LED, NULL, &configT2);
    
		
		
    osKernelStart();
    
    for (;;) {} 
}
