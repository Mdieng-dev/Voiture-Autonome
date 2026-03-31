/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template - OPTIMISÉ TEMPS RÉEL
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "stm32f4xx.h"                  // Device header
#include "Driver_I2C.h"                 // CMSIS Driver:I2C
#include "RTE_Device.h"                 // Device:STM32Cube Framework:Classic
#include "rtx_os.h"                     // CMSIS:RTOS2:Keil RTX5&&Source
#include "EventRecorder.h"              // CMSIS-View:Event Recorder&&DAP
#include "stm32f4xx_hal.h"              // Device:STM32Cube HAL:Common

extern ARM_DRIVER_I2C        Driver_I2C1;
#define I2C_A                (&Driver_I2C1)

// Définition du nombre de capteurs et de leurs adresses
#define NB_CAPTEURS 3
const uint8_t ADRESSES_I2C[NB_CAPTEURS] = {(0xE0>>1),(0xE2>>1),(0xE4>>1),(0xE8>>1)};

// Structure du message envoyé dans }a Mailbox
typedef struct {
    uint8_t id_capteur;  // 0 pour le premier, 1 pour le deuxième, etc.
    uint16_t distance;   // Valeur mesurée
} MessageMesure_t;



osThreadId_t ID_tacheI2C;
osThreadId_t ID_tacheLED;
//ID BAL
osMessageQueueId_t ID_BAL;

// Définition d'un flag pour l'événement I2C
#define I2C_EVENT_DONE 0x01

/*----------------------------------------------------------------------------
 * Fonction de Callback I2C (Exécutée sous interruption)
 * Réveille la tâche I2C quand la transmission/réception est terminée
 *---------------------------------------------------------------------------*/
void I2C_SignalEvent(uint32_t event) {
    // Si le transfert est terminé avec succès
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        osThreadFlagsSet(ID_tacheI2C, I2C_EVENT_DONE); // Envoie le flag pour réveiller la tâche
    }
}

/*----------------------------------------------------------------------------
 * Tache I2C : mesure de distance capteurUS (cm)
 *---------------------------------------------------------------------------*/
static void tache_I2C (void *argument) {

		static uint8_t index = 0;
		uint8_t reg[10], regMSB[10], regLSB[10];
		uint8_t val_LSB, val_MSB;
		MessageMesure_t message;


		reg[0] = 0x00; // registre de commande 
		reg[1] = 0x51; // commande de mesure en cm

		//registre de mesure
		regMSB[0] = 0x02; // registre de mesure bit de poids fort
		regLSB[0] = 0x03; // registre de mesure bit de poids faible

		while(1) {
				
			uint8_t addr_slv = ADRESSES_I2C[index];
			
			// 1. Commande pour demander la mesure
			Driver_I2C1.MasterTransmit(addr_slv, reg, 2, false);
			osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever); // Attente passive

			osDelay(65); // 65ms d'attente passive pour laisser le capteur mesurer

			// 2. Lire le MSB
			Driver_I2C1.MasterTransmit(addr_slv, regMSB, 1, true); // true = sans stop
			osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever); // Attente passive

			Driver_I2C1.MasterReceive(addr_slv, &val_MSB, 1, false); // false = avec stop
			osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever); // Attente passive

			// 3. Lire le LSB
			Driver_I2C1.MasterTransmit(addr_slv, regLSB, 1, true); // true = sans stop
			osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever); // Attente passive

			Driver_I2C1.MasterReceive(addr_slv, &val_LSB, 1, false); // false = avec stop
			osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever); // Attente passive


			// Sauvegarde des 2 octets mesurés sous une variable de 16 bits
			message.distance = (val_MSB << 8) + val_LSB;
			message.id_capteur = addr_slv;
			// Envoie la valeur mesurée sur 16 bits à la mailbox (La priorité est à 0, pas NULL)
			osMessageQueuePut(ID_BAL, &message, 0, osWaitForever);


			// Passage au capteur suivant
			index = (index + 1) % NB_CAPTEURS;
			osDelay(100);
		}
}

/*----------------------------------------------------------------------------
 * Tache LED : affichage distance 
 *---------------------------------------------------------------------------*/
static void tache_LED (void *argument) {
    MessageMesure_t message;
    osStatus_t status;
		uint8_t ID=0;
    
		
    while(1) {
			// La tâche s'endort ici jusqu'à recevoir un message
			status = osMessageQueueGet(ID_BAL, &message, NULL, osWaitForever);						
			if (status == osOK) {
				if(message.id_capteur==ADRESSES_I2C[0]){
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
					
					// Affichage de la distance
					if(message.distance > 0x0025) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
					if(message.distance > 0x0020) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
					if(message.distance > 0x0015) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					if(message.distance > 0x0010) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
				}					
			}
		}
}

osThreadAttr_t configT1 = {.priority = osPriorityAboveNormal};

int main (void) {
    // System Initialization
    SystemCoreClockUpdate();
    EventRecorderInitialize (EventRecordAll, 1);
    
		//initialisation LEDs
		__HAL_RCC_GPIOD_CLK_ENABLE(); 
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
    GPIO_InitStruct.Pull = GPIO_NOPULL;           
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
    
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
    // Initialisation I2C avec ajout du CALLBACK pour supprimer l'attente active
    Driver_I2C1.Initialize(I2C_SignalEvent); 
    Driver_I2C1.PowerControl(ARM_POWER_FULL);
    Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD); // 100 kHz                                
    
    // Initialize mailbox (correction de la signature de osMessageQueueNew)
    ID_BAL = osMessageQueueNew(4, sizeof(MessageMesure_t), NULL); // File de 4 messages de 2 octets

    osKernelInitialize(); // Initialize CMSIS-RTOS
    
    // Création des threads (la signature standard demande void *argument, géré plus haut)
    ID_tacheI2C = osThreadNew(tache_I2C, NULL, &configT1);
    ID_tacheLED = osThreadNew(tache_LED, NULL, &configT1);
    
    osKernelStart(); // Start thread execution
    
    for (;;) {} // Fallback classique du main
}


