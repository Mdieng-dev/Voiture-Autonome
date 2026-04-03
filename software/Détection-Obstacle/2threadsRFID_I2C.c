/*----------------------------------------------------------------------------
 * Système RFID + Capteur US I2C Multitâche (STM32F407)
 *---------------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"                  
#include "Driver_USART.h"
#include "Driver_I2C.h"                 
#include "string.h"
#include "cmsis_os2.h"
#include "rtx_os.h"                     
#include "EventRecorder.h"              

// ==========================================================
// DÉCLARATION DES DRIVERS MATÉRIELS
// ==========================================================
extern ARM_DRIVER_USART Driver_USART2;
extern ARM_DRIVER_I2C   Driver_I2C1;

// ==========================================================
// DÉFINITIONS ET VARIABLES GLOBALES
// ==========================================================
// Flags de synchronisation
#define EVENT_I2C_DONE      0x01
#define EVENT_UART_DONE     0x01
#define FLAG_BADGE_OK       0x02 // Signal de la logique vers l'I2C

// Identifiants des tâches
osThreadId_t ID_tacheUART;
osThreadId_t ID_tacheLogic;
osThreadId_t ID_tacheI2C;
osThreadId_t ID_tacheLED;

// Files de messages (Mailboxes)
osMessageQueueId_t ID_BAL;       // Pour transmettre la distance (2 octets)
osMessageQueueId_t badgeQueue;   // Pour transmettre le badge lu (14 octets)

// La liste blanche
const uint8_t badge1[14] = {0x02,0x30,0x38,0x30,0x30,0x38,0x43,0x32,0x33,0x45,0x39,0x34,0x45,0x03}; 

// Attribut commun pour la priorité des threads
osThreadAttr_t configT1 = {.priority = osPriorityAboveNormal};

// ==========================================================
// FONCTIONS D'INITIALISATION MATÉRIELLE
// ==========================================================
void Init_GPIO_LEDs(void) {
    __HAL_RCC_GPIOD_CLK_ENABLE(); 
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
    GPIO_InitStruct.Pull = GPIO_NOPULL;           
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
    
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// ==========================================================
// CALLBACKS MATÉRIELS (Appelés par les interruptions)
// ==========================================================
void UART_SignalEvent(uint32_t event) {
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        // L'UART a fini de lire les 14 octets, on réveille la tâche UART
        osThreadFlagsSet(ID_tacheUART, EVENT_UART_DONE); 
    }
}

void I2C_SignalEvent(uint32_t event) {
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        // L'I2C a terminé sa transaction, on réveille la tâche I2C
        osThreadFlagsSet(ID_tacheI2C, EVENT_I2C_DONE); 
    }
}

// ==========================================================
// TÂCHE 1 : LECTURE DE L'UART (Mode Événementiel)
// ==========================================================
void Thread_UART() {
    uint8_t tab_local[14]; 
    
    while (1) {
        memset(tab_local, 0, sizeof(tab_local)); // Nettoyage
        
        // Lance la réception matérielle en tâche de fond
        Driver_USART2.Receive(tab_local, 14);
        
        // Endort la tâche jusqu'à ce que le Callback UART la réveille
        osThreadFlagsWait(EVENT_UART_DONE, osFlagsWaitAny, osWaitForever);
        
        // Envoie le badge lu dans la file pour analyse
        osMessageQueuePut(badgeQueue, tab_local, 0, osWaitForever);
    }
}

// ==========================================================
// TÂCHE 2 : LOGIQUE DE VÉRIFICATION DU BADGE
// ==========================================================
void Thread_Logic() {
    uint8_t buffer_reception[14]; 

    while (1) {
        // ETAT D'ATTENTE : LED Rouge allumée
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        
        // Attend qu'un badge soit lu par l'UART
        osMessageQueueGet(badgeQueue, buffer_reception, NULL, osWaitForever);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

        // Comparaison avec la liste blanche
        if (memcmp(buffer_reception, badge1, 14) == 0) {
            // BADGE OK : LED Verte
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            osDelay(1500); 
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
            
            // WAKE-UP : Déclenche la tâche I2C pour la mesure
            osThreadFlagsSet(ID_tacheI2C, FLAG_BADGE_OK);
        } 
        else {
            // BADGE REFUSÉ : Clignotement Rouge
            for(int i = 0; i < 4; i++) {
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
                osDelay(100); 
            }
        }
    }
}

// ==========================================================
// TÂCHE 3 : MESURE I2C (Uniquement si badge OK)
// ==========================================================
void tache_I2C () {
    uint8_t addr_slv = (0xE2 >> 1);
    uint8_t reg[2] = {0x00, 0x51}; // Demande de mesure
    uint8_t regMSB = 0x02;
    uint8_t regLSB = 0x03;
    uint8_t val_LSB, val_MSB;
    uint16_t val_mes;
    
		// 0. ATTENTE : On ne fait rien tant que le badge n'est pas validé !
		osThreadFlagsWait(FLAG_BADGE_OK, osFlagsWaitAny, osWaitForever);
		
    while(1) {
        
        
        
        // 1. Demande de mesure
        Driver_I2C1.MasterTransmit(addr_slv, reg, 2, false);
        osThreadFlagsWait(EVENT_I2C_DONE, osFlagsWaitAny, osWaitForever);
        osDelay(65); // Temps de conversion du capteur
        
        // 2. Lire le MSB
        Driver_I2C1.MasterTransmit(addr_slv, &regMSB, 1, true);
        osThreadFlagsWait(EVENT_I2C_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr_slv, &val_MSB, 1, false);
        osThreadFlagsWait(EVENT_I2C_DONE, osFlagsWaitAny, osWaitForever);
        
        // 3. Lire le LSB
        Driver_I2C1.MasterTransmit(addr_slv, &regLSB, 1, true);
        osThreadFlagsWait(EVENT_I2C_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr_slv, &val_LSB, 1, false);
        osThreadFlagsWait(EVENT_I2C_DONE, osFlagsWaitAny, osWaitForever);
        
        // 4. Calcul et envoi à la tâche d'affichage
        val_mes = (val_MSB << 8) | val_LSB;
        osMessageQueuePut(ID_BAL, &val_mes, 0, osWaitForever);
				
    }
}

// ==========================================================
// TÂCHE 4 : AFFICHAGE DE LA DISTANCE (LEDs)
// ==========================================================
void tache_LED (void) {
    uint16_t val_mes;
    
    while(1) {
        // Attend de recevoir une nouvelle distance
        if (osMessageQueueGet(ID_BAL, &val_mes, NULL, osWaitForever) == osOK) {
            
            // Extinction de tout
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
            
            // Affichage de la distance (bargraph approximatif)
            if(val_mes > 0x0025) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
            if(val_mes > 0x0020) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            if(val_mes > 0x0015) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
            if(val_mes > 0x0010) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
            
            osDelay(100); // Laisse l'affichage pendant 2 secondes
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
        }
    }
}

// ==========================================================
// MAIN
// ==========================================================
int main (void) {
    // 1. Initialisations Système Bas Niveau
    HAL_Init();                  
    SystemCoreClockUpdate();
    Init_GPIO_LEDs();    
    
    // 2. Initialisation Event Recorder
    EventRecorderInitialize(EventRecordAll, 1);
    EventRecorderStart();
    
    // 3. Initialisation Drivers (LIAISON DES CALLBACKS ICI !)
    Driver_USART2.Initialize(UART_SignalEvent); 
    Driver_USART2.PowerControl(ARM_POWER_FULL); 
    Driver_USART2.Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | 
                          ARM_USART_STOP_BITS_1 | ARM_USART_PARITY_NONE | 
                          ARM_USART_FLOW_CONTROL_NONE, 9600);
    Driver_USART2.Control(ARM_USART_CONTROL_TX, 1); 
    Driver_USART2.Control(ARM_USART_CONTROL_RX, 1); 
    
    Driver_I2C1.Initialize(I2C_SignalEvent); 
    Driver_I2C1.PowerControl(ARM_POWER_FULL); 
    Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);                                   
    
    // 4. Initialisation Kernel et Objets OS
    osKernelInitialize();                                 
    
    badgeQueue = osMessageQueueNew(4, 14, NULL); 
    ID_BAL = osMessageQueueNew(4, sizeof(uint16_t), NULL);
    
    ID_tacheUART = osThreadNew((osThreadFunc_t)Thread_UART, NULL, &configT1);        
    ID_tacheLogic = osThreadNew((osThreadFunc_t)Thread_Logic, NULL, &configT1);       
    ID_tacheI2C = osThreadNew((osThreadFunc_t)tache_I2C, NULL, &configT1);    
    ID_tacheLED = osThreadNew((osThreadFunc_t)tache_LED, NULL, &configT1);    
    
    // 5. Démarrage RTOS
    osKernelStart();                      
    
    while(1); // Le code n'arrive jamais ici
}
