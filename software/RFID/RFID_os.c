#include "stm32f4xx_hal.h"
#include "Driver_USART.h"
#include "string.h"
#include "cmsis_os2.h"

extern ARM_DRIVER_USART Driver_USART3;

// ==========================================================
// VARIABLES GLOBALES
// ==========================================================
// La liste blanche (stockée en mémoire Flash)
const uint8_t badge1[14] = {0x02 ,0x30 ,0x38 ,0x30 ,0x30 ,0x38 ,0x43 ,0x32 ,0x33 ,0x45 ,0x39 ,0x34 ,0x45 ,0x03}; 
// L'identifiant de la file de messages
osMessageQueueId_t badgeQueue;

// ==========================================================
// INITIALISATION DES LEDs (Port D, Broches 12 et 14)
// ==========================================================
void Init_GPIO_LEDs(void) {
    __HAL_RCC_GPIOD_CLK_ENABLE(); // Activation de l'horloge du Port D
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14; // Verte (12) et Rouge (14)
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
    GPIO_InitStruct.Pull = GPIO_NOPULL;           
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
    
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// ==========================================================
// TÂCHE 1 : LECTURE DE L'UART
// ==========================================================
void Thread_UART(void *argument) {
    uint8_t tab_local[15]; 
    
    //Initialisation de l'USART3
    Driver_USART3.Initialize(NULL); 
    Driver_USART3.PowerControl(ARM_POWER_FULL); 
    Driver_USART3.Control(ARM_USART_MODE_ASYNCHRONOUS | 
                          ARM_USART_DATA_BITS_8 | 
                          ARM_USART_STOP_BITS_1 | 
                          ARM_USART_PARITY_NONE | 
                          ARM_USART_FLOW_CONTROL_NONE, 9600);
    Driver_USART3.Control(ARM_USART_CONTROL_TX, 1); 
    Driver_USART3.Control(ARM_USART_CONTROL_RX, 1); 

    while (1) {
        memset(tab_local, 0, sizeof(tab_local)); // "Nettoyage" du buffer
        Driver_USART3.Control(ARM_USART_ABORT_RECEIVE, 0);

        //lecture de 14 octets
        Driver_USART3.Receive(tab_local, 14);
        while(Driver_USART3.GetStatus().rx_busy == 1) {
            osDelay(10); 
        }

        // Verification de début et fin de trame
        if (tab_local[0] == 0x02 && tab_local[13] == 0x03) {
            // Envoi de la trame dans la file de messages
            osMessageQueuePut(badgeQueue, tab_local, 0, 0);
        }
    }
}

// ==========================================================
// TÂCHE 2 : VÉRIFICATION ET TEST LEDs
// ==========================================================
void Thread_Logic(void *argument) {
    uint8_t buffer_reception[14]; 

    while (1) {
        // ETAT D'ATTENTE : LED Rouge allumée, LED Verte éteinte
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

        osMessageQueueGet(badgeQueue, buffer_reception, NULL, osWaitForever);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

        // Comparaison du badge avec la liste blanche choisi
        if (memcmp(buffer_reception, badge1, 14) == 0) 
        {
            // SI BADGE AUTORISÉ : LED Verte pendant 2 secondes
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            osDelay(1500); 
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        } 
        else 
        {
            // SI BADGE REFUSÉ : Clignotement de la LED Rouge
            for(int i = 0; i < 4; i++) {
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
                osDelay(100); 
            }
        }
    }
}

// ==========================================================
// MAIN PROGRAM 
// ==========================================================
int main(void) {

    HAL_Init(); 
    SystemCoreClockUpdate(); 
    Init_GPIO_LEDs();

    osKernelInitialize();

    badgeQueue = osMessageQueueNew(4, 14, NULL); // File de 4 messages de 14 octets
    osThreadNew(Thread_UART, NULL, NULL);        // Lancement de la tâche UART
    osThreadNew(Thread_Logic, NULL, NULL);       // Lancement de la tâche LEDs

    osKernelStart();
    while (1) 
		{
    }
}
