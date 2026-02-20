#include "string.h"                          
#include "stm32f4xx.h"                  
#include "Driver_USART.h"               
#include "stm32f4xx_hal.h"              

extern ARM_DRIVER_USART Driver_USART2;

// ==========================================================
// LA LISTE BLANCHE (WHITELIST)
// Seul ce badge est autorisé désormais.
// ==========================================================
uint8_t badge1[14] = {0x02 ,0x30 ,0x38 ,0x30 ,0x30 ,0x38 ,0x43 ,0x32 ,0x33 ,0x45 ,0x39 ,0x34 ,0x45 ,0x03}; 
uint8_t badge2[14]={0x02 ,0x34 ,0x44 ,0x30 ,0x30 ,0x31 ,0x33 ,0x45 ,0x32 ,0x44 ,0x38 ,0x36 ,0x34 ,0x03};
void Init_GPIO_LEDs(void) {
    // 1. Activer l'horloge du Port D
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 2. Configuration des LEDs (PD12=Verte, PD14=Rouge)
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Sortie Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // Pas de résistance interne
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Vitesse basse
    
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Init_UART(void){
    Driver_USART2.Initialize(NULL); 
    Driver_USART2.PowerControl(ARM_POWER_FULL); 
    Driver_USART2.Control( ARM_USART_MODE_ASYNCHRONOUS |
                        ARM_USART_DATA_BITS_8 |
                        ARM_USART_STOP_BITS_1 |
                        ARM_USART_PARITY_NONE |
                        ARM_USART_FLOW_CONTROL_NONE ,
                        9600); // Vitesse configurée à 9600 bauds
    Driver_USART2.Control(ARM_USART_CONTROL_TX, 1); 
    Driver_USART2.Control(ARM_USART_CONTROL_RX, 1); 
}
// ==========================================================
// GESTIONNAIRE D'INTERRUPTION DE L'HORLOGE SYSTÈME (SysTick)
// Indispensable pour que HAL_Delay() fonctionne !
// ==========================================================

int main (void){
    uint8_t tab[15];  
    
    // Initialisation du système HAL (active le SysTick pour HAL_Delay)
    HAL_Init(); 
    Init_GPIO_LEDs();
		Init_UART();

    while (1){
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Bascule la rouge
			HAL_Delay(500); // Pause 0.5s
        // ETAT D'ATTENTE : On allume la LED Rouge pour signaler que la carte écoute
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        
        memset(tab, 0, sizeof(tab));
        
        // Lancement de la réception de 14 octets
        Driver_USART2.Receive(tab, 14);
        
        // Le programme bloque ici tant qu'il n'a pas reçu EXACTEMENT 14 octets
        while(Driver_USART2.GetStatus().rx_busy == 1);  

        // LECTURE RÉUSSIE : On éteint la LED Rouge
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

        // COMPARAISON : On vérifie UNIQUEMENT badge1
        if (memcmp(tab, badge1, 14) == 0) 
        {
            // ACCÈS VALIDÉ : La LED Verte s'allume pendant 2 secondes
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_Delay(2000); 
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        } 
        else 
        {
            // ACCÈS REFUSÉ (Badge inconnu ou badge2) : La LED Rouge clignote
            for(int i = 0; i < 4; i++) {
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
                HAL_Delay(250); 
            }
        }

        // Renvoi de la trame vers le PC (Terminal) pour vérifier ce qui a été lu
        Driver_USART2.Send(tab, 14);
        while(Driver_USART2.GetStatus().tx_busy == 1);
    }   
}
// Indispensable pour le fonctionnement tu code test.
void SysTick_Handler(void) {
    HAL_IncTick(); // Incrémente le compteur de 1 milliseconde
}
