#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "Driver_USART.h"
#include <stdio.h>   
#include <string.h>  // Pour strncmp
#include "stm32f4xx.h"                  // Device header
#include "Driver_CAN.h"                 // CMSIS Driver:CAN
#include "Driver_SPI.h"
#include "Driver_I2C.h"                 // CMSIS Driver:I2C
#include  CMSIS_device_header



#define NB_CAPTEURS 4 // Modifié à 3 selon ta description textuelle
const uint8_t ADRESSES_I2C[NB_CAPTEURS] = {(0xE0>>1), (0xE2>>1), (0xE4>>1),(0xE8>>1)};
uint16_t distance_global[NB_CAPTEURS];
#define I2C_A                (&Driver_I2C1)
extern ARM_DRIVER_I2C        Driver_I2C1;
osThreadId_t ID_tacheI2C,ID_tacheCAN;
osThreadId_t ID_tacheLED; 
osMutexId_t ID_Mutex_Data; 




extern ARM_DRIVER_CAN Driver_CAN2;
extern ARM_DRIVER_USART Driver_USART3;
extern ARM_DRIVER_USART Driver_USART2;

osThreadId_t tid_GPS;
osThreadId_t envoi_GPS;
osMessageQueueId_t ID_BAL;

osStatus_t status;
int32_t stat;                      
int octet_recu;             
float latitude_recup, longitude_recup;
int fix_quality; 
volatile float latitude, longitude;
volatile float cbon=0.0;




// Définition des paramètres pour les LEDs
#define LUMINOSITE_ROUGE 5
#define NB_LEDS_PAR_COTE 6
#define TOTAL_LEDS 12
#define SPI_BUFFER_SIZE (4 + (TOTAL_LEDS * 4) + 4)
extern ARM_DRIVER_SPI Driver_SPI1;
volatile int action_manette = 0;
osThreadId_t id_bouton;
osThreadId_t id_can;
osThreadId_t id_leds;
uint8_t spi_buffer[SPI_BUFFER_SIZE];
uint32_t spi_idx = 0;



// Constantes Capteur liquide
#define NB_MESURES 11
#define PROF_MAX 4.8f
osThreadId_t thread_capteur;
osThreadId_t CAN_liquide;
const float tab_tension[NB_MESURES] = {0.0, 1.3, 1.53, 1.62, 1.69, 1.74, 1.77, 1.81, 1.84, 1.86, 1.88};
const float tab_hauteur[NB_MESURES] = {0.0, 0.5, 1.0,  1.5,  2.0,  2.5,  3.0,  3.5,  4.0,  4.5,  4.8};
osThreadId_t ID_tache_CAN_liquide;

float niveau_pourcent;


// +++++++++++++++++++++++++++++ GPS +++++++++++++++++++++++++++++++++++++
//FONCTION CALLBACK 
void GPS_callback(uint32_t event) {
		static char trame_gps[100],idx=0;
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
		
				if ((octet_recu == '$' && (idx < 99))|| idx > 0) {
								trame_gps[idx++] = (char)octet_recu;}
        else;
            // Si on reçoit la fin de ligne
        if (octet_recu == '\n') {
								osMessageQueuePut (ID_BAL, &trame_gps, 0, 0);
                //osThreadFlagsSet(tid_GPS, 0x01); // On réveille la tâche de traitement
								idx = 0;}
        else;
        }
    
		Driver_USART3.Receive(&octet_recu, 1);          // On relance la réception de l'octet suivant
		}

//FONCTION CALLBACK 
/*void ENVOI_GPS_callback (uint32_t obj_idx, uint32_t event) {
    if (event & ARM_CAN_EVENT_SEND_COMPLETE) {
        osThreadFlagsSet(envoi_GPS, 0x02);
        Anes = 1.0; 
    }
}*/
//traitement de la trame 
void Thread_Traitement_GPS(void *argument) {
		int deg_lat,deg_lon;
		float min_lat,min_lon;
		char trame_recu[100];
		unsigned char D_GPS_CAN[8];
		ARM_CAN_MSG_INFO tx_msg_info;
		tx_msg_info.id = ARM_CAN_STANDARD_ID (0x170);
//		tx_msg_info.rtr = 0; // 0 = trame DATA
//		tx_msg_info.dlc = 8;
    while (1) {
				status = osMessageQueueGet (ID_BAL, trame_recu, NULL, osWaitForever);
			  //Anes = 0.0;
        // La tâche s'endort ici et attend le flag 0x01 envoyé par la Callback
        //osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
				if ( status == osOK){
						// analyse de la trame
						if (strncmp(trame_recu, "$GPGGA", 6) == 0) {
								sscanf(trame_recu, "$GPGGA,%*f,%f,%*c,%f,%*c,%d", &latitude_recup, &longitude_recup, &fix_quality);
						
								if (fix_quality > 0) {
										// Latitude
										deg_lat = (int)(latitude_recup / 100);             // Récupère la partie entiere 48,75058
										min_lat = latitude_recup - (deg_lat * 100);        // Récupère le reste 
										latitude = (float)deg_lat + (min_lat / 60.0);      // on calcule la vrai valeur

										// Longitude
										deg_lon = (int)(longitude_recup / 100);            
										min_lon = longitude_recup - (deg_lon * 100);       
										longitude = (float)deg_lon + (min_lon / 60.0);
										osThreadFlagsSet(envoi_GPS, 0x02);
								}
							}
						}
	}
}
void envoi_donnes_GPS_CAN_CAN(void *argument){
	
		typedef struct {
			float latitude;
			float longitude;
		} donnees;
		
		donnees data_recues;
    ARM_CAN_MSG_INFO tx_msg_info;
    tx_msg_info.id = ARM_CAN_STANDARD_ID(0x170);
    tx_msg_info.rtr = 0;
    tx_msg_info.dlc = 8;

    while (1) {
        // Attend le signal de la tâche traitement
        osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever);
			
			  data_recues.latitude = latitude;
			  data_recues.longitude = longitude;

        // Préparation du buffer CAN
        //memcpy(&D_GPS_CAN[0], (const void*)&latitude, 4);
        //memcpy(&D_GPS_CAN[4], (const void*)&longitude, 4);

        // Envoi sur le bus CAN
        Driver_CAN2.MessageSend(2, &tx_msg_info, &data_recues, 8);
				//cbon += 1.0;
					
    }


}








// ++++++++++++++++++++++++++++++ LEDS +++++++++++++++++++++++++++++++++++++++

// Ajoute un octet dans le tableau qui sera envoyé par SPI
void SPI_Send(uint8_t data) {
    if (spi_idx < SPI_BUFFER_SIZE) {
        spi_buffer[spi_idx++] = data;
    }
}

// Prépare les 4 octets de couleur pour une seule LED
void Send_LED(uint8_t br, uint8_t r, uint8_t g, uint8_t b) {
    SPI_Send(0xE0 | br);
    SPI_Send(b);
    SPI_Send(g);
    SPI_Send(r);
}

// Calcule et envoie l'état de toutes les LEDs selon l'action demandée
void Update_LEDs(uint8_t mode, uint8_t etape) {
    spi_idx = 0;
    
    // Trame de début obligatoire pour les LEDs
    for(int i=0; i<4; i++) SPI_Send(0x00);

    for(int i=0; i<TOTAL_LEDS; i++) {
        // Mode 0 : pas d'action, on allume tout en rouge faible
        if (mode == 0) {
            Send_LED(LUMINOSITE_ROUGE, 255, 0, 0);
        }
        else {
            int est_cligno = 0;
            
            // On détermine si la LED actuelle fait partie des clignotants
            if (mode == 1 && i < NB_LEDS_PAR_COTE) est_cligno = 1;
            if (mode == 2 && i >= NB_LEDS_PAR_COTE) est_cligno = 1;
            if (mode == 3) est_cligno = 1;

            if (est_cligno) {
                // Calcule la position de la LED sur son côté
                uint8_t pos = (i < NB_LEDS_PAR_COTE) ? i : (i - NB_LEDS_PAR_COTE);
                
                // Animation : allume la LED en orange selon l'étape en cours
                if (pos == etape) {
                    Send_LED(31, 255, 120, 0);
                } else if (pos < etape) {
                    Send_LED(5, 255, 60, 0);
                } else {
                    Send_LED(0, 0, 0, 0);
                }
            } else {
                // Les autres LEDs restent en rouge faible
                Send_LED(LUMINOSITE_ROUGE, 255, 0, 0);
            }
        }
    }
    
    // Trame de fin pour valider l'envoi
    for(int i=0; i<4; i++) SPI_Send(0xFF);

    // Envoi des données préparées sur le bus SPI
    Driver_SPI1.Send(spi_buffer, spi_idx);
}

// Fonction appelée automatiquement par le matériel quand un message CAN arrive
void CAN_Callback(uint32_t obj_idx, uint32_t event) {
    if (event & ARM_CAN_EVENT_RECEIVE) {
        ARM_CAN_MSG_INFO rx_msg_info;
        uint8_t data_rx[8];
        if (Driver_CAN2.MessageRead(obj_idx, &rx_msg_info, data_rx, 8) > 0) {
            if (rx_msg_info.id == ARM_CAN_STANDARD_ID(0x128)) {
                action_manette = data_rx[4]; 
                osThreadFlagsSet(id_leds, 0x10); // <--- ON RÉVEILLE LE THREAD LED IMMÉDIATEMENT
            }
        }
    }
}

// Tâche gérant exclusivement l'animation visuelle
void Thread_LEDs(void *argument) {
    uint8_t etape = 0;
    while(1) {
        // Attend soit le signal CAN (0x10), soit un timeout de 40ms pour l'animation
        osThreadFlagsWait(0x10, osFlagsWaitAny, 40); 
        
        int mode_actuel = action_manette;
        
        if (mode_actuel > 0) {
            Update_LEDs(mode_actuel, etape);
            etape = (etape + 1) % 8;
        } else {
            etape = 0;
            Update_LEDs(0, 0);
        }
    }
}








// ++++++++++++++++++++++++++++++++ Ultrason son ++++++++++++++++++++++++++++++++

void init_LED(void){	
    __HAL_RCC_GPIOD_CLK_ENABLE(); 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
    GPIO_InitStruct.Pull = GPIO_NOPULL;            
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


void callbackI2C(uint32_t event) {
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        osThreadFlagsSet(ID_tacheI2C, 0x01);
    }
}

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
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        osDelay(70);
        
        Driver_I2C1.MasterTransmit(addr1, &reg_msb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr1, &vMSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        
        Driver_I2C1.MasterTransmit(addr1, &reg_lsb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr1, &vLSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        dist = (vMSB << 8) | vLSB;
				
				//ecriture le la valeur mesurée, du capteur 1, a l'aide du mutex
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        distance_global[1] = dist; 
        osMutexRelease(ID_Mutex_Data);
				
			
        // --- LECTURE CAPTEUR 2 ---
        Driver_I2C1.MasterTransmit(addr2, reg_cmd, 2, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        osDelay(70);
        
        Driver_I2C1.MasterTransmit(addr2, &reg_msb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr2, &vMSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        
        Driver_I2C1.MasterTransmit(addr2, &reg_lsb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr2, &vLSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        dist = (vMSB << 8) | vLSB; // On réutilise dist
        
				//ecriture le la valeur mesurée, du capteur 2, a l'aide du mutex
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        distance_global[3] = dist;  
        osMutexRelease(ID_Mutex_Data);

        osDelay(100);
    }
}

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
		osThreadFlagsSet(ID_tacheCAN, 0x01);	
		} 
}
static void tache_CAN (void *argument) {
	ARM_CAN_MSG_INFO tx_msg_info;
	
	uint16_t msg_recu;
	uint8_t data[2];
	
	tx_msg_info.id = ARM_CAN_STANDARD_ID(0x110); // ID CAN
	tx_msg_info.rtr = 0; // 0 = trame DATA
	
	while (1) {
		// Sommeil attente fin envoi sur EV1
		osThreadFlagsWait(0x01,osFlagsWaitAny, osWaitForever);
		// Envoie de la valeur mesuré du capteur 1
		osMutexAcquire(ID_Mutex_Data, osWaitForever);
		msg_recu = distance_global[1];  
		osMutexRelease(ID_Mutex_Data);
		
		data[0]=(msg_recu>>8);
		data[1]=msg_recu;
		
		// envoi CAN
		Driver_CAN2.MessageSend(2, &tx_msg_info, data, 2);

		
		//envoie periodique toute les 50ms
		osDelay(500);
	}
}






// ++++++++++++++++++++++++++++++++ capteur liquide ++++++++++++++++++++++++++++++++

// Configuration des registres
void init_capteur(void) {
    // Allumer les horloges (GPIOA et ADC1)
    RCC->AHB1ENR |= 1; 
    RCC->APB2ENR |= (1 << 8);

    // Mettre la broche PA4 en sortie 
    GPIOA->MODER |= (1 << 8);
    // On force à 0V 
    GPIOA->BSRR = (1 << 20); 

    // Mettre PA6 en entrée analogique pour lire la valeur
    GPIOA->MODER |= (3 << 12);

    // Paramétrage de l'ADC
    ADC1->CR2 |= 1;           // Activer l'ADC
    ADC1->SQR3 = 6;           // On lit PA6
    ADC1->SMPR2 |= (7 << 18); 
}

// Fonction pour trouver la hauteur d'eau à partir de la tension 
float get_hauteur(float v_mesuree) {
    // Sécurité si on déborde des valeurs du tableau
    if (v_mesuree <= tab_tension[0]) return tab_hauteur[0];
    if (v_mesuree >= tab_tension[NB_MESURES - 1]) return tab_hauteur[NB_MESURES - 1];

    // On cherche entre quelles valeurs on se trouve
    for (int i = 0; i < NB_MESURES - 1; i++) {
        if (v_mesuree >= tab_tension[i] && v_mesuree <= tab_tension[i + 1]) {
            // Calcul de la pente (y2 - y1) / (x2 - x1)
            float pente = (tab_hauteur[i + 1] - tab_hauteur[i]) / (tab_tension[i + 1] - tab_tension[i]);
            // Calcul final
            return tab_hauteur[i] + pente * (v_mesuree - tab_tension[i]);
        }
    }
    return 0.0f;
}

// La tâche 
void tache_lecture_eau(void *arg) {
    uint32_t val_adc;
    float v_capteur;
    float h_eau;
    float pourcent_calc;
    while(1) {
        // allume  capteur 
        GPIOA->BSRR = (1 << 4); 
        
        
        osDelay(10); 

        // Lancement d ADC
        ADC1->CR2 |= (1 << 30);
        
        while (!(ADC1->SR & (1 << 1))); 
        val_adc = ADC1->DR;

        //  On coupe PA4 = 0
        GPIOA->BSRR = (1 << 20); 

        // Calculs
        // Conversion 
        v_capteur = ((float)val_adc / 4095.0f) * 3.0f;
        h_eau = get_hauteur(v_capteur);
        
        // pourcentage
        pourcent_calc = (h_eau / PROF_MAX) * 100.0f; 
        niveau_pourcent = (pourcent_calc * 100.0f) / 60.0f; 
        
        // On bloque à 100 au cas où ça dépasse
        if (niveau_pourcent > 100.0f) {
            niveau_pourcent = 100.0f;
        }
        osDelay(500);
				osThreadFlagsSet(CAN_liquide, 0x02);
    }
}

void tache_CAN_liquide (void *argument) {
ARM_CAN_MSG_INFO tx_msg_info;
tx_msg_info.id = ARM_CAN_STANDARD_ID(0x250);
			typedef struct {
			float niveau_pourcent;
			} data_liquide;	
data_liquide liquide_donnees;
	
	while (1) {
		osThreadFlagsWait(0x02,osFlagsWaitAny, osWaitForever);
		
		liquide_donnees.niveau_pourcent = niveau_pourcent;
		// envoi CAN
		Driver_CAN2.MessageSend(2, &tx_msg_info, &liquide_donnees, 4);
		cbon +=1.0;
		osDelay(500);
	}
}


//MAIN
int main (void) {
    HAL_Init();
		SystemCoreClockUpdate();
    //	Init UART3
    Driver_USART3.Initialize(GPS_callback);
    Driver_USART3.PowerControl(ARM_POWER_FULL);
    Driver_USART3.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8       |
                          ARM_USART_PARITY_NONE       |
                          ARM_USART_STOP_BITS_1       |
                          ARM_USART_FLOW_CONTROL_NONE , 9600);
    
    Driver_USART3.Control(ARM_USART_CONTROL_RX, 1); // Active la réception
		Driver_USART3.Receive(&octet_recu, 1);
	
		    // Initialisation du SPI
    Driver_SPI1.Initialize(NULL);
    Driver_SPI1.PowerControl(ARM_POWER_FULL);
    Driver_SPI1.Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_UNUSED | ARM_SPI_DATA_BITS(8), 1000000);
    Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

	
	

    //	Init CAN2 Reception LEDs et envoie DOnnées GPS
    Driver_CAN2.Initialize(NULL,CAN_Callback);
    Driver_CAN2.PowerControl(ARM_POWER_FULL);  
    Driver_CAN2.SetMode(ARM_CAN_MODE_INITIALIZATION);
    Driver_CAN2.SetBitrate(ARM_CAN_BITRATE_NOMINAL, 125000, ARM_CAN_BIT_PROP_SEG(5U) | ARM_CAN_BIT_PHASE_SEG1(1U) | ARM_CAN_BIT_PHASE_SEG2(1U) | ARM_CAN_BIT_SJW(1U));
   
			// Initialisation I2C
    Driver_I2C1.Initialize(callbackI2C); 
    Driver_I2C1.PowerControl(ARM_POWER_FULL);
    Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD); 

		init_LED();
		
		
    //Configuration pour ne recevoir que l'ID 0x128 dans la boîte de réception 0
    Driver_CAN2.ObjectSetFilter(0, ARM_CAN_FILTER_ID_EXACT_ADD, ARM_CAN_STANDARD_ID(0x128), 0);
    Driver_CAN2.ObjectConfigure(0, ARM_CAN_OBJ_RX); // objet 0 pour la réception
		Driver_CAN2.ObjectConfigure(2,ARM_CAN_OBJ_TX); // Objet 2 pour émission
    Driver_CAN2.SetMode(ARM_CAN_MODE_NORMAL);
		
		
		
		init_capteur();
		
    osKernelInitialize();  
    // Création de la tâche
		ID_BAL = osMessageQueueNew(4, 100,NULL);
    tid_GPS = osThreadNew(Thread_Traitement_GPS, NULL, NULL);
		envoi_GPS= osThreadNew(envoi_donnes_GPS_CAN_CAN, NULL, NULL);
    

    id_leds   = osThreadNew(Thread_LEDs,   NULL, NULL);
		
		
		ID_Mutex_Data = osMutexNew(NULL);
		ID_tacheLED = osThreadNew(tache_LED, NULL, NULL);
		ID_tacheCAN = osThreadNew(tache_CAN, NULL, NULL);
		ID_tacheI2C = osThreadNew(tache_I2C, NULL, NULL);
		
		
		thread_capteur = osThreadNew(tache_lecture_eau, NULL, NULL);
		CAN_liquide = osThreadNew(tache_CAN_liquide, NULL, NULL);
    osKernelStart();
    while(1);
}














/*

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "Driver_USART.h"
#include <stdio.h>   
#include <string.h>  // Pour strncmp
#include "stm32f4xx.h"                  // Device header
#include "Driver_CAN.h"                 // CMSIS Driver:CAN
#include "Driver_SPI.h"
#include "Driver_I2C.h"                 // CMSIS Driver:I2C
#include  CMSIS_device_header



#define NB_CAPTEURS 4 // Modifié à 3 selon ta description textuelle
const uint8_t ADRESSES_I2C[NB_CAPTEURS] = {(0xE0>>1), (0xE2>>1), (0xE4>>1),(0xE8>>1)};
uint16_t distance_global[NB_CAPTEURS];
#define I2C_A                (&Driver_I2C1)
extern ARM_DRIVER_I2C        Driver_I2C1;
osThreadId_t ID_tacheI2C,ID_tacheCAN;
osThreadId_t ID_tacheLED; 
osMutexId_t ID_Mutex_Data; 




extern ARM_DRIVER_CAN Driver_CAN2;
extern ARM_DRIVER_USART Driver_USART3;
extern ARM_DRIVER_USART Driver_USART2;

osThreadId_t tid_GPS;
osThreadId_t envoi_GPS;
osMessageQueueId_t ID_BAL;

osStatus_t status;
int32_t stat;                      
int octet_recu;             
float latitude_recup, longitude_recup;
int fix_quality; 
volatile float latitude, longitude;
volatile float cbon=0.0;




// Définition des paramètres pour les LEDs
#define LUMINOSITE_ROUGE 5
#define NB_LEDS_PAR_COTE 6
#define TOTAL_LEDS 12
#define SPI_BUFFER_SIZE (4 + (TOTAL_LEDS * 4) + 4)
extern ARM_DRIVER_SPI Driver_SPI1;
volatile int action_manette = 0;
osThreadId_t id_bouton;
osThreadId_t id_can;
osThreadId_t id_leds;
uint8_t spi_buffer[SPI_BUFFER_SIZE];
uint32_t spi_idx = 0;




// +++++++++++++++++++++++++++++ GPS +++++++++++++++++++++++++++++++++++++
//FONCTION CALLBACK 
void GPS_callback(uint32_t event) {
		static char trame_gps[100],idx=0;
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
		
				if ((octet_recu == '$' && (idx < 99))|| idx > 0) {
								trame_gps[idx++] = (char)octet_recu;}
        else;
            // Si on reçoit la fin de ligne
        if (octet_recu == '\n') {
								osMessageQueuePut (ID_BAL, &trame_gps, 0, 0);
                //osThreadFlagsSet(tid_GPS, 0x01); // On réveille la tâche de traitement
								idx = 0;}
        else;
        }
    
		Driver_USART3.Receive(&octet_recu, 1);          // On relance la réception de l'octet suivant
		}

//FONCTION CALLBACK 
void ENVOI_GPS_callback (uint32_t obj_idx, uint32_t event) {
    if (event & ARM_CAN_EVENT_SEND_COMPLETE) {
        osThreadFlagsSet(envoi_GPS, 0x02);
        Anes = 1.0; 
    }
}
//traitement de la trame 
void Thread_Traitement_GPS(void *argument) {
		int deg_lat,deg_lon;
		float min_lat,min_lon;
		char trame_recu[100];
		unsigned char D_GPS_CAN[8];
		ARM_CAN_MSG_INFO tx_msg_info;
		tx_msg_info.id = ARM_CAN_STANDARD_ID (0x170);
//		tx_msg_info.rtr = 0; // 0 = trame DATA
//		tx_msg_info.dlc = 8;
    while (1) {
				status = osMessageQueueGet (ID_BAL, trame_recu, NULL, osWaitForever);
			  //Anes = 0.0;
        // La tâche s'endort ici et attend le flag 0x01 envoyé par la Callback
        //osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
				if ( status == osOK){
						// analyse de la trame
						if (strncmp(trame_recu, "$GPGGA", 6) == 0) {
								sscanf(trame_recu, "$GPGGA,%*f,%f,%*c,%f,%*c,%d", &latitude_recup, &longitude_recup, &fix_quality);
						
								if (fix_quality > 0) {
										// Latitude
										deg_lat = (int)(latitude_recup / 100);             // Récupère la partie entiere 48,75058
										min_lat = latitude_recup - (deg_lat * 100);        // Récupère le reste 
										latitude = (float)deg_lat + (min_lat / 60.0);      // on calcule la vrai valeur

										// Longitude
										deg_lon = (int)(longitude_recup / 100);            
										min_lon = longitude_recup - (deg_lon * 100);       
										longitude = (float)deg_lon + (min_lon / 60.0);
										osThreadFlagsSet(envoi_GPS, 0x02);
								}
							}
						}
	}
}
void envoi_donnes_GPS_CAN_CAN(void *argument){
	
		typedef struct {
			float latitude;
			float longitude;
		} donnees;
		
		donnees data_recues;
    ARM_CAN_MSG_INFO tx_msg_info;
    tx_msg_info.id = ARM_CAN_STANDARD_ID(0x170);
    tx_msg_info.rtr = 0;
    tx_msg_info.dlc = 8;

    while (1) {
        // Attend le signal de la tâche traitement
        osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever);
			
			  data_recues.latitude = latitude;
			  data_recues.longitude = longitude;

        // Préparation du buffer CAN
        //memcpy(&D_GPS_CAN[0], (const void*)&latitude, 4);
        //memcpy(&D_GPS_CAN[4], (const void*)&longitude, 4);

        // Envoi sur le bus CAN
        Driver_CAN2.MessageSend(2, &tx_msg_info, &data_recues, 8);
				//cbon += 1.0;
					
    }


}








// ++++++++++++++++++++++++++++++ LEDS +++++++++++++++++++++++++++++++++++++++

// Ajoute un octet dans le tableau qui sera envoyé par SPI
void SPI_Send(uint8_t data) {
    if (spi_idx < SPI_BUFFER_SIZE) {
        spi_buffer[spi_idx++] = data;
    }
}

// Prépare les 4 octets de couleur pour une seule LED
void Send_LED(uint8_t br, uint8_t r, uint8_t g, uint8_t b) {
    SPI_Send(0xE0 | br);
    SPI_Send(b);
    SPI_Send(g);
    SPI_Send(r);
}

// Calcule et envoie l'état de toutes les LEDs selon l'action demandée
void Update_LEDs(uint8_t mode, uint8_t etape) {
    spi_idx = 0;
    
    // Trame de début obligatoire pour les LEDs
    for(int i=0; i<4; i++) SPI_Send(0x00);

    for(int i=0; i<TOTAL_LEDS; i++) {
        // Mode 0 : pas d'action, on allume tout en rouge faible
        if (mode == 0) {
            Send_LED(LUMINOSITE_ROUGE, 255, 0, 0);
        }
        else {
            int est_cligno = 0;
            
            // On détermine si la LED actuelle fait partie des clignotants
            if (mode == 1 && i < NB_LEDS_PAR_COTE) est_cligno = 1;
            if (mode == 2 && i >= NB_LEDS_PAR_COTE) est_cligno = 1;
            if (mode == 3) est_cligno = 1;

            if (est_cligno) {
                // Calcule la position de la LED sur son côté
                uint8_t pos = (i < NB_LEDS_PAR_COTE) ? i : (i - NB_LEDS_PAR_COTE);
                
                // Animation : allume la LED en orange selon l'étape en cours
                if (pos == etape) {
                    Send_LED(31, 255, 120, 0);
                } else if (pos < etape) {
                    Send_LED(5, 255, 60, 0);
                } else {
                    Send_LED(0, 0, 0, 0);
                }
            } else {
                // Les autres LEDs restent en rouge faible
                Send_LED(LUMINOSITE_ROUGE, 255, 0, 0);
            }
        }
    }
    
    // Trame de fin pour valider l'envoi
    for(int i=0; i<4; i++) SPI_Send(0xFF);

    // Envoi des données préparées sur le bus SPI
    Driver_SPI1.Send(spi_buffer, spi_idx);
}

// Fonction appelée automatiquement par le matériel quand un message CAN arrive
void CAN_Callback(uint32_t obj_idx, uint32_t event) {
    if (event & ARM_CAN_EVENT_RECEIVE) {
        ARM_CAN_MSG_INFO rx_msg_info;
        uint8_t data_rx[8];
        if (Driver_CAN2.MessageRead(obj_idx, &rx_msg_info, data_rx, 8) > 0) {
            if (rx_msg_info.id == ARM_CAN_STANDARD_ID(0x128)) {
                action_manette = data_rx[4]; 
                osThreadFlagsSet(id_leds, 0x10); // <--- ON RÉVEILLE LE THREAD LED IMMÉDIATEMENT
            }
        }
    }
}

// Tâche gérant exclusivement l'animation visuelle
void Thread_LEDs(void *argument) {
    uint8_t etape = 0;
    while(1) {
        // Attend soit le signal CAN (0x10), soit un timeout de 40ms pour l'animation
        osThreadFlagsWait(0x10, osFlagsWaitAny, 40); 
        
        int mode_actuel = action_manette;
        
        if (mode_actuel > 0) {
            Update_LEDs(mode_actuel, etape);
            etape = (etape + 1) % 8;
        } else {
            etape = 0;
            Update_LEDs(0, 0);
        }
    }
}








// ++++++++++++++++++++++++++++++++ Ultrason son ++++++++++++++++++++++++++++++++

void init_LED(void){	
    __HAL_RCC_GPIOD_CLK_ENABLE(); 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   
    GPIO_InitStruct.Pull = GPIO_NOPULL;            
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


void callbackI2C(uint32_t event) {
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        osThreadFlagsSet(ID_tacheI2C, 0x01);
    }
}

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
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        osDelay(70);
        
        Driver_I2C1.MasterTransmit(addr1, &reg_msb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr1, &vMSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        
        Driver_I2C1.MasterTransmit(addr1, &reg_lsb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr1, &vLSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        dist = (vMSB << 8) | vLSB;
				
				//ecriture le la valeur mesurée, du capteur 1, a l'aide du mutex
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        distance_global[1] = dist; 
        osMutexRelease(ID_Mutex_Data);
				
			
        // --- LECTURE CAPTEUR 2 ---
        Driver_I2C1.MasterTransmit(addr2, reg_cmd, 2, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        osDelay(70);
        
        Driver_I2C1.MasterTransmit(addr2, &reg_msb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr2, &vMSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        
        Driver_I2C1.MasterTransmit(addr2, &reg_lsb, 1, true);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr2, &vLSB, 1, false);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        dist = (vMSB << 8) | vLSB; // On réutilise dist
        
				//ecriture le la valeur mesurée, du capteur 2, a l'aide du mutex
				osMutexAcquire(ID_Mutex_Data, osWaitForever);
        distance_global[3] = dist;  
        osMutexRelease(ID_Mutex_Data);

        osDelay(100);
    }
}

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
		osThreadFlagsSet(ID_tacheCAN, 0x01);	
		} 
}
static void tache_CAN (void *argument) {
	ARM_CAN_MSG_INFO tx_msg_info;
	
	uint16_t msg_recu;
	uint8_t data[2];
	
	tx_msg_info.id = ARM_CAN_STANDARD_ID(0x110); // ID CAN
	tx_msg_info.rtr = 0; // 0 = trame DATA
	
	while (1) {
		// Sommeil attente fin envoi sur EV1
		osThreadFlagsWait(0x01,osFlagsWaitAny, osWaitForever);
		// Envoie de la valeur mesuré du capteur 1
		osMutexAcquire(ID_Mutex_Data, osWaitForever);
		msg_recu = distance_global[1];  
		osMutexRelease(ID_Mutex_Data);
		
		data[0]=(msg_recu>>8);
		data[1]=msg_recu;
		
		// envoi CAN
		Driver_CAN2.MessageSend(2, &tx_msg_info, data, 2);

		cbon +=1.0;
		//envoie periodique toute les 50ms
		osDelay(500);
	}
}



//MAIN
int main (void) {
    HAL_Init();
		SystemCoreClockUpdate();
    //	Init UART3
    Driver_USART3.Initialize(GPS_callback);
    Driver_USART3.PowerControl(ARM_POWER_FULL);
    Driver_USART3.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8       |
                          ARM_USART_PARITY_NONE       |
                          ARM_USART_STOP_BITS_1       |
                          ARM_USART_FLOW_CONTROL_NONE , 9600);
    
    Driver_USART3.Control(ARM_USART_CONTROL_RX, 1); // Active la réception
		Driver_USART3.Receive(&octet_recu, 1);
	
		    // Initialisation du SPI
    Driver_SPI1.Initialize(NULL);
    Driver_SPI1.PowerControl(ARM_POWER_FULL);
    Driver_SPI1.Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_UNUSED | ARM_SPI_DATA_BITS(8), 1000000);
    Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

	
	

    //	Init CAN2 Reception LEDs et envoie DOnnées GPS
    Driver_CAN2.Initialize(NULL,CAN_Callback);
    Driver_CAN2.PowerControl(ARM_POWER_FULL);  
    Driver_CAN2.SetMode(ARM_CAN_MODE_INITIALIZATION);
    Driver_CAN2.SetBitrate(ARM_CAN_BITRATE_NOMINAL, 125000, ARM_CAN_BIT_PROP_SEG(5U) | ARM_CAN_BIT_PHASE_SEG1(1U) | ARM_CAN_BIT_PHASE_SEG2(1U) | ARM_CAN_BIT_SJW(1U));
   
			// Initialisation I2C
    Driver_I2C1.Initialize(callbackI2C); 
    Driver_I2C1.PowerControl(ARM_POWER_FULL);
    Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD); 

		init_LED();
		
		
    //Configuration pour ne recevoir que l'ID 0x128 dans la boîte de réception 0
    Driver_CAN2.ObjectSetFilter(0, ARM_CAN_FILTER_ID_EXACT_ADD, ARM_CAN_STANDARD_ID(0x128), 0);
    Driver_CAN2.ObjectConfigure(0, ARM_CAN_OBJ_RX);
		Driver_CAN2.ObjectConfigure(2,ARM_CAN_OBJ_TX); // Objet 2 pour émission
    Driver_CAN2.SetMode(ARM_CAN_MODE_NORMAL);
		
		
		
		
		
    osKernelInitialize();  
    // Création de la tâche
		ID_BAL = osMessageQueueNew(4, 100,NULL);
    tid_GPS = osThreadNew(Thread_Traitement_GPS, NULL, NULL);
		envoi_GPS= osThreadNew(envoi_donnes_GPS_CAN_CAN, NULL, NULL);
    

    id_leds   = osThreadNew(Thread_LEDs,   NULL, NULL);
		
		
		ID_Mutex_Data = osMutexNew(NULL);
		ID_tacheLED = osThreadNew(tache_LED, NULL, NULL);
		ID_tacheCAN = osThreadNew(tache_CAN, NULL, NULL);
		ID_tacheI2C = osThreadNew(tache_I2C, NULL, NULL);
		
		
    osKernelStart();
    while(1);
} */