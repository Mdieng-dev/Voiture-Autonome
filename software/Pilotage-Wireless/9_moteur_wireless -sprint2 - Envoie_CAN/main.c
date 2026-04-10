#include "LPC17xx.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "Board_GLCD.h"                 // Board Support:Graphic LCD
#include "Board_ADC.h"                  // Board Support:A/D Converter
#include "Driver_CAN.h"                 // CMSIS Driver:CAN
#include "Driver_USART.h"
#include "Driver_SPI.h"
#include "GLCD_Fonts.h"                 // Board Support:Graphic LCD
#include "GLCD_Config.h"                // Board Support:Graphic LCD


extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_USART Driver_USART2;
extern ARM_DRIVER_CAN Driver_CAN1;
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;


// Configuration Moteur et Servo
#define PWM_PERIOD      2500    // Periode PWM Moteur
#define SERVO_PERIOD    500000  // Periode Servo (20ms a 25MHz)

#define VITESSE_MOY     1500    // Vitesse normale (50%)
#define VITESSE_MAX     2500    // Vitesse Turbo avec bouton Z (100%)
#define V_MAX_REELLE_KMH  11.32

// Valeurs pour le Servo (Timer0 MR1)
#define GAUCHE          30000   // 1ms
#define CENTRE          36000   // 1.5ms
#define DROITE          42000   // 2ms

// Valeurs pour le LIDAR
#define PI 3.14159265f
#define CENTRE_X 160
#define CENTRE_Y 120  
#define ECHELLE 2     // 1 pixel = 2 cm
#define DISTANCE_MIN_CM   5 // Distance minimale valide en mm
#define DISTANCE_MAX_CM   500  // Portée max LiDAR
#define DISTANCE_AFFICHAGE_CM 100 // NOUVEAU : n'affiche que ce qui est à moins d'1m

typedef struct { 
 int vitesse; 
} donnees;

// Structure Bluetooth
typedef struct {
    uint8_t x; 
    uint8_t y;
    bool z; 
    bool c;
    int err;
} Msg_t;

typedef struct {
			float M_distance;
			float M_angle;
			float M_qualite;
	} MaLettre;

osThreadId_t ID_thread_motor, ID_CANthreadT, ID_thread_bluetooth, ID_tache_recup,ID_tache_traitement,ID_tache_fonction,ID_tache_vitesse,ID_tache_2D,ID_tache_alerte;
osMessageQueueId_t q_id, ID_BAL_LCD, ID_BAL_2D, ID_BAL_ALERTE, ID_BAL_VITESSE_CAN;;
osMutexId_t ID_mut_GLCD;

uint8_t lidar_buffer0[7];// Taille du buffer sur 7 octets de réponse de confirmation du mode "Flux continu" (Voir S.P.D page 5,e)
uint8_t lidar_buffer1[5]; // Tableau du buffer sur 5 octets (Voir S.P.D page 5,e)
int packet_count = 0;

float distance = 0.0f; // On stocke la distance ici
float angle = 0.0f;    // On stocke l'angle ici
uint8_t qualite = 0;   // On stocke la qualité ici (fiabilité du signal renvoyée par le LiDAR)
donnees vitesse;

void Init_Motor(void) {
  // Activation du périphérique PWM1
    LPC_SC->PCONP |= (1 << 6);

    // Configuration des broches 
		LPC_PINCON->PINSEL7 |= (3 << 18);     

    LPC_PINCON->PINSEL4 &= ~(1 << 9);
    LPC_PINCON->PINSEL4 |=  (1 << 8);

    // Configuration des broches GPIO pour le sens du moteur 
    LPC_GPIO0->FIODIR |= (1 << 16) | (1 << 17);

    // Réglage de la fréquence à 10 kHz
    LPC_PWM1->PR = 0;                // Prescaler = 0
    LPC_PWM1->MR0 = PWM_PERIOD - 1;  // Période complète

    // Réglage du duty cycle 
    LPC_PWM1->MR2 = 0;  // vitesse
    LPC_PWM1->MR5 = PWM_PERIOD / 2;  // PWM1.5

    // Activation des sorties PWM
    LPC_PWM1->PCR |= (1 << 10) | (1 << 13);  // PWMENA2 et PWMENA5

    // Validation des changements pour MR0, MR2, MR5
    LPC_PWM1->LER |= (1 << 0) | (1 << 2) | (1 << 5);

    // Lancement du PWM
    LPC_PWM1->TCR = 0x09;  // Activer PWM + Counter + Reset                               
}

void Init_Servo_Timer(void) {         
    LPC_PINCON->PINSEL7 &= ~(3 << 20);     
    LPC_GPIO3->FIODIR |= (1 << 26);   
    LPC_SC->PCONP |= (1 << 1);      
    LPC_TIM0->PR = 0;  
    LPC_TIM0->MR0 = SERVO_PERIOD;       
    LPC_TIM0->MR1 = CENTRE;             
    
    LPC_TIM0->MCR = 0x0B;               
    NVIC_SetPriority(TIMER0_IRQn, 10);
    NVIC_EnableIRQ(TIMER0_IRQn);
    LPC_TIM0->TCR = 1;                  
}

void TIMER0_IRQHandler(void) {   
    if (LPC_TIM0->IR & (1 << 0)) {  
        LPC_GPIO3->FIOSET = (1 << 26); 
        LPC_TIM0->IR = (1 << 0); 
    }
    if (LPC_TIM0->IR & (1 << 1)) { 
        LPC_GPIO3->FIOCLR = (1 << 26); 
        LPC_TIM0->IR = (1 << 1); 
    }
}

void Action(int vitesse, int sens, uint32_t angle) {
    // Sens : 1=Avant, 2=Arriere, 0=Stop
    if (sens == 1) { 
        LPC_GPIO0->FIOSET = (1 << 16); 
        LPC_GPIO0->FIOCLR = (1 << 17); 
    }
    else if (sens == 2) { 
        LPC_GPIO0->FIOCLR = (1 << 16); 
        LPC_GPIO0->FIOSET = (1 << 17); 
    }
    else { // Stop
        LPC_GPIO0->FIOCLR = (1 << 16);
        LPC_GPIO0->FIOCLR = (1 << 17);
    }
    
    LPC_PWM1->MR2 = vitesse;
    LPC_PWM1->LER |= (1 << 2);
    LPC_TIM0->MR1 = angle;
}

void thread_bluetooth(void const * argument) {
    uint8_t sync_byte;
    uint8_t payload[4];
    Msg_t m;

    while(1) {
        m.err = 0;
        Driver_USART1.Receive(&sync_byte, 1);
        while(Driver_USART1.GetStatus().rx_busy) {
            osDelay(1); 
        }

        if (sync_byte == 0xFF) {
            Driver_USART1.Receive(payload, 4);
            uint32_t timeout = 20; 
            while(Driver_USART1.GetStatus().rx_busy && timeout > 0) {
                osDelay(1); 
                timeout--;
            }
            if (timeout == 0) m.err = 1; 

            if (!m.err) {
                m.x = payload[0];
                m.y = payload[1];
                m.z = payload[2];
                m.c = payload[3];
                osMessageQueuePut(q_id, &m, 0, 0);
            }
        }
    }
}

void thread_motor(void const * arg) {
    Msg_t m;
    int vitesse_actuelle = 0, sens_actuel = 0, vitesse_can;
		float kmh;
    uint32_t angle_actuel = CENTRE;

    while (1) {
        
        // Attente des donnees Bluetooth 
        if (osMessageQueueGet(q_id, &m, NULL, 500) == osOK) {
            
            // Axe X (Direction)
            if (m.x > 160) {
                angle_actuel = GAUCHE;
            } else if (m.x < 90) {
                angle_actuel = DROITE;
            } else {
                angle_actuel = CENTRE;
            }

            // Axe Y (Sens de marche)
            if (m.y > 160) {
                sens_actuel = 1; // Avance
            } else if (m.y < 90) {
                sens_actuel = 2; // Recule
            } else {
                sens_actuel = 0; // Stop (centre)
            }

            // Gestion de la vitesse
            if (sens_actuel != 0) {
                if (m.z) { // Si le bouton Z est appuye 
                    vitesse_actuelle = VITESSE_MAX; // Mode TURBO
                } else {
                    vitesse_actuelle = VITESSE_MOY; // Mode Normal
                }
           }else {
                vitesse_actuelle = 0;
            }
			
            // ENVOI DES COMMANDES AU MOTEUR
            Action(vitesse_actuelle, sens_actuel, angle_actuel);
						
						kmh = ((float)vitesse_actuelle * V_MAX_REELLE_KMH) / 2500.0;
            vitesse_can = (int)(kmh * 100.0); 
						vitesse.vitesse=vitesse_can;
						
        } else {
            // Failsafe : si perte de connexion Bluetooth pendant 500ms, tout s arrete
            Action(0, 0, CENTRE);
        }
				osThreadFlagsSet (ID_CANthreadT, 0x01);

    }
}


// CAN2 utilisé pour émission
void InitCan1 (void) 
{
		Driver_CAN1.Initialize(NULL,NULL);
		
		// Code d'initialisation du CAN2 en TX
		Driver_CAN1.PowerControl(ARM_POWER_FULL);
		
		Driver_CAN1.SetMode(ARM_CAN_MODE_INITIALIZATION);
		Driver_CAN1.SetBitrate( ARM_CAN_BITRATE_NOMINAL,
														125000,
														ARM_CAN_BIT_PROP_SEG(3U)   |         // Set propagation segment to 5 time quanta
														ARM_CAN_BIT_PHASE_SEG1(1U) |         // Set phase segment 1 to 1 time quantum (sample point at 87.5% of bit time)
														ARM_CAN_BIT_PHASE_SEG2(1U) |         // Set phase segment 2 to 1 time quantum (total bit is 8 time quanta long)
														ARM_CAN_BIT_SJW(1U));                // Resynchronization jump width is same as phase segment 2
														
		
		Driver_CAN1.ObjectConfigure(1,ARM_CAN_OBJ_TX);				// Objet 1 du CAN1 pour emission
		
		// Mettre ici les filtres ID d'emission sur objet 0

		Driver_CAN1.SetMode(ARM_CAN_MODE_NORMAL);					// fin init	
}

void CANthreadT(void)
{
		ARM_CAN_MSG_INFO                tx_msg_info;
		uint8_t data_buf[8];
		//uint32_t vitesse_recue;		
		InitCan1(); //initialise le CAN utilisé pour l'émission
		
		// Configuration de la trame
		tx_msg_info.id = ARM_CAN_STANDARD_ID(0x0f6);
		tx_msg_info.rtr = 0; // Trame de données (0 = DATA)
		
		while (1) {
			osThreadFlagsWait (0x01, osFlagsWaitAll, osWaitForever);
			
			data_buf[0] = (uint8_t)(vitesse.vitesse);        
			data_buf[1] = (uint8_t)(vitesse.vitesse >> 8); 
			data_buf[2] = (uint8_t)(vitesse.vitesse >> 16);
			data_buf[3] = (uint8_t)(vitesse.vitesse >> 24);
			// Code pour envoyer trame Id 0x0f6 
			Driver_CAN1.MessageSend(1, &tx_msg_info, data_buf, 4);
			osDelay(50); //attente 50ms
		}		
}

//Fonction d'interruption utilisée, pour activer la tâche de traitement dès qu'on reçoit un octet
void myUART_Callback(uint32_t event) 
{
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) 
    {
			osThreadFlagsSet(ID_tache_traitement,2); //Mise à 1 du flag n°1 de la tâche traitement
		}
}

void tache_traitement(void const*argument) // Acquisition Données LiDAR
{
			(void)argument;
			
			MaLettre msgE;
			
				while(1)
				{
				
				osThreadFlagsWait(2, osFlagsWaitAny, osWaitForever);// Attente mise à 1 Flag n°1
				
				//Détail sur position lidar_buffer1 page 16 	
				
        // Calcul de la distance par rapport au centre du LiDAR avec lidar_buffer1.
        uint16_t distance_brute = (lidar_buffer1[4] << 8) | lidar_buffer1[3]; // Découpage car l'UART ne peut envoyer que 8 bits à la fois
        distance = ((float)distance_brute / 40.0f); // Division par 40 pour obtenir des cm (Voir S.P.D page 5,g)

        // Calcul de l'angle par rapport au centre avec lidar_buffer1
        uint16_t angle_brut = (lidar_buffer1[2] << 8) | lidar_buffer1[1]; // Découpage car l'UART ne peut envoyer que 8 bits à la fois
        angle = (float)(angle_brut >> 1) / 64.0f;// Décalage (Voir S.P.D page 5,e) + Division par 64 pour obtenir des millimètres (Voir S.P.D page 5,g)			
			
        // Extraction de la qualité
        uint16_t qualite_brute = (lidar_buffer1[0] >> 2);
				qualite = ((float)qualite_brute*100.0f ) / 63.0f; // On décale car les deux premiers bits de cet octet sont des signaux de synchronisation (A ignorer) ((Voir S.P.D page 5,e)					
				
				// Relance de la réception sur le buffer des mesures (5 octets)
        Driver_USART1.Receive(lidar_buffer1, 5);	
		
				msgE.M_distance = distance;
				msgE.M_angle = angle;
				msgE.M_qualite = qualite;	
				
				osMessageQueuePut(ID_BAL_LCD, &msgE, 0, 0);
				osMessageQueuePut(ID_BAL_2D, &msgE, 0, 0);
				osMessageQueuePut(ID_BAL_ALERTE, &msgE, 0, 0);
				}
		}


void tache_recup (void const * argument) // Tache récupération & Affichage LCD 
{
			(void)argument;
			
  MaLettre msgR;
	float d;
	float a;
	float q;
	char tab[30];
	
	int seuil = 10;
	osStatus_t status;
	
				while(1)
				{			
				status = osMessageQueueGet(ID_BAL_LCD, &msgR, NULL, osWaitForever); // attente mail
             if (status == osOK) 
               {
									 osMutexAcquire(ID_mut_GLCD, osWaitForever);				
									 GLCD_DrawString(0,0,"Valeurs recuperees !:"); 
									 osMutexRelease(ID_mut_GLCD); 
									 
										d = msgR.M_distance;
									  a = msgR.M_angle;
										q = msgR.M_qualite;
						
									 if ((d < seuil))
									 {
									 osMutexAcquire(ID_mut_GLCD, osWaitForever);				 
									 GLCD_DrawString(0,72,"Obstacle proche !:"); 							 
									 sprintf(tab,"distance : %.2f cm",d);
									 GLCD_DrawString(0,96,tab);
										 
									 osMutexRelease(ID_mut_GLCD); 
									 }
									 
									 else 
									 {
									 
									 }

							}
		}
	}

void tache_vitesse (void const * argument) // Variation vitesse d'acquisition
{
		(void)argument;
		
		char etat23,etat25;
		LPC_GPIO1 -> FIODIR2 &= 0x7F; //Mise en entrée de 1.23
		LPC_GPIO1 -> FIODIR3 &= 0xFD; //Mise en entrée de 1.25

		while(1)
		{
		etat23 = LPC_GPIO1 -> FIOPIN2;
		etat25 = LPC_GPIO1 -> FIOPIN3;
		
			if((etat23 & 0x80)==0x00)
			{
//				osMutexAcquire(ID_mut_GLCD, osWaitForever);
//				GLCD_DrawString(0,24,"Vitesse +"); 
//				osMutexRelease(ID_mut_GLCD); 
				
				if(LPC_PWM1->MR5 < 1200) 
				{ // Sécurité pour ne pas dépasser MR0
                LPC_PWM1->MR5 += 100;
                LPC_PWM1->LER |= (1 << 5); // Applique le changement
        }
				osDelay(300);
			}
			
			if((etat25 & 0x02)==0x00)
			{
//				osMutexAcquire(ID_mut_GLCD, osWaitForever);
//				GLCD_DrawString(0,48,"Vitesse -"); 
//				osMutexRelease(ID_mut_GLCD); 
				
				if(LPC_PWM1->MR5 > 250) 
				{ // Sécurité pour baisser que si > 200, sinon calé
                LPC_PWM1->MR5 -= 100;
                LPC_PWM1->LER |= (1 << 5); // Applique le changement
        }
				osDelay(300);
			}
			

		
		}	
}

void tache_2D(void const * argument)
{
    (void)argument;
    MaLettre msgR;
    osStatus_t status;

    float angle_rad_corrige;
    float rayon_px;
    int x_px, y_px;
    float ancien_angle = -1.0f;  // Init à -1 pour forcer le premier dessin de croix
		int nouveau_tour;



    while(1)
    {
        status = osMessageQueueGet(ID_BAL_2D, &msgR, NULL, osWaitForever);

        if (status == osOK)
				{

					// Validation data
					if (msgR.M_distance < DISTANCE_MIN_CM || msgR.M_distance > DISTANCE_MAX_CM)     // Si ce qu'on détecte est entre 50mm ou > 5m
					{
							ancien_angle = msgR.M_angle; //On remet juste l'angle à jour pour ne pas rester sur celui de des distances invalides

					}
					
					else
					{
						//Détecttion d'un nouveau tour avec une marge de 10°
						// On détecte le passage 360°->0° 
						if (ancien_angle > 300.0f && msgR.M_angle < 10.0f) 
							{
									nouveau_tour = 1;  // VRAI : On vient de passer le 0°
							} 
						else 
							{
									nouveau_tour = 0;  // FAUX : On est encore dans le même tour
							}

						if (nouveau_tour || ancien_angle < 0.0f) // Pour premier passsage : force l'affichage de la croix
							{
									// Clear + Croix dans un SEUL bloc mutex ? pas de fenêtre entre les deux
									osMutexAcquire(ID_mut_GLCD, osWaitForever);
									GLCD_ClearScreen();
									// Croix centrale (LiDAR) redessinée immédiatement après le clear
									GLCD_SetForegroundColor(GLCD_COLOR_BLACK);
									//Ligne horizontale et verticale de la croix centrale
									GLCD_DrawHLine(CENTRE_X - 5, CENTRE_Y,     11); // 11 : 5pixels gauche + 5 picels droie + 1 central
									GLCD_DrawVLine(CENTRE_X,     CENTRE_Y - 5, 11);
									osMutexRelease(ID_mut_GLCD);
							}

						ancien_angle = msgR.M_angle; // Mise à jour de l'angle pour ne plus être à -1

						// Calcul positions
						angle_rad_corrige = (PI / 2.0f) - (msgR.M_angle * (PI / 180.0f)); //Conversion angle en radians et place le 0° en haut de l'écran au lieu d'être à droite
						rayon_px = msgR.M_distance / (float)ECHELLE; // Distance convertie en pixels 

						// On se place aux centres respectifs de X et Y
						x_px = CENTRE_X + (int)(rayon_px * cosf(angle_rad_corrige));
						y_px = CENTRE_Y - (int)(rayon_px * sinf(angle_rad_corrige)); // Signe moins car ici, sur le LCD Y diminiue quand on monte

						// affichage des points (obstacles) uniquement si <= Ditance affichage
						if (msgR.M_distance <= DISTANCE_AFFICHAGE_CM)
						{
							if (x_px >= 1 && x_px < 319 && y_px >= 1 && y_px < 239) //Vérification pour ne pas dépasser les bords de l'écran
							{
									osMutexAcquire(ID_mut_GLCD, osWaitForever);
									GLCD_SetForegroundColor(GLCD_COLOR_RED);
									GLCD_DrawPixel(x_px,     y_px);
									GLCD_DrawPixel(x_px + 1, y_px);
									GLCD_DrawPixel(x_px,     y_px + 1);
									GLCD_DrawPixel(x_px + 1, y_px + 1);
									osMutexRelease(ID_mut_GLCD);
							}
						}
					}

			}
    }
}
void tache_alerte(void const * argument)
{
	(void)argument;
	MaLettre msgR;

	osStatus_t status;
	float d;
	LPC_GPIO2 -> FIODIR0 |= 0xFC; // Mise en sortie de P2.2 à P2.6
	LPC_GPIO1 -> FIODIR3 |= 0xF0; // Mise en sortie de P1.28 à P1.31
	
	LPC_GPIO2 -> FIOPIN0 &= ~(0xFC); //Éteindre sortie de P2.2 à P2.6
	LPC_GPIO1 -> FIOPIN3 &= ~(0xF0); // Éteindre sortie de P1.28 à P1.31
	
	
				while(1)
				{			
				while(osMessageQueueGet(ID_BAL_ALERTE, &msgR, NULL, 0) == osOK); //tant qu'on a des choses dans la BAL, on reboucle jusqu'à ce que ça soit vide
				status = osMessageQueueGet(ID_BAL_ALERTE, &msgR, NULL, osWaitForever); // attente mail
	
             if (status == osOK) 
               {
								d = msgR.M_distance;
									if(d < 3) // si d < 3 cm
									{
										LPC_GPIO2 -> FIOPIN0 |= 0xFC;
										LPC_GPIO1 -> FIOPIN3 |= 0xF0;
										osDelay(500);
										
										LPC_GPIO2 -> FIOPIN0 &= ~(0xFC);
										LPC_GPIO1 -> FIOPIN3 &= ~(0xF0);
										osDelay(500);
										
										
									}
									else 
									{
										LPC_GPIO2 -> FIOPIN0 &= ~(0xFC);
										LPC_GPIO1 -> FIOPIN3 &= ~(0xF0);									
									}
							 }
				}
}

void Init_UART(void)
{
	// LIDAR
	Driver_USART2.Initialize(myUART_Callback); // Mode interruption, passage dans myUART_Callback quand un octet arrive
	Driver_USART2.PowerControl(ARM_POWER_FULL);
	Driver_USART2.Control(	ARM_USART_MODE_ASYNCHRONOUS |
							ARM_USART_DATA_BITS_8		|
							ARM_USART_STOP_BITS_1		|
							ARM_USART_PARITY_NONE		|
							ARM_USART_FLOW_CONTROL_NONE,
							115200);
	Driver_USART2.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART2.Control(ARM_USART_CONTROL_RX,1);

	//Bluetooth
	Driver_USART1.Initialize(NULL);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | 
												ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | 
												ARM_USART_FLOW_CONTROL_NONE, 115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);


}

//Configuration Envoi des commandes
void Lidar_Start_Scan(void) 
{
    uint8_t cmd_scan[] = {0xA5, 0x20}; //  0xA5 (= Start Flag) + 0x20(= Scan) (Voir S.P.D page 4,d)
    Driver_USART2.Send(cmd_scan, 2); //Envoi des 2 octets
}


int main(void) {
    
    SystemInit();
    SystemCoreClockUpdate();
    
		//Utilisation LED en test
		LPC_GPIO1->FIODIR3 |= 0x10;//Config LED P1.28 en sortie
		LPC_GPIO1->FIOPIN3 &= 0x00;

    // Initialisation Materielle
    Init_Motor();
    Init_Servo_Timer();
		Init_UART();


    osKernelInitialize();
		ADC_Initialize();
		GLCD_Initialize();
		GLCD_ClearScreen();
		GLCD_SetFont(&GLCD_Font_16x24);

    q_id = osMessageQueueNew(10, sizeof(Msg_t), NULL); 
		ID_BAL_VITESSE_CAN = osMessageQueueNew(5, sizeof(int), NULL);		// Création de thread
    ID_thread_bluetooth = osThreadNew((osThreadFunc_t)thread_bluetooth, NULL, NULL); // nunchuk
    ID_thread_motor = osThreadNew((osThreadFunc_t)thread_motor, NULL, NULL); // moteur
    ID_CANthreadT = osThreadNew((osThreadFunc_t)CANthreadT, NULL, NULL);    // tâche CAN2 envoi moteur
		
		//création de taches LIDAR
		ID_tache_traitement = osThreadNew ( (osThreadFunc_t )tache_traitement, NULL, NULL);
		ID_tache_recup = osThreadNew ( (osThreadFunc_t )tache_recup, NULL, NULL);
		ID_tache_vitesse = osThreadNew ( (osThreadFunc_t )tache_vitesse, NULL, NULL);
		ID_tache_2D = osThreadNew ( (osThreadFunc_t )tache_2D, NULL, NULL);
		ID_tache_alerte = osThreadNew ( (osThreadFunc_t )tache_alerte, NULL, NULL);

		ID_mut_GLCD = osMutexNew(NULL) ;
		ID_BAL_LCD = osMessageQueueNew(10, sizeof(MaLettre), NULL);
		ID_BAL_2D = osMessageQueueNew(10, sizeof(MaLettre), NULL);
		ID_BAL_ALERTE = osMessageQueueNew(10, sizeof(MaLettre), NULL);
		
		packet_count = 0;

//		Lidar_Start_Scan();//Commande de démarrage
//		
//		Driver_USART2.Receive(lidar_buffer0, 7);// Réception 7 octets de confirmation 
//		while(Driver_USART2.GetStatus().rx_busy); //Attendre que les 7 octets de confirmation soient traités
//		Driver_USART2.Receive(lidar_buffer1, 5);//Réception 5 octets de données

    
    osKernelStart();
    while (1);
}