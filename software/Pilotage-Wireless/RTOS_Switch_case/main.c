#include "LPC17xx.h"
#include "cmsis_os2.h"
#include "Board_Joystick.h"
#include "Board_LED.h"
#include "Board_Buttons.h"

//  Configuration 
#define PWM_PERIOD      2500    // Période PWM Moteur
#define SERVO_PERIOD    500000  // Période Servo (20ms @ 25MHz)
#define VITESSE_MOY     1250    // 50% de puissance

// Valeurs pour le Servo (Timer0 MR1)
#define GAUCHE         	25000   // 1ms
#define CENTRE         	37500   // 1.5ms
#define DROITE					50000   // 2ms

#define INT0						(1U<<8)

osThreadId_t ID_thread_motor ;
osThreadAttr_t configT1 = {.priority=osPriorityNormal};


//  Initialisation Hardware 
void Init_Motor(void) {
    // Configuration Moteur (PWM1.2 sur P2.1 + Sens sur P0.16/17)
    LPC_SC->PCONP |= (1 << 6);          
    LPC_PINCON->PINSEL7 |= (3 << 18);     // P3.25 en mode PWM (Sens/ vitesse Moteur via VNH5019)
    LPC_GPIO0->FIODIR   |= (1 << 16) | (1 << 17); 
    
	  LPC_PWM1->PR = 0;               // Pas de pré-diviseur
    LPC_PWM1->MR0 = PWM_PERIOD - 1;     
    LPC_PWM1->MR2 = 0;                  // Vitesse 0 au départ
    LPC_PWM1->LER |= (1 << 0) | (1 << 2);
    LPC_PWM1->PCR |= (1 << 10);         // Activation sortie PWM1.2
    LPC_PWM1->TCR = 0x09;                               
}

void Init_Servo_Timer(void) {         
    // Configuration (Timer0 sur P3.26)     
		LPC_PINCON->PINSEL7 &= ~(3 << 20);     // P3.26 en mode GPIO (Sens/ vitesse Moteur via VNH5019)
    LPC_GPIO3->FIODIR |= (1 << 26);   // P3.26  
    LPC_SC->PCONP |= (1 << 1);      
		LPC_TIM0->PR = 0;  // le registre PR prend la valeur du prescaler
    LPC_TIM0->MR0 = SERVO_PERIOD;       
    LPC_TIM0->MR1 = CENTRE;             
    
		// Config timer interruption
		LPC_TIM0->MCR = 0x0B;               // Interruptions sur MR0 et MR1
		NVIC_SetPriority(TIMER0_IRQn, 10);
    NVIC_EnableIRQ(TIMER0_IRQn);
    LPC_TIM0->TCR = 1;     // Reset compteur             
}

// Gestion du signal Servo via interruption
void TIMER0_IRQHandler(void) {   // Simulation de PWM à 50 Hz
    if (LPC_TIM0->IR & (1 << 0)) {  // Si interruption sur MR0 
			LPC_GPIO3->FIOSET = (1 << 26); 
			LPC_TIM0->IR = (1 << 0); 
		}
    if (LPC_TIM0->IR & (1 << 1)) { // Si interruption sur MR1
			LPC_GPIO3->FIOCLR = (1 << 26); 
			LPC_TIM0->IR = (1 << 1); 
		}
}

void Action(int vitesse, int sens, uint32_t angle) {
    // Sens : 1=Avant, 2=Arrière, 0=Stop
    if (sens == 1) { 
			LPC_GPIO0->FIOSET = (1 << 16); 
			LPC_GPIO0->FIOCLR = (1 << 17); 
		}
    else if (sens == 2) { 
			LPC_GPIO0->FIOCLR = (1 << 16); 
			LPC_GPIO0->FIOSET = (1 << 17); 
		}
    else {
			LPC_GPIO0->FIOCLR = (1 << 16);
			LPC_GPIO0->FIOCLR = (1 << 17);
		}
		
    LPC_PWM1->MR2 = vitesse;
    LPC_PWM1->LER |= (1 << 2);
    LPC_TIM0->MR1 = angle;
}

// --- Tâche Unique ---
void thread_motor(void const *argument) {
    uint32_t etat;
    uint32_t BUTTON_INT0;		
		while (1) {
				BUTTON_INT0 = ((Buttons_GetState()&0x01)<<8); // Retourne 1 si INT0 pressé
				etat = Joystick_GetState()| BUTTON_INT0;
        // On récupère l'état complet du joystick (combinaison de bits)
        switch (etat) {
            
            case JOYSTICK_UP:
                Action(VITESSE_MOY, 1, CENTRE); // Avancer Tout Droit
                break;

            case JOYSTICK_RIGHT:
                Action(VITESSE_MOY, 1, DROITE); // Avancer Droite
                break;

            case JOYSTICK_LEFT:
                Action(VITESSE_MOY, 1, GAUCHE); // Avancer Gauche
                break;

            case JOYSTICK_DOWN:
                Action(VITESSE_MOY, 2, CENTRE); // Reculer Tout Droit
                break;

            case (INT0 | JOYSTICK_RIGHT):
                Action(VITESSE_MOY, 2, DROITE); // Reculer Droite
                break;

            case (INT0 | JOYSTICK_LEFT):
                Action(VITESSE_MOY, 2, GAUCHE); // Reculer Gauche
                break;

            default: 
                Action(0, 0, CENTRE);           // Arrêt total
                break;
        }
        osDelay(50);
    }
}


// Main 
int main(void) {
    SystemCoreClockUpdate();
    
    // Initialisation des drivers de la carte
    Joystick_Initialize();
    LED_Initialize();
    Buttons_Initialize(); 
		Init_Motor();
    Init_Servo_Timer();

    osKernelInitialize();
    
    // Création de thread
    ID_thread_motor = osThreadNew((osThreadFunc_t)thread_motor, NULL, &configT1);
    
    osKernelStart();
    while (1);
}