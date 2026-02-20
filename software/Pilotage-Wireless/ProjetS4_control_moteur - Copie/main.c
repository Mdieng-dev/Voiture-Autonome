#include "LPC17xx.h"                    // Device header
#include "GPIO_LPC17xx.h"               // Device:GPIO


void Init_motor (void)
{
    // Activation de la puissance pour le bloc PWM1
    LPC_SC->PCONP |= (1 << 6); 

    // Configuration des broches P3.25 et P3.26 en mode PWM1
    // P3.26 = PWM1.3 (Direction Servo)
    // P3.25 = PWM1.2 (Sens/ vitesse Moteur via VNH5019)
    // Valeur binaire voulue : 0011 1100 0000 0000 0000 0000 (0x3C0000)
    LPC_PINCON->PINSEL7 |= (3 << 20) | (3 << 18); 

    // Configuration des broches P0.16 et P0.18 en sorties (GPIO)
    // P0.16 = INA (Sens moteur)
    // P0.18 = INB (Sens moteur)
    LPC_GPIO0->FIODIR |= (1 << 16) | (1 << 18); 
    LPC_GPIO0->FIOCLR = (1 << 16) | (1 << 18); // Moteur à l'arrêt (00 ou 11 voir tableau)

    // Configuration de la fréquence (50 Hz)
    LPC_PWM1->PR = 0;               // Pas de pré-diviseur
    LPC_PWM1->MR0 = 500000 - 1;     // 25 000 000 % 50 = 500 000

    // Configuration du compteur et des sorties
    LPC_PWM1->MCR |= (1 << 1);      // Reset du compteur sur MR0
    // Activation des ports PWM1.2 (bit 10) et PWM1.3 (bit 11)
    LPC_PWM1->PCR |= (1 << 10) | (1 << 11); 
    
    // Valeurs initiales (Rapport cyclique)
    LPC_PWM1->MR3 = 37500;          // Direction : Milieu (1.5ms) PWM1.3
    LPC_PWM1->MR2 = 0;              // Vitesse : 0 (Arrêt) PWM1.2

    // Validation des changements (LATCH)
    // On valide MR0 (bit 0), MR2 (bit 2) et MR3 (bit 3) -> 1101 binaire = 0x0D
    LPC_PWM1->LER = 0x0D;  

    // Démarrage du PWM
    // Bit 0 = Counter Enable, Bit 3 = PWM Enable
    LPC_PWM1->TCR = 0x09;  
}

void Gerer_Propulsion (int vitesse_moyenne){

	// --- CAS 1 : AVANCER (P1.24 appuyé) ---
	if ((LPC_GPIO1->FIOPIN3 & (1 << 0))== 0) {
		// Sens Avant : INA=1, INB=0
		LPC_GPIO0->FIOPIN2 = 0x01;
		
		//Verification
		LPC_GPIO1->FIOPIN3 = 0x10; //LED  P1.28
	
		// Vitesse
		LPC_PWM1->MR2 = vitesse_moyenne;
		LPC_PWM1->LER |= (1 << 2); // Valider Vitesse
	}
	
	// --- CAS 2 : RECULER (P1.26 appuyé) ---
	else if ((LPC_GPIO1->FIOPIN3 & (1 << 2))==0 ) {
		// Sens Arrière : INA=0, INB=1
		LPC_GPIO0->FIOPIN2 = 0x04;
		
		//Verification
		LPC_GPIO1->FIOPIN3 = 0x20; //LED  P1.29
	
		// Vitesse
		LPC_PWM1->MR2 = vitesse_moyenne;
		LPC_PWM1->LER |= (1 << 2); // Valider Vitesse
	}
	
	// --- CAS 3 : RIEN (Arrêt) ---
	else {
		// Couper la puissance (MR2 = 0)
		LPC_PWM1->MR2 = 0;
		LPC_PWM1->LER |= (1 << 2); // Valider l'arrêt		
		
		LPC_GPIO0->FIOPIN2 = 0x05; // Par sécurité Brake : INA=1 et INB=1
		
		//Verification
		LPC_GPIO1->FIOPIN3 = 0x80; //LED  P1.31
	}
}



void Gerer_Direction (void) {
    
    // CAS 1 : GAUCHE (P1.23 / Bit 1 à 0)
    if ((LPC_GPIO1->FIOPIN2 & (1 << 7)) == 0) {
        LPC_PWM1->MR3 = 25000; // Position Gauche (1ms)
        LPC_PWM1->LER |= (1 << 3); // Valider MR3
				
				LPC_GPIO2->FIOPIN0 = 0x40;
    }
    
    // CAS 2 : DROITE (P1.25 / Bit 3 à 0)
    else if ((LPC_GPIO1->FIOPIN3 & (1 << 1)) == 0) {
        LPC_PWM1->MR3 = 50000; // Position Droite (2ms)
        LPC_PWM1->LER |= (1 << 3); // Valider MR3
			
				LPC_GPIO2->FIOPIN0 = 0x20;
    }
    
    // CAS 3 : TOUT DROIT (Aucun bouton directionnel)
    else {
        LPC_PWM1->MR3 = 37500; // Position Centre (1.5ms)
        LPC_PWM1->LER |= (1 << 3); // Valider MR3
			
				LPC_GPIO2->FIOPIN0 = 0x10;
    }
}


int main(void)		{
	// Déclaration des variables
	int vitesse_moyenne=250000;
	
	LPC_GPIO1->FIODIR3 &= ~(0x05); // Init joystick  P1.24 et P1.26
	LPC_GPIO1->FIODIR3 |= 0xB0; // Init LED  P1.28 et P1.29 et P1.31
	LPC_GPIO2->FIODIR0 = LPC_GPIO2->FIODIR0 | 0x7C;	// FIO2DIR0 pour 5 LEDs LED0 = P2.6 ... LED4 = P2.2  en sortie
	
	Init_motor();
	
	while (1) {
		Gerer_Propulsion (vitesse_moyenne);
		Gerer_Direction();
	}

	return 0;
}


