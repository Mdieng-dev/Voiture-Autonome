#include "LPC17xx.h"                    // Device header
#include "GPIO_LPC17xx.h"               // Device:GPIO

#define DROITE 0x01
#define GAUCHE 0x04
#define HAUT 0x80
#define BAS 0x02


void Init_motor (void);
void Init_GPIO ();
void avancer (int vitesse_moyenne);
void reculer(int vitesse_moyenne);
void arret();
void gauche();
void droite();
void centre ();
char lecture_joystick();
void eteindre_toutes_les_leds(void);


int main(void) {
    int vitesse_moyenne = 250000;
    Init_GPIO();
    Init_motor();
    
    // On s'assure que tout est éteint au démarrage
    eteindre_toutes_les_leds(); 
    
		while (1) {
        switch (lecture_joystick()){
            case 1: // AVANCER TOUT DROIT
                eteindre_toutes_les_leds();
                LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x40; // Allume LED P2.6
                
                avancer(vitesse_moyenne);
                centre();
                break;

            case 2: // AVANCER DROITE
                eteindre_toutes_les_leds();
                LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x20; // Allume LED P2.5
                
                avancer(vitesse_moyenne);
                droite();
                break;

            case 3: // AVANCER GAUCHE
                eteindre_toutes_les_leds();
                LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x10; // Allume LED P2.4
                
                avancer(vitesse_moyenne);
                gauche();
                break;

            case 4: // RECULER TOUT DROIT
                eteindre_toutes_les_leds();
                LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x08; // Allume LED P2.3
                
                reculer(vitesse_moyenne);
                centre();
                break;

            case 5: // RECULER DROITE
                eteindre_toutes_les_leds();
                LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x04; // Allume LED P2.2
                
                reculer(vitesse_moyenne);
                droite();
                break;

            case 6: // RECULER GAUCHE
                eteindre_toutes_les_leds();
                LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 | 0x80; // Allume LED P1.31
                
                reculer(vitesse_moyenne);
                gauche();
                break;

            default:
                eteindre_toutes_les_leds();
                LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 | 0x20; //  Allume LED P1.29
                
								arret();
                centre();
                break;
        }
    }
    return 0;
}


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

void Init_GPIO(void) {
    // --- CONFIGURATION DES ENTRÉES (Joystick et Bouton Poussoir) ---
    
    // Joystick : Droit (P1.24), Bas (P1.25), Gauche (P1.26)
    // FIODIR3 : Bits 0, 1 et 2 mis à 0 pour entrée
    LPC_GPIO1->FIODIR3 &= ~(0x07); 
    
    // Joystick : Haut (P1.23) et Centre (P1.20)
    // FIODIR2 : Bits 7 (0x80) et 4 (0x10) mis à 0 pour entrée
    LPC_GPIO1->FIODIR2 &= ~((1 << 7) | (1 << 4)); 
    
    // Bouton BP (INT0) : P2.10
    // FIODIR1 : Bit 2 mis à 0 pour entrée
    LPC_GPIO2->FIODIR1 &= ~(1 << 2); 

	
    // --- CONFIGURATION DES SORTIES (LEDs) ---
    
    // LEDs de statut : P1.28, P1.29, P1.31
    // FIODIR3 : Masque 0xB0 (1011 0000) mis à 1 pour sortie
    LPC_GPIO1->FIODIR3 |= 0xB0; 
    
    // LEDs de direction : P2.2 à P2.6
    // FIODIR0 : Masque 0x7C (0111 1100) mis à 1 pour sortie
    LPC_GPIO2->FIODIR0 |= 0x7C; 
}

void avancer (int vitesse_moyenne){
// --- CAS 1 : AVANCER (P1.24 appuyé) ---
		// Sens Avant : INA=1, INB=0
		LPC_GPIO0->FIOPIN2 = 0x01;
	
		// Vitesse
		LPC_PWM1->MR2 = vitesse_moyenne;
		LPC_PWM1->LER |= (1 << 2); // Valider Vitesse
	
}

void reculer(int vitesse_moyenne){
	// --- CAS 2 : RECULER (P1.26 appuyé) ---
		// Sens Arrière : INA=0, INB=1
		LPC_GPIO0->FIOPIN2 = 0x04;	
		// Vitesse
		LPC_PWM1->MR2 = vitesse_moyenne;
		LPC_PWM1->LER |= (1 << 2); // Valider Vitess
	
}
void arret(){
	// --- CAS 3 : RIEN (Arrêt) ---
		// Couper la puissance (MR2 = 0)
		LPC_PWM1->MR2 = 0;
		LPC_PWM1->LER |= (1 << 2); // Valider l'arrêt		
		
		LPC_GPIO0->FIOPIN2 = 0x05; // Par sécurité Brake : INA=1 et INB=1

}

void gauche(){
  // CAS 1 : GAUCHE (P1.23 / Bit 1 à 0)
	LPC_PWM1->MR3 = 25000; // Position Gauche (1ms)
	LPC_PWM1->LER |= (1 << 3); // Valider MR3
}

void droite(){
	 // CAS 2 : DROITE (P1.25 / Bit 3 à 0)
	LPC_PWM1->MR3 = 50000; // Position Droite (2ms)
	LPC_PWM1->LER |= (1 << 3); // Valider MR3
}

void centre (){
	// CAS 3 : TOUT DROIT (Aucun bouton directionnel)
	LPC_PWM1->MR3 = 37500; // Position Centre (1.5ms)
	LPC_PWM1->LER |= (1 << 3); // Valider MR3
}

char lecture_joystick(void) {
    // Lecture des boutons (0 = appuyé, différent de 0 = relâché)
    char droit  = LPC_GPIO1->FIOPIN3 & 0x01;
    char gauche = LPC_GPIO1->FIOPIN3 & 0x04;
    char haut   = LPC_GPIO1->FIOPIN2 & 0x80;
    char bas    = LPC_GPIO1->FIOPIN3 & 0x02;
    int  int0   = LPC_GPIO2->FIOPIN1 & 0x04;

    // --- PRIORITÉ 1 : LES COMBINAISONS (Reculer en tournant) ---
    // On doit tester INT0 en premier pour ne pas confondre avec la marche avant
    if (int0 == 0) {
        if (droit == 0)  return 5; // INT0 + Droite appuyés -> Case 5
        if (gauche == 0) return 6; // INT0 + Gauche appuyés -> Case 6
    }

    // --- PRIORITÉ 2 : TOURNER EN AVANÇANT ---
    if (droit == 0)  return 2; // Seulement Droite -> Case 2
    if (gauche == 0) return 3; // Seulement Gauche -> Case 3

    // --- PRIORITÉ 3 : LES LIGNES DROITES ---
    if (haut == 0) return 1; // Seulement Haut -> Case 1
    if (bas == 0)  return 4; // Seulement Bas -> Case 4 (Même si INT0 est appuyé, ça recule droit)

    // --- PAR DÉFAUT ---
    return 0; // Aucun bouton appuyé -> Case 0 (Arrêt)
}

void eteindre_toutes_les_leds(void) {
    LPC_GPIO1->FIOPIN3 &= ~(0xB0); // Éteint P1.28, P1.29, P1.31
    LPC_GPIO2->FIOPIN0 &= ~(0x7C); // Éteint P2.2, P2.3, P2.4, P2.5, P2.6
}
