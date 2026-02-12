#include "LPC17xx.h"                    // Device header
#include "GPIO_LPC17xx.h"               // Device:GPIO

void Init_motor ()
{
    // Activation de la puissance pour le bloc PWM1
    LPC_SC->PCONP |= (1 << 6); 

    // Configuration des broches P2.0 et P2.1 en mode PWM1
    // P2.0 = PWM1.1 (Direction Servo)
    // P2.1 = PWM1.2 (Sens Moteur via VNH5019)
    LPC_PINCON->PINSEL4 &= ~(0x0F); // On nettoie les 4 premiers bits
    LPC_PINCON->PINSEL4 |= 0x05;    // 0101 en binaire -> P2.0 et P2.1 en PWM

    // Configuration des broches P2.2 et P2.3 en sorties (GPIO)
    // P2.2 = INA (Sens moteur)
    // P2.3 = INB (Sens moteur)
    LPC_GPIO2->FIODIR |= (1 << 2) | (1 << 3); 
    LPC_GPIO2->FIOCLR = (1 << 2) | (1 << 3); // Moteur à l'arrêt (0 et 0)

    // Configuration de la fréquence (Période de 20ms)
    LPC_PWM1->PR = 0;               // Pas de pré-diviseur
    LPC_PWM1->MR0 = 500000 - 1;     // 500 000 * 40ns = 20ms (50Hz)

    // Configuration du compteur et des sorties
    LPC_PWM1->MCR |= (1 << 1);      // Reset du compteur sur MR0
    LPC_PWM1->PCR |= (1 << 9) | (1 << 10); // Active les sorties PWM1.1 et PWM1.2
    
    // Valeurs initiales (Rapport cyclique)
    LPC_PWM1->MR1 = 37500;          // Direction : Milieu (1.5ms)
    LPC_PWM1->MR2 = 0;              // Vitesse : 0 (Arrêt)

    // Validation des changements (LATCH)
    // On valide MR0, MR1 (bits 0, 1)
    LPC_PWM1->LER = 0x07; 

    // Démarrage du PWM
    // Bit 0 = Counter Enable, Bit 3 = PWM Enable
    LPC_PWM1->TCR = 0x09;  
}

int main(void)		{


}