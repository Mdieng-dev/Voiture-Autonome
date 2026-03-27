#include "stm32f4xx.h"
#include "Driver_SPI.h" // Ajout de la bibliothèque standard CMSIS

// Valeurs pour le capteur de lumière
#define SEUIL_NUIT 1000
#define SEUIL_JOUR 2000

// Valeurs de puissance des LEDs
#define LUMINOSITE_NUIT 31
#define LUMINOSITE_JOUR 2

// Déclaration du driver SPI (comme sur la diapo)
extern ARM_DRIVER_SPI Driver_SPI1;

// Variable globale
uint8_t puissance = LUMINOSITE_JOUR;

// Configuration initiale
void Init_System(void) {
    // Activation des horloges pour le Port A et l'ADC1
    // L'horloge du SPI1 est gérée automatiquement par le driver CMSIS
    RCC->AHB1ENR |= (1 << 0);
    RCC->APB2ENR |= (1 << 8);

    // Bouton Bleu (PA0) en mode Entree
    GPIOA->MODER &= ~(3 << 0);

    // Capteur de lumiere (PA1) en mode Analogique
    GPIOA->MODER |= (3 << 2);

    // Initialisation du convertisseur analogique (ADC1)
    ADC1->CR2 |= (1 << 0);
    ADC1->SQR3 = 1;

    // Initialisation du SPI avec le driver CMSIS
    Driver_SPI1.Initialize(NULL);
    Driver_SPI1.PowerControl(ARM_POWER_FULL);
    
    // Configuration du SPI : Maître, Mode 0 (standard pour ces LEDs), 8 bits, 1 MHz
    Driver_SPI1.Control(ARM_SPI_MODE_MASTER | 
                        ARM_SPI_CPOL0_CPHA0 | 
                        ARM_SPI_MSB_LSB | 
                        ARM_SPI_SS_MASTER_UNUSED | 
                        ARM_SPI_DATA_BITS(8), 1000000);
                        
    Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

// Mise à jour des couleurs des 6 LEDs
void Update_LEDs(uint8_t mode, uint8_t etape) {
    // Tableau qui contiendra toute la commande à envoyer (32 octets au total)
    // 4 octets de début + (6 LEDs x 4 octets) + 4 octets de fin
    uint8_t trame[32]; 
    int index = 0;

    // Start Frame (4 octets à 0x00)
    for(int i=0; i<4; i++) {
        trame[index++] = 0x00;
    }

    // Calcul et remplissage des couleurs pour les 6 LEDs
    for(int i=0; i<6; i++) {
        // Couleurs par défaut (Blanc)
        uint8_t br = puissance, r = 255, g = 255, b = 255;

        // Si on n'est pas en mode phares normaux
        if (mode > 0) {
            int est_cligno = 0;
            if (mode == 1 && i < 3) est_cligno = 1;
            if (mode == 2 && i >= 3) est_cligno = 1;
            if (mode == 3) est_cligno = 1;

            if (est_cligno) {
                // Calcul de la position dans le clignotant (0, 1 ou 2)
                uint8_t pos = (i < 3) ? i : (i - 3);
                
                if (pos == etape) {
                    br = 31; r = 255; g = 120; b = 0; // Orange fort
                } else if (pos < etape) {
                    br = 5; r = 255; g = 60; b = 0;   // Traînée orange faible
                } else {
                    br = 0; r = 0; g = 0; b = 0;      // Eteint
                }
            }
        }

        // Ajout de la LED dans le tableau (Format : Luminosité, Bleu, Vert, Rouge)
        trame[index++] = 0xE0 | br;
        trame[index++] = b;
        trame[index++] = g;
        trame[index++] = r;
    }

    // End Frame (4 octets à 0xFF)
    for(int i=0; i<4; i++) {
        trame[index++] = 0xFF;
    }

    // Envoi de toute la commande d'un seul coup grâce au driver CMSIS
    Driver_SPI1.Send(trame, 32);
}

// Boucle Principale
int main(void) {
    Init_System();
    uint16_t valeur_luz = 0;
    int action_manette = 0;

    while(1) {
        // Lecture de la lumière
        ADC1->CR2 |= (1 << 30);
        while(!(ADC1->SR & (1 << 1)));
        valeur_luz = ADC1->DR;

        // Calcul de la puissance automatique
        if (valeur_luz <= SEUIL_NUIT) {
            puissance = LUMINOSITE_NUIT;
        } else if (valeur_luz >= SEUIL_JOUR) {
            puissance = LUMINOSITE_JOUR;
        } else {
            uint32_t progression = valeur_luz - SEUIL_NUIT;
            uint32_t plage_totale = SEUIL_JOUR - SEUIL_NUIT;
            uint32_t ecart_lum = LUMINOSITE_NUIT - LUMINOSITE_JOUR;
            puissance = LUMINOSITE_NUIT - ((progression * ecart_lum) / plage_totale);
        }

        // Lecture du bouton avec anti-rebond
        if (GPIOA->IDR & (1 << 0)) {
            action_manette++;
            if (action_manette > 3) {
                action_manette = 0;
            }
            for(volatile int i=0; i<500000; i++);
            while(GPIOA->IDR & (1 << 0));
        }

        // Exécution de l'affichage
        if (action_manette > 0) {
            // Animation clignotant
            for (uint8_t e = 0; e < 5; e++) {
                Update_LEDs(action_manette, e);
                for(volatile int d=0; d<40000; d++); // Vitesse animation
            }
        } else {
            // Phares normaux
            Update_LEDs(0, 0);
            for(volatile int d=0; d<10000; d++);
        }
    }
}