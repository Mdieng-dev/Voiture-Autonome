#include "stm32f4xx.h"

void Configuration(void) {
    // horloges
    RCC->AHB1ENR |= (1 << 0) | (1 << 3);
    RCC->APB2ENR |= (1 << 8);

    // leds PD12 et PD14 sortie
    GPIOD->MODER |= (1 << 24) | (1 << 28);

    // bouton PA0 entree
    GPIOA->MODER &= ~(3 << 0);
    
    // capteur PA1 analogique
    GPIOA->MODER |= (3 << 2);

    // adc active
    ADC1->CR2 |= (1 << 0);
    ADC1->SQR3 = 1;
}

int main(void) {
    // 0 = rien
    // 1 = gauche
    // 2 = droite
    // 3 = warning
    int action_manette = 0;

    int lumiere = 0;
    int mode_nuit = 0;
    int compteur = 0;
    int etat_cligno = 0;

    Configuration();

    while(1) {
        
        // capteur lumiere
        ADC1->CR2 |= (1 << 30);
        while ((ADC1->SR & 2) == 0);
        lumiere = ADC1->DR;

        // decision nuit
        if (lumiere < 2000) {
            mode_nuit = 1;
        }
        else if (lumiere > 3000) {
            mode_nuit = 0;
        }

        // simulation manette avec le bouton
        // appuie pour changer : rien -> gauche -> droite -> warning -> rien
        if (GPIOA->IDR & 1) {
            action_manette++;
            if (action_manette > 3) {
                action_manette = 0;
            }
            // attente anti rebond
            for(int i=0; i<500000; i++);
            while(GPIOA->IDR & 1);
        }

        // vitesse clignotement
        compteur++;
        if (compteur > 100000) {
            compteur = 0;
            etat_cligno = !etat_cligno;
        }

        // GESTION LED GAUCHE (PD12)
        // priorite au clignotant ou warning
        if (action_manette == 1 || action_manette == 3) {
            if (etat_cligno == 1) GPIOD->ODR |= (1 << 12);
            else GPIOD->ODR &= ~(1 << 12);
        }
        else {
            // sinon eclairage auto
            if (mode_nuit == 1) GPIOD->ODR |= (1 << 12);
            else GPIOD->ODR &= ~(1 << 12);
        }

        // GESTION LED DROITE (PD14)
        // priorite au clignotant ou warning
        if (action_manette == 2 || action_manette == 3) {
            if (etat_cligno == 1) GPIOD->ODR |= (1 << 14);
            else GPIOD->ODR &= ~(1 << 14);
        }
        else {
            // sinon eclairage auto
            if (mode_nuit == 1) GPIOD->ODR |= (1 << 14);
            else GPIOD->ODR &= ~(1 << 14);
        }
    }
}