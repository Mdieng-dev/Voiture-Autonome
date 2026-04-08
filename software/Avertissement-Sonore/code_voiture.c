#include "cmsis_os.h"
#include "Driver_USART.h"
#include "stm32f4xx.h"

osThreadId ID_TacheBouton;   // identifiant tâche bouton
osThreadId ID_TacheLED;      // identifiant tâche led
osThreadId ID_TacheUART;     // identifiant tâche uart

osThreadDef(tache_bouton, osPriorityHigh,   1, 0);   // priorité haute, 1 tâche, pile par défaut
osThreadDef(tache_led,    osPriorityNormal, 1, 0);   // priorité normale
osThreadDef(tache_uart,   osPriorityNormal, 1, 0);   // priorité normale

extern ARM_DRIVER_USART Driver_USART2;   // driver uart2

/* 0001.mp3 = klaxon
   0002.mp3 = demarrage voiture
   0003.mp3 = clignotant
   0004.mp3 = bip digicode
*/

#define TRACK_KLAXON       1
#define TRACK_MOTEUR       2
#define TRACK_CLIGNOTANT   3
#define TRACK_BIP          4

// variable globale pour choisir le son
int son_courant = 0;

void uart_init(void)
{
    Driver_USART2.Initialize(NULL);          // initialisation uart
    Driver_USART2.PowerControl(ARM_POWER_FULL);   // alimentation périphérique
    Driver_USART2.Control(
        ARM_USART_MODE_ASYNCHRONOUS |        // mode asynchrone
        ARM_USART_DATA_BITS_8       |        // 8 bits de données
        ARM_USART_STOP_BITS_1       |        // 1 bit stop
        ARM_USART_PARITY_NONE       |        // pas de parité
        ARM_USART_FLOW_CONTROL_NONE,         // pas de contrôle de flux
        9600                                 // débit 9600 bauds
    );
    Driver_USART2.Control(ARM_USART_CONTROL_TX, 1);   
}

// fonction pour envoyer une trame au dfplayer
void df_send(uint8_t cmd, uint16_t param)
{
    uint8_t t[10] = { 				//creation d un tableau de 10 octets avec les trame 
        0x7E, 0xFF, 0x06, cmd, 0x00,         // début trame + commande
        param >> 8, param & 0xFF,            // paramètre sur 2 octets
        0, 0, 0xEF                           
    };

    uint16_t sum = 0;                        // creation  variable somme pour checksum
    for (int i = 1; i < 7; i++) sum += t[i]; // somme des octets utiles
    sum = 0 - sum;                           // complément à 0

    t[7] = sum >> 8;                        
    t[8] = sum & 0xFF;                       

    while (Driver_USART2.GetStatus().tx_busy == 1);   // attente buffer libre
    Driver_USART2.Send(t, 10);                        // envoi de la trame
}

// fonction pour jouer le son en fonction du compteur
void jouer_son(void)
{
    if (son_courant == 0)
        df_send(0x08, TRACK_KLAXON);        // klaxon

    else if (son_courant == 1)
        df_send(0x08, TRACK_CLIGNOTANT);    // clignotant

    else if (son_courant == 2)
        df_send(0x08, TRACK_MOTEUR);        // moteur

    else
        df_send(0x08, TRACK_BIP);           // bip
}

// fonction pour passer au son suivant
void son_suivant(void)
{
    son_courant++;

    if (son_courant > 3)
        son_courant = 0;
}

// interruption bouton
void EXTI0_IRQHandler(void)
{
    osSignalSet(ID_TacheBouton, 0x0001); // réveille tâche bouton
    EXTI->PR = (1 << 0);                 // remise à zéro du drapeau
}

// tâche bouton
void tache_bouton(void const *argument)
{
    while (1)
    {
        osSignalWait(0x0001, osWaitForever); 

        if (GPIOA->IDR & (1 << 0))           // si bouton appuyé
        {
            osSignalSet(ID_TacheLED,  0x0001);   // led allume 
            osSignalSet(ID_TacheUART, 0x0001);   // jouer son 
        }
        else                                  // si bouton relâché
        {
            osSignalSet(ID_TacheLED,  0x0002);   // demande led eteinte 
            osSignalSet(ID_TacheUART, 0x0002);   // son stop
        }
    }
}

// tâche led
void tache_led(void const *argument)
{
    while (1)
    {
        osEvent result = osSignalWait(0, osWaitForever);   // attente d'un signal
        int flags = result.value.signals;                  // récupération du signal

        if (flags & 0x0001)
            GPIOD->BSRR = (1 << 12);              // allume LED verte PD12

        if (flags & 0x0002)
            GPIOD->BSRR = (1 << (12 + 16));       // éteint LED verte PD12
    }
}

// tâche uart
void tache_uart(void const *argument)
{
    while (1)
    {
        osEvent result = osSignalWait(0, osWaitForever);   // attente d'un signal
        int flags = result.value.signals;                  // récupération du signal

        if (flags & 0x0001)
        {
            jouer_son();          // jouer son courant
        }

        if (flags & 0x0002)
        {
            df_send(0x16, 0);     // stop
            son_suivant();        // passer au son suivant
        }
    }
}

int main(void)
{
    RCC->AHB1ENR |= (1 << 3);               // activation horloge port D
    GPIOD->MODER &= ~(3 << (12 * 2));       // remise à zéro mode PD12
    GPIOD->MODER |=  (1 << (12 * 2));       // PD12 en sortie

    RCC->AHB1ENR |= (1 << 0);               // activation horloge port A
    GPIOA->MODER &= ~(3 << 0);              // PA0 en entrée
    GPIOA->PUPDR &= ~(3 << 0);              // remise à zéro pull-up/pull-down
    GPIOA->PUPDR |=  (2 << 0);              // pull-down

    RCC->APB2ENR |= (1 << 14);           
    EXTI->IMR  |= (1 << 0);                 
    EXTI->RTSR |= (1 << 0);                 
    EXTI->FTSR |= (1 << 0);                 
    NVIC_EnableIRQ(EXTI0_IRQn);             

    uart_init();                            // initialisation uart dfplayer

    osKernelInitialize();                   // initialisation noyau RTOS

    ID_TacheBouton = osThreadCreate(osThread(tache_bouton), NULL);   // création tâche bouton
    ID_TacheLED    = osThreadCreate(osThread(tache_led), NULL);      // création tâche led
    ID_TacheUART   = osThreadCreate(osThread(tache_uart), NULL);     // création tâche uart

    osKernelStart();                        // démarrage RTOS

    df_send(0x06, 20);                      // volume

    osDelay(osWaitForever);                 // main en attente infinie
    return 0;
}
