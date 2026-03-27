#include "LPC17xx.h"
#include "cmsis_os2.h"
#include "Driver_I2C.h"
#include "Board_GLCD.h"
#include "GLCD_Config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


extern ARM_DRIVER_I2C Driver_I2C2;
extern GLCD_FONT GLCD_Font_16x24;

// Adresse de la manette Nunchuck
#define ADDR 0x52

// Modèle du paquet d'informations envoyé entre les tâches
typedef struct {
    uint8_t x; 
    uint8_t y;
    bool z; 
    bool c;
    int err;
} Msg_t;

// Identifiant de la boîte aux lettres pour le multitâche
osMessageQueueId_t q_id;

// Tâche 1 : Lecture du Nunchuck
void tache1(void *arg) {
    uint8_t cmd = 0x00;
    uint8_t data[6];
    Msg_t m;
    
    // Séquence spéciale pour réveiller la manette Wii
    uint8_t t1[2] = {0xF0, 0x55};
    uint8_t t2[2] = {0xFB, 0x00};

    // Configuration de l'I2C
    Driver_I2C2.Initialize(NULL);
    Driver_I2C2.PowerControl(ARM_POWER_FULL);
    Driver_I2C2.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);

    // Envoi du signal de réveil
    Driver_I2C2.MasterTransmit(ADDR, t1, 2, false);
    osDelay(20);
    Driver_I2C2.MasterTransmit(ADDR, t2, 2, false);
    osDelay(20);

    while(1) {
        m.err = 0;
        
        // Demande les données à la manette
        Driver_I2C2.MasterTransmit(ADDR, &cmd, 1, false);
        uint32_t tout = 500;
        
        // Rend la main au système d'exploitation si l'I2C est occupé
        while(Driver_I2C2.GetStatus().busy && tout--) osThreadYield();
        if(tout==0) m.err = 1;

        osDelay(5);
        
        // Reçoit les 6 octets de la manette
        Driver_I2C2.MasterReceive(ADDR, data, 6, false);
        tout = 500;
        while(Driver_I2C2.GetStatus().busy && tout--) osThreadYield();
        if(tout==0) m.err = 1;

        // Décrypte (nintendo)
        m.x = data[0];
        m.y = data[1];
        m.z = !(data[5] & 0x01);
        m.c = !(data[5] & 0x02);

        // Poste le paquet dans la boîte aux lettres pour l'autre tâche
        osMessageQueuePut(q_id, &m, 0, 0);
        
        // tempo
        osDelay(50);
    }
}

// Tâche 2 : Gestion de l'écran LCD
void tache2(void *arg) {
    Msg_t r;
    // Curseur positionné au milieu de l'écran au démarrage
    uint32_t px = 160, py = 120;

    
    bool last_z = false, last_c = false;
    bool force_draw = true; 
    char str[32];

    // Prépare le premier affichage
    GLCD_ClearScreen(); 
    GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
    GLCD_DrawString(px, py, "O");

    while(1) {
        // sommeil jusqu'a message
        osMessageQueueGet(q_id, &r, NULL, osWaitForever);

        // Affiche une erreur si la manette est débranchée
        GLCD_SetForegroundColor(GLCD_COLOR_BLACK);
        if(r.err) {
            sprintf(str, "il y a un probleme en I2C");
            GLCD_DrawString(0, 0, str);
        }

        if (!r.err) {
            // Si le joystick bouge
            if (r.x < 80 || r.x > 170 || r.y < 80 || r.y > 170) {
                
                // Efface l'ancien "O" en le redessinant couleur fond 
                GLCD_SetForegroundColor(GLCD_COLOR_WHITE);
                GLCD_DrawString(px, py, "O");

                // Met à jour les coordonnées selon la direction
                if(r.x < 80) px -= 10; 
                if(r.x > 170) px += 10;
                if(r.y < 80) py += 10; 
                if(r.y > 170) py -= 10;

                // limite de l'ecran
                if(px<5) px=5; 
                if(px>300) px=300;
                if(py<30) py=30; 
                if(py>160) py=160;

                // Dessine le nouveau "O" en Bleu
                GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
                GLCD_DrawString(px, py, "O");
            }

            // Affiche ou efface l'alerte du Bouton Z
            if (r.z != last_z || force_draw) {
                if (r.z) {
                    GLCD_SetForegroundColor(GLCD_COLOR_RED);
                    GLCD_DrawString(10, 180, "Z APPUYE "); 
                } else {
                    GLCD_DrawString(10, 180, "         "); // Remplace par du vide
                }
                last_z = r.z;
            }

            // Affiche ou efface l'alerte du Bouton C
            if (r.c != last_c || force_draw) {
                if (r.c) {
                    GLCD_SetForegroundColor(GLCD_COLOR_RED);
                    GLCD_DrawString(10, 210, "C APPUYE "); 
                } else {
                    GLCD_DrawString(10, 210, "         "); 
                }
                last_c = r.c;
            }

            force_draw = false; // L'affichage initial est terminé
        }
    }
}

// main
int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    // Allume et configure l'écran
    GLCD_Initialize();
    GLCD_ClearScreen();
    GLCD_SetFont(&GLCD_Font_16x24);
    GLCD_SetForegroundColor(GLCD_COLOR_RED);
    GLCD_DrawString(10, 10, "LCD fonctionne");

    // Allume le noyau du système d'exploitation RTOS
    osKernelInitialize();
    GLCD_DrawString(10, 40, "Initialisation fonctionne");

    // Crée la boîte aux lettres pour communiquer entre les tâches
    q_id = osMessageQueueNew(5, sizeof(Msg_t), NULL);

    // Déclare les deux tâches parallèles à l'OS
    osThreadNew(tache1, NULL, NULL);
    osThreadNew(tache2, NULL, NULL);

    // Démarre la rotation des tâches 
    GLCD_DrawString(10, 70, "os va commencer");
    osKernelStart();

    while(1);
}