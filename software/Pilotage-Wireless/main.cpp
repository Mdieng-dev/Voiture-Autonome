#include "LPC17xx.h"
#include "cmsis_os2.h"
#include "Driver_I2C.h"
#include "Board_GLCD.h" 
#include "GLCD_Config.h"
#include "RTE_Components.h"


extern ARM_DRIVER_I2C Driver_I2C2;
extern GLCD_FONT GLCD_Font_16x24; 

// structure
#define SLAVE_I2C_ADDR 0x52
#define SEUIL_HAUT     170
#define SEUIL_BAS      80
#define VITESSE        10

// envoyé de la tâche Nunchuck vers la tâche LCD
typedef struct {
    int delta_x;   
    int delta_y;   
    bool bouton_1; 
    bool bouton_2; 
} Mail_Nunchuck_t;

osThreadId_t       id_tache1; 
osThreadId_t       id_tache2; 
osMessageQueueId_t mail_id;   




void writelbyte(unsigned char adress, unsigned char registre, unsigned char valeur) {
    unsigned char t[2];
    t[0] = registre;
    t[1] = valeur;
    Driver_I2C2.MasterTransmit(adress, t, 2, false);
    while(Driver_I2C2.GetStatus().busy == 1);
}

void Init_I2C(void) {
    Driver_I2C2.Initialize(NULL);
    Driver_I2C2.PowerControl(ARM_POWER_FULL);
    Driver_I2C2.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD); 
    Driver_I2C2.Control(ARM_I2C_BUS_CLEAR, 0);
    
    // Séquence Nunchuck
    writelbyte(SLAVE_I2C_ADDR, 0xF0, 0x55);
    osDelay(10);
    writelbyte(SLAVE_I2C_ADDR, 0xFB, 0x00);
    osDelay(10);
}


// lecture
void tache1(void *argument) {
    uint8_t reg = 0x00;
    uint8_t data[6];
    Mail_Nunchuck_t msg;
    
    Init_I2C(); 

    while(1) {
        // Demande de lecture
        Driver_I2C2.MasterTransmit(SLAVE_I2C_ADDR, &reg, 1, false);
        while(Driver_I2C2.GetStatus().busy == 1);
        osDelay(2); 
        
        // Récupération des 6 octets
        Driver_I2C2.MasterReceive(SLAVE_I2C_ADDR, data, 6, false);
        while(Driver_I2C2.GetStatus().busy == 1);
        
        // Calcul du mouvement 
        msg.delta_x = 0;
        msg.delta_y = 0;
        
        if (data[0] < SEUIL_BAS)  msg.delta_x = -VITESSE; 
        if (data[0] > SEUIL_HAUT) msg.delta_x =  VITESSE; 
        if (data[1] < SEUIL_BAS)  msg.delta_y =  VITESSE; // Bas
        if (data[1] > SEUIL_HAUT) msg.delta_y = -VITESSE; // Haut
        
        // Etat des boutons (Logique inverse)
        msg.bouton_1 = !(data[5] & 0x01); // Z
        msg.bouton_2 = !(data[5] & 0x02); // C
        
        // Envoi du Mail à la tâche LCD
        osMessageQueuePut(mail_id, &msg, 0, 0); 
        
        osDelay(50); // Attente RTOS propre (remplace le timer, lecture à 20Hz)
    }
}



// 5. TACHE 2 : AFFICHAGE LCD (Elle reçoit le mail et dessine)

void tache2(void *argument) {
    Mail_Nunchuck_t msg_recu;
    
    // Coordonnées de départ au milieu de l'écran
    uint32_t x = 160; 
    uint32_t y = 120;
    
    // --- API OFFICIELLE KEIL (D'après votre capture) ---
    GLCD_Initialize();
    GLCD_SetBackgroundColor(GLCD_COLOR_WHITE);
    GLCD_ClearScreen();
    GLCD_SetFont(&GLCD_Font_16x24);

    while(1) {
        // La tâche s'endort ici jusqu'à la réception d'un mail du Nunchuck
        osMessageQueueGet(mail_id, &msg_recu, NULL, osWaitForever);
        
        // 1. EFFACER L'ANCIENNE POSITION
        // On remet la couleur du fond (Blanc) et on redessine par-dessus pour gommer
        GLCD_SetForegroundColor(GLCD_COLOR_WHITE);
        // *Astuce: l'API Keil de base ne gérant pas les cercles, on affiche la lettre 'O' géante qui fait office de cercle !
        GLCD_DrawString(x, y, "O"); 
        
        // 2. METTRE À JOUR LES COORDONNÉES
        x += msg_recu.delta_x;
        y += msg_recu.delta_y;
        
        // Bloquer le cercle pour ne pas sortir de l'écran (320x240)
        if(x < 5) x = 5;
        if(x > 300) x = 300;
        if(y < 5) y = 5;
        if(y > 210) y = 210;
        
        // 3. DESSINER LA NOUVELLE POSITION
        GLCD_SetForegroundColor(GLCD_COLOR_BLUE); // Couleur du cercle
        GLCD_DrawString(x, y, "O"); // Affiche le "cercle" à la nouvelle position
        
        // 4. AFFICHER LE TEXTE DES BOUTONS
        GLCD_SetForegroundColor(GLCD_COLOR_RED); // Texte en rouge
        
        if (msg_recu.bouton_1) {
            GLCD_DrawString(10, 50, "Bouton 1 appuye!   "); // X=10, Y=50
        } 
        else if (msg_recu.bouton_2) {
            GLCD_DrawString(10, 50, "Bouton 2 appuye!   ");
        } 
        else {
            // Si on lâche, on efface la ligne (en écrivant des espaces)
            GLCD_DrawString(10, 50, "                   "); 
        }
    }
}


// main
int main(void) {
    SystemInit();
    osKernelInitialize();
    
    // Création de la boîte aux lettres (5 messages max)
    mail_id = osMessageQueueNew(5, sizeof(Mail_Nunchuck_t), NULL);
    
    // Création des 2 tâches
    id_tache1 = osThreadNew(tache1, NULL, NULL);
    id_tache2 = osThreadNew(tache2, NULL, NULL);
    
    if (osKernelGetState() == osKernelReady) {
        osKernelStart();
    }
    
    while(1);
}