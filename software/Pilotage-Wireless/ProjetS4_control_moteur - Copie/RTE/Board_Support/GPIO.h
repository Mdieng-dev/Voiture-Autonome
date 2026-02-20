//---------------------------------------------------------------------------------------------
// Librairie des fonctions permettant l'utilisation de la carte d'extension avec MCB1768 (KEIL)
// IUT de Cachan
// version : 05/09/2022 
//---------------------------------------------------------------------------------------------

#include "LPC17xx.h"

// Gestion GPIO
void Initialise_GPIO (void);
char Valeur_BP(void);
void Allumer_1LED(char Num_LED);
void Eteindre_1LED(char Num_LED);
void Ecriture_GPIO(unsigned char Valeur);
void Ecriture_GPIO2(unsigned char Valeur);
char Lecture_GPIO(void);

// Fonctions de gestion du Joystick (carte Keil)
char Valeur_Joystick_droit(void);
char Valeur_Joystick_gauche(void);
char Valeur_Joystick_haut(void);
char Valeur_Joystick_bas(void);
char Valeur_Joystick_centre(void);

// Fonctions de gestion des trains
void initTrain1(void);
char capteursTrain1(void);
void arreterTrain1(void);
void avancerTrain1(void);
void reculerTrain1(void);
void initTrain2(void);
char capteursTrain2(void);
void arreterTrain2(void); 
void avancerTrain2(void);
void reculerTrain2(void);
void initTrain3(void);
char capteursTrain3(void);
void arreterTrain3(void); 
void avancerTrain3(void);
void reculerTrain3(void);
