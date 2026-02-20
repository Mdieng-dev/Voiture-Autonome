//---------------------------------------------------------------------------------------------
// Librairie des fonctions permettant l'utilisation de la carte d'extension avec MCB1768 (KEIL)
// IUT de Cachan
// version : 05/09/2022 
//---------------------------------------------------------------------------------------------

#include "LPC17xx.h"
#include "GPIO.h"

// Initialisation des différentes entrées/sorties GPIO
// ---------------------------------------------------
void Initialise_GPIO (void)
{
	// Sortie LEDs
	LPC_GPIO2->FIODIR0 = LPC_GPIO2->FIODIR0 | 0x7C;	// FIO2DIR0 pour 5 LEDs LED0 = P2.6 ... LED4 = P2.2  en sortie
	LPC_GPIO1->FIODIR3 = LPC_GPIO1->FIODIR3 | 0xB0;	// FIO2DIR3 pour 3 LEDs LED5 = P1.31, LED6 = P1.29 et LED7 = P1.28 en sortie
	// Entree BP carte principale
	LPC_GPIO2->FIODIR1 = LPC_GPIO2->FIODIR1 & 0xFB;	// FIO2DIR1 pour BP = P2.10 en entree
	// Entrees BP carte d'extension
	LPC_GPIO0->FIODIR2 = LPC_GPIO0->FIODIR2 & 0x00;	// FIO0DIR2 pour BP0 = P0.16 ... BP0 = en entree
	// Entrees Joystick Carte Keil
	LPC_GPIO1->FIODIR3 = LPC_GPIO1->FIODIR3 & 0xF8;	// FIO1DIR3 pour JS en entree
	LPC_GPIO1->FIODIR2 = LPC_GPIO1->FIODIR2 & 0x6F;	// FIO1DIR2 pour JS en entree
	LPC_PINCON->PINMODE1 = LPC_PINCON->PINMODE1|0x0000aaaa; //désactive les résistances internes du µC pour P0.16, 17, 187, 19, 20, 21, 22,23

}

// Lecture BP carte principale
// Argument sortie : 1 si P2.10 enfoncé, 0 sinon
// ---------------------------------------------
char Valeur_BP(void)		
{
	char inter, sortie;
	inter = LPC_GPIO2->FIOPIN1 & 0x04;
	if (inter == 0x00)		// BP enfonce
		sortie = 1;
	else
		sortie = 0;
	return sortie;
}

// Lecture Joystick droit carte principale
// Argument sortie : 1 si Joystick droit enfoncé, 0 sinon
// ------------------------------------------------------
char Valeur_Joystick_droit(void)
{
	char inter, sortie;
	inter = LPC_GPIO1->FIOPIN3 & 0x01;
	if (inter == 0x00)	// BP enfonce
		sortie = 1;
	else
		sortie = 0;
	return sortie;
}

// Lecture Joystick gauche carte principale
// Argument sortie : 1 si Joystick gauche enfoncé, 0 sinon
// -------------------------------------------------------
char Valeur_Joystick_gauche(void)
{
	char inter, sortie;
	inter = LPC_GPIO1->FIOPIN3 & 0x04;
	if (inter == 0x00)	// BP enfonce
		sortie = 1;
	else
		sortie = 0;
	return sortie;
}
// Lecture Joystick haut carte principale
// Argument sortie : 1 si Joystick haut enfoncé, 0 sinon
// -----------------------------------------------------
char Valeur_Joystick_haut(void)
{
	char inter, sortie;
	inter = LPC_GPIO1->FIOPIN2 & 0x80;
	if (inter == 0x00)	// BP enfonce
		sortie = 1;
	else
		sortie = 0;
	return sortie;
}

// Lecture Joystick bas carte principale
// Argument sortie : 1 si Joystick bas enfoncé, 0 sinon
// ----------------------------------------------------
char Valeur_Joystick_bas(void)
{
	char inter, sortie;
	inter = LPC_GPIO1->FIOPIN3 & 0x02;
	if (inter == 0x00)	// BP enfonce
		sortie = 1;
	else
		sortie = 0;
	return sortie;
}

// Lecture Joystick centre carte principale
// Argument sortie : 1 si Joystick centre enfoncé, 0 sinon
// -------------------------------------------------------
char Valeur_Joystick_centre(void)
{
	char inter, sortie;
	inter = LPC_GPIO1->FIOPIN2 & 0x10;
	if (inter == 0x00)	// BP enfonce
		sortie = 1;
	else
		sortie = 0;
	return sortie;
}	

// Allumage d'une LED
// Argument entrée : le numéro de la LED à allumer, entre 0 et 7
//--------------------------------------------------------------
void Allumer_1LED(char Num_LED)
{
	switch (Num_LED)
		{
			case 0: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x40;
							break;
			case 1: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x20;
							break;
			case 2: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x10;
							break;
			case 3: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x08;
							break;
			case 4: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 | 0x04;
							break;
			case 5: LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 | 0x80;
							break;
			case 6: LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 | 0x20;
							break;
			case 7: LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 | 0x10;
							break;							
		}
}

// Extinction d'une LED
// Argument entrée : le numéro de la LED à éteindre, entre 0 et 7
//---------------------------------------------------------------
void Eteindre_1LED(char Num_LED)	
{
	switch (Num_LED)
		{
			case 0: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 & 0xBF;
							break;
			case 1: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 & 0xDF;
							break;
			case 2: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 & 0xEF;
							break;
			case 3: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 & 0xF7;
							break;
			case 4: LPC_GPIO2->FIOPIN0 = LPC_GPIO2->FIOPIN0 & 0xFB;
							break;
			case 5: LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 & 0x7F;
							break;
			case 6: LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 & 0xDF;
							break;
			case 7: LPC_GPIO1->FIOPIN3 = LPC_GPIO1->FIOPIN3 & 0xEF;
							break;							
		}
}

// Ecriture d'une valeur sur les sorties GPIO (LEDs)
// Argument entrée : l'octet à envoyer sur la sortie
// -------------------------------------------------
void Ecriture_GPIO(unsigned char Valeur)
{
	unsigned char Sortie = 0;
	LPC_GPIO2->FIOPIN0 = Sortie | ((Valeur&0x01)<<6) | ((Valeur&0x02)<<4) | ((Valeur&0x04)<<2) | ((Valeur&0x08)) | ((Valeur&0x10)>>2);
	Sortie = 0;
	LPC_GPIO1->FIOPIN3 = Sortie | ((Valeur&0x20)<<2) | ((Valeur&0x40)>>1) | ((Valeur&0x80)>>3);
	
}

// Ecriture d'une valeur sur les sorties GPIO (LEDs) - version 2 (non optimisée)
// Argument entrée : l'octet à envoyer sur les LED
// -----------------------------------------------
void Ecriture_GPIO2(unsigned char Valeur)
{
	char i;
	unsigned char test;
	for ( i =0 ; i<8 ; i++)		
	{
		test = (Valeur>>i)&0x01;	// on scanne les bits de l'octet 1 a 1
		if (test == 1)
			Allumer_1LED(i);
		else
			Eteindre_1LED(i);
	}
	
}

// Lecture d'une valeur sur les entrées GPIO (BP carte extension)
// Argument sortie : l'octet présent au niveau des BP
// --------------------------------------------------
char Lecture_GPIO()
{
	return LPC_GPIO0->FIOPIN2;
}


// Initialisation des E/S TOR pour le train 1
// ------------------------------------------
void initTrain1()
{
	LPC_GPIO2->FIODIR0 |= 0x60;
}

// Lecture d'une valeur sur les entrées GPIO (train1)
// Argument sortie : l'octet présent au niveau des BP
// --------------------------------------------------
char capteursTrain1()
{
	return LPC_GPIO0->FIOPIN2&0x0F;
}

// Arrêt du train1
//----------------
void arreterTrain1()
{
	LPC_GPIO2->FIOPIN0 = 0x00;
}

// Lancement du train1 en marche avant 
void avancerTrain1()
{
	LPC_GPIO2->FIOPIN0 = 0x20;
}

// Lancement du train1 en marche arrière 
void reculerTrain1()
{
	LPC_GPIO2->FIOPIN0 = 0x60;
}

// Initialisation des E/S TOR pour le train 2
// ------------------------------------------
void initTrain2()
{
	LPC_GPIO1->FIODIR3 |= 0x30;
}

// Lecture d'une valeur sur les entrées GPIO (train2)
// Argument sortie : l'octet présent au niveau des BP
// --------------------------------------------------
char capteursTrain2()
{
	return LPC_GPIO0->FIOPIN2&0xF0;
}

// Arrêt du train2
//----------------
void arreterTrain2()
{
	LPC_GPIO1->FIOPIN3 = 0x00;
}

// Lancement du train2 en marche avant 
void avancerTrain2()
{
	LPC_GPIO1->FIOPIN3 = 0x10;
}

// Lancement du train2 en marche arrière 
void reculerTrain2()
{
	LPC_GPIO1->FIOPIN3 = 0x30;
}




// Initialisation des E/S TOR pour le train 3
// ------------------------------------------
void initTrain3()
{
	LPC_GPIO2->FIODIR0 |= 0x18;
}

// Lecture d'une valeur sur les entrées GPIO1 (train3)
// Argument sortie : l'octet 0 0 0 0 M3 A3 B3 C3
// --------------------------------------------------
char capteursTrain3()
{
	char valCapteurs;
	valCapteurs=((LPC_GPIO1->FIOPIN3&0x07)<<1) | ((LPC_GPIO1->FIOPIN2&0x80)>>7);
	return valCapteurs;
}

// Arrêt du train3
//----------------
void arreterTrain3()
{
	LPC_GPIO2->FIOPIN0 = 0x00;
}

// Lancement du train3 en marche avant 
void avancerTrain3()
{
	LPC_GPIO2->FIOPIN0 = 0x08;
}

// Lancement du train3 en marche arrière 
void reculerTrain3()
{
	LPC_GPIO2->FIOPIN0 = 0x18;
}
