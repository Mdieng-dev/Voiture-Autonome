#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Driver_USART.h"
#include <stdio.h>   
#include <string.h>  // Pour strncmp


extern ARM_DRIVER_USART Driver_USART3;
osThreadId_t tid_GPS;           

char trame_gps[100],idx=0;                      
int octet_recu;             


float latitude_recup, longitude_recup;
int fix_quality;
float latitude, longitude;

//FONCTION CALLBACK 
void myUART_callback(uint32_t event) {
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
		
        
		if ((octet_recu == '$' && (idx < 99))|| idx > 0) {
                trame_gps[idx++] = (char)octet_recu;}
        else;
            // Si on reçoit la fin de ligne
        if (octet_recu == '\n') {
                osThreadFlagsSet(tid_GPS, 0x01); // On réveille la tâche de traitement
								idx = 0;}
        else;
        }
    
		Driver_USART3.Receive(&octet_recu, 1);          // On relance la réception de l'octet suivant
		}

//traitement de la trame 
void Thread_Traitement_GPS(void *argument) {
		int deg_lat,deg_lon;
		float min_lat,min_lon;
    while (1) {
        // La tâche s'endort ici et attend le flag 0x01 envoyé par la Callback
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        // analyse de la trame
        if (strncmp(trame_gps, "$GPGGA", 6) == 0) {
            sscanf(trame_gps, "$GPGGA,%*f,%f,%*c,%f,%*c,%d", &latitude_recup, &longitude_recup, &fix_quality); }
		else;
		if (fix_quality > 0) {
						// Latitude
						deg_lat = (int)(latitude_recup / 100);             // Récupère la partie entiere 48,75058
						min_lat = latitude_recup - (deg_lat * 100);        // Récupère le reste 
						latitude = (float)deg_lat + (min_lat / 60.0);      // on calcule la vrai valeur

						// Longitude
						deg_lon = (int)(longitude_recup / 100);            
						min_lon = longitude_recup - (deg_lon * 100);       
						longitude = (float)deg_lon + (min_lon / 60.0);}
		 else;
		}
        
}



//MAIN

int main (void) {
    HAL_Init();

    //	Init UART3
    Driver_USART3.Initialize(myUART_callback);
    Driver_USART3.PowerControl(ARM_POWER_FULL);
    Driver_USART3.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8       |
                          ARM_USART_PARITY_NONE       |
                          ARM_USART_STOP_BITS_1       |
                          ARM_USART_FLOW_CONTROL_NONE , 9600);
    
    Driver_USART3.Control(ARM_USART_CONTROL_RX, 1); // Active la réception
    

    osKernelInitialize();  
    // Création de la tâche
    tid_GPS = osThreadNew(Thread_Traitement_GPS, NULL, NULL);
    
    osKernelStart();
    while(1);
}






