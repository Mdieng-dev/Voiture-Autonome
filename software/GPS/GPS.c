#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Driver_USART.h"
#include <stdio.h>   
#include <string.h>  // Pour strncmp


/*extern ARM_DRIVER_USART Driver_USART3;
uint8_t buffer_gps;   // Octet reçu par l'UART
char trame[100];      // Tableau pour stocker la phrase complète
int index_trame = 0;  

float latitude, longitude;
int fix_quality;


void extraire_position(char *t);


int main(void) {
    HAL_Init();
    

    Driver_USART3.Initialize(NULL);
    Driver_USART3.PowerControl(ARM_POWER_FULL);
    Driver_USART3.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8       |
                          ARM_USART_PARITY_NONE       |
                          ARM_USART_STOP_BITS_1       |
                          ARM_USART_FLOW_CONTROL_NONE ,
                          9600);
    Driver_USART3.Control(ARM_USART_CONTROL_RX, 1);

    while (1) {
        Driver_USART3.Receive(&buffer_gps, 1);
        while (Driver_USART3.GetRxCount() < 1);

        if (buffer_gps == '$') {
            index_trame = 0;
        }
        
        
        trame[index_trame++] = (char)buffer_gps;

        // Si on reçoit un retour à la ligne '\n', la phrase est finie
        if (buffer_gps == '\n') {
            trame[index_trame] = '\0'; 
            extraire_position(trame);  
            index_trame = 0;           
        }
				if (index_trame > 100) {index_trame = 0;}
    }
}


void extraire_position(char *t) {
    // On ne traite que la trame qui contient la position (GGA)
    if (strncmp(t, "$GPGGA", 6) == 0) {
        // On découpe la trame pour extraire Latitude, Longitude et Qualité du Fix
        // Le %*f permet d'ignorer l'heure, le %*c ignore les lettres N/S/E/W
        sscanf(t, "$GPGGA,%*f,%f,%*c,%f,%*c,%d", &latitude, &longitude, &fix_quality);
        
        if (fix_quality > 0) {
            
        }
    }
}*/




extern ARM_DRIVER_USART Driver_USART3;
osThreadId_t tid_GPS;           
uint8_t buffer_uart;            
char trame_complete[100];
int idx = 0;


float latitude, longitude;
int fix_q;


// Cette fonction est lancée AUTOMATIQUEMENT à chaque événement sur l'UART
void myUART_callback(uint32_t event) {
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        osThreadFlagsSet(tid_GPS, 0x01); 
    }
}


void parser_gps(char *t) {
    if (strncmp(t, "$GPGGA", 6) == 0) {
        sscanf(t, "$GPGGA,%*f,%f,%*c,%f,%*c,%d", &latitude, &longitude, &fix_q);
    }
}


void GPS_Thread(void *argument) {
	
    Driver_USART3.Initialize(myUART_callback); 
    Driver_USART3.PowerControl(ARM_POWER_FULL);
    Driver_USART3.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8       |
                          ARM_USART_PARITY_NONE       |
                          ARM_USART_STOP_BITS_1       |
                          ARM_USART_FLOW_CONTROL_NONE , 9600);
    
    Driver_USART3.Control(ARM_USART_CONTROL_RX, 1);

    while (1) {
        // On lance une demande de réception d'un octet
        Driver_USART3.Receive(&buffer_uart, 1);
        
        // SOMMEIL ATTENTE RÉCEPTION (Comme à la Slide 15)
        // Le thread s'arrête ici et ne consomme plus de CPU
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        // Si on arrive ici, c'est que le callback nous a réveillé !
        if (buffer_uart == '$') idx = 0;
        
        if (idx < 99) {
            trame_complete[idx++] = (char)buffer_uart;
        }

        if (buffer_uart == '\n') {
            trame_complete[idx] = '\0';
            parser_gps(trame_complete);
            idx = 0;
        }
    }
}


int main (void) {
    HAL_Init();
	
    osKernelInitialize(); 

    tid_GPS = osThreadNew(GPS_Thread, NULL, NULL);

    if (osKernelGetState() == osKernelReady) {
        osKernelStart(); 
    }

    while(1);
}



