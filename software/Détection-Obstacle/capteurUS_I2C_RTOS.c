/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "stm32f4xx.h"                  // Device header
#include "Driver_I2C.h"                 // CMSIS Driver:I2C
#include "RTE_Device.h"                 // Device:STM32Cube Framework:Classic
#include "rtx_os.h"                     // CMSIS:RTOS2:Keil RTX5&&Source


extern ARM_DRIVER_I2C         Driver_I2C1;
#define I2C_A               (&Driver_I2C1)

osThreadId_t ID_tacheI2C;

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
__NO_RETURN static void tache_I2C (void ) {
  
    uint8_t addr_slv;
    uint8_t reg_com,reg_mes;
    uint8_t tab[10],tab2[10],tab3[10];
    uint8_t val_LSB,val_MSB;
    
    addr_slv= (0xE0>>1); //0xE0
    
    tab[0]=0x00; // registre de commande 
    tab[1]=0x51; // commande de mesure en cm
    
    //registre de mesure
    tab2[0]=0x02;// registre de mesure bit de poids fort
    tab3[0]=0x03;// registre de mesure bit de poids faible
    
  while(1){
        Driver_I2C1.MasterTransmit (addr_slv, tab, 2, false);// commande pour demander la mesure, registre 0
        osDelay(70);//70ms
        
        
        
        //Code lire valeur mesurée en cm
        Driver_I2C1.MasterTransmit(addr_slv, tab2, 1, true); // true = sans stop
        while (Driver_I2C1.GetStatus().busy == 1);// attente fin transmission
        
    Driver_I2C1.MasterReceive(addr_slv, &val_MSB, 1, false); // false = avec stop
        while (Driver_I2C1.GetStatus().busy == 1); // attente fin transmission
        
        Driver_I2C1.MasterTransmit(addr_slv, tab3, 1, true); // true = sans stop
        while (Driver_I2C1.GetStatus().busy == 1);// attente fin transmission
        
    Driver_I2C1.MasterReceive(addr_slv, &val_LSB, 1, false); // false = avec stop
        while (Driver_I2C1.GetStatus().busy == 1); // attente fin transmission
      osDelay(10);
    
    }
  
}

osThreadAttr_t configT1 = {.priority=osPriorityAboveNormal};

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  
    // Initialize and configure I2C 
    Driver_I2C1.Initialize(NULL); // début initialisation
    Driver_I2C1.PowerControl(ARM_POWER_FULL); // alimentation périphérique
    Driver_I2C1.Control( ARM_I2C_BUS_SPEED, // 2nd argument = débit
                                    ARM_I2C_BUS_SPEED_STANDARD ); // =100 kHz                                
    //Driver_I2C1.Control( ARM_I2C_BUS_CLEAR,0 ); // non utilisé
    
    
    
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  ID_tacheI2C = osThreadNew((osThreadFunc_t)tache_I2C, NULL, &configT1);    // Create application main thread
  
    
    osKernelStart();                      // Start thread execution
}



