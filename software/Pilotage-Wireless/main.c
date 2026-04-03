#include "LPC17xx.h"
#include "cmsis_os2.h"
#include "Driver_I2C.h"
#include "Driver_USART.h"

extern ARM_DRIVER_I2C Driver_I2C2;
extern ARM_DRIVER_USART Driver_USART1; 

#define ADDR 0x52

void tache_lecture_et_envoi(void *arg) {
    uint8_t cmd = 0x00;
    uint8_t data_i2c[6];
    uint8_t data_bt[5]; 

    //  Initialisation I2C (Nunchuck)
    Driver_I2C2.Initialize(NULL);
    Driver_I2C2.PowerControl(ARM_POWER_FULL);
    Driver_I2C2.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);

    // Initialisation UART (Bluetooth) à 115200 bauds
    Driver_USART1.Initialize(NULL);
    Driver_USART1.PowerControl(ARM_POWER_FULL);
    Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | 
                          ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | 
                          ARM_USART_FLOW_CONTROL_NONE, 115200);
    Driver_USART1.Control(ARM_USART_CONTROL_TX, 1); 

    // Réveil Nunchuck
    uint8_t t1[2] = {0xF0, 0x55};
    uint8_t t2[2] = {0xFB, 0x00};
    Driver_I2C2.MasterTransmit(ADDR, t1, 2, false);
    osDelay(20);
    Driver_I2C2.MasterTransmit(ADDR, t2, 2, false);
    osDelay(20);

    while(1) {
        // Lecture I2C
        Driver_I2C2.MasterTransmit(ADDR, &cmd, 1, false);
        osDelay(5);
        Driver_I2C2.MasterReceive(ADDR, data_i2c, 6, false);
        osDelay(5); 

        // Préparation du paquet Bluetooth
        data_bt[0] = 0xFF; // Octet de synchronisation
        data_bt[1] = data_i2c[0]; // X
        data_bt[2] = data_i2c[1]; // Y

        
        // On interdit aux valeurs X et Y d'atteindre 0xFF (255)
        if (data_bt[1] >= 0xFF) data_bt[1] = 254;
        if (data_bt[2] >= 0xFF) data_bt[2] = 254;

        // Boutons
        data_bt[3] = !(data_i2c[5] & 0x01); // Bouton Z 
        data_bt[4] = !(data_i2c[5] & 0x02); // Bouton C 

        // Envoi via Bluetooth
        Driver_USART1.Send(data_bt, 5);
        
        osDelay(20); // 50 Hz, mise en sommeil
    }
}

int main(void) {
    SystemInit();
    osKernelInitialize();
    osThreadNew(tache_lecture_et_envoi, NULL, NULL);
    osKernelStart();
    while(1);
}
