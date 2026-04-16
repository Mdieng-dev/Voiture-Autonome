#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "stm32f4xx.h"
#include "Driver_I2C.h"
#include "Driver_CAN.h"
#include "RTE_Device.h"
#include "rtx_os.h"

extern ARM_DRIVER_I2C  Driver_I2C1;
#define I2C_A          (&Driver_I2C1)

extern ARM_DRIVER_CAN  Driver_CAN2;

/* --------------------------------------------------------------------------
 * Mutex pour protéger les bus I2C et CAN
 * --------------------------------------------------------------------------*/
osMutexId_t ID_mutex_I2C;
osMutexId_t ID_mutex_CAN;

/* --------------------------------------------------------------------------
 * IDs des tâches
 * --------------------------------------------------------------------------*/
osThreadId_t ID_tacheI2C_1, ID_tacheI2C_2;
osThreadId_t ID_tacheCAN_1, ID_tacheCAN_2;

/* --------------------------------------------------------------------------
 * Mailboxes (une par capteur)
 * --------------------------------------------------------------------------*/
osMessageQueueId_t ID_mailbox_dist1;   /* I2C_1 -> CAN_1 */
osMessageQueueId_t ID_mailbox_dist2;   /* I2C_2 -> CAN_2 */

/* --------------------------------------------------------------------------
 * Flags internes
 * --------------------------------------------------------------------------*/
#define I2C_EVENT_DONE  0x01U
#define CAN_EVENT_DONE  0x01U

/* --------------------------------------------------------------------------
 * Callbacks
 * --------------------------------------------------------------------------*/
void callbackI2C(uint32_t event) {
    /*
     * Avec un Mutex, une seule tâche attend ce flag à un instant T.
     * On notifie les deux, seule la tâche active consommera le flag.
     */
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        osThreadFlagsSet(ID_tacheI2C_1, I2C_EVENT_DONE);
        osThreadFlagsSet(ID_tacheI2C_2, I2C_EVENT_DONE);
    }
}

void callbackCAN(uint32_t obj_idx, uint32_t event) {
    if (event & ARM_CAN_EVENT_SEND_COMPLETE) {
        osThreadFlagsSet(ID_tacheCAN_1, CAN_EVENT_DONE);
        osThreadFlagsSet(ID_tacheCAN_2, CAN_EVENT_DONE);
    }
}

/* --------------------------------------------------------------------------
 * Initialisations
 * --------------------------------------------------------------------------*/
void init_I2C(void) {
    Driver_I2C1.Initialize(callbackI2C);
    Driver_I2C1.PowerControl(ARM_POWER_FULL);
    Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
}

void init_CAN(void) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    Driver_CAN2.Initialize(NULL, callbackCAN);
    Driver_CAN2.PowerControl(ARM_POWER_FULL);
    Driver_CAN2.SetMode(ARM_CAN_MODE_INITIALIZATION);
    Driver_CAN2.SetBitrate(ARM_CAN_BITRATE_NOMINAL,
                           125000,
                           ARM_CAN_BIT_PROP_SEG(5U)   |
                           ARM_CAN_BIT_PHASE_SEG1(1U) |
                           ARM_CAN_BIT_PHASE_SEG2(1U) |
                           ARM_CAN_BIT_SJW(1U));
    Driver_CAN2.ObjectConfigure(2, ARM_CAN_OBJ_TX);
    Driver_CAN2.ObjectConfigure(0, ARM_CAN_OBJ_RX);
    Driver_CAN2.SetMode(ARM_CAN_MODE_NORMAL);
}

/* --------------------------------------------------------------------------
 * Tâche I2C — capteur 1  (adresse 0xE4)
 * --------------------------------------------------------------------------*/
static void tache_I2C_1(void *argument) {
    const uint8_t addr       = (0xE4 >> 1);// a changer selon le capteur
    const uint8_t reg_cmd[2] = {0x00, 0x51};
    const uint8_t reg_msb    = 0x02;
    const uint8_t reg_lsb    = 0x03;
    uint8_t  vMSB, vLSB;
    uint16_t dist;

    while (1) {
        /* Bloquer le bus I2C pour cette tâche */
        osMutexAcquire(ID_mutex_I2C, osWaitForever);

        /* Lancer une mesure */
        Driver_I2C1.MasterTransmit(addr, reg_cmd, 2, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        osDelay(70);

        /* Lire MSB */
        Driver_I2C1.MasterTransmit(addr, &reg_msb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr, &vMSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);

        /* Lire LSB */
        Driver_I2C1.MasterTransmit(addr, &reg_lsb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr, &vLSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);

        dist = ((uint16_t)vMSB << 8) | vLSB;

        /* Publier la mesure */
        osMessageQueuePut(ID_mailbox_dist1, &dist, NULL, 0);

        
        osMutexRelease(ID_mutex_I2C);
				
				osDelay(250);
    }
}

/* --------------------------------------------------------------------------
 * Tâche I2C — capteur 2  (adresse 0xE8)
 * --------------------------------------------------------------------------*/
static void tache_I2C_2(void *argument) {
    const uint8_t addr       = (0xE8 >> 1);// a changer selon le capteur
    const uint8_t reg_cmd[2] = {0x00, 0x51};
    const uint8_t reg_msb    = 0x02;
    const uint8_t reg_lsb    = 0x03;
    uint8_t  vMSB, vLSB;
    uint16_t dist;

    while (1) {
        /* Bloquer le bus I2C pour cette tâche */
        osMutexAcquire(ID_mutex_I2C, osWaitForever);

        /* Lancer une mesure */
        Driver_I2C1.MasterTransmit(addr, reg_cmd, 2, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        osDelay(70);

        /* Lire MSB */
        Driver_I2C1.MasterTransmit(addr, &reg_msb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr, &vMSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);

        /* Lire LSB */
        Driver_I2C1.MasterTransmit(addr, &reg_lsb, 1, true);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
        Driver_I2C1.MasterReceive(addr, &vLSB, 1, false);
        osThreadFlagsWait(I2C_EVENT_DONE, osFlagsWaitAny, osWaitForever);
				
        dist = ((uint16_t)vMSB << 8) | vLSB;
				
        /* Publier la mesure */
        osMessageQueuePut(ID_mailbox_dist2, &dist, NULL, 0);
				
        osMutexRelease(ID_mutex_I2C);
				
				osDelay(250);
    }
}

/* --------------------------------------------------------------------------
 * Tâche CAN — capteur 1  (ID 0x110)
 * --------------------------------------------------------------------------*/
static void tache_CAN_1(void *argument) {
    ARM_CAN_MSG_INFO tx_msg_info = {0};
    tx_msg_info.rtr = 0;
    tx_msg_info.id  = ARM_CAN_STANDARD_ID(0x110);// a changer selon la carte

    uint16_t dist;
    uint8_t  data[2];

    while (1) {
        /* Attendre qu'une mesure soit disponible (bloquant) */
        osMessageQueueGet(ID_mailbox_dist1, &dist, NULL, osWaitForever);

        data[0] = dist ;

        /* Bloquer le contrôleur CAN avant d'envoyer */
        osMutexAcquire(ID_mutex_CAN, osWaitForever);

        Driver_CAN2.MessageSend(2, &tx_msg_info, &dist, 1);
        osThreadFlagsWait(CAN_EVENT_DONE, osFlagsWaitAny, osWaitForever);

        /* Libérer le contrôleur CAN */
        osMutexRelease(ID_mutex_CAN);
    }
}

/* --------------------------------------------------------------------------
 * Tâche CAN — capteur 2  (ID 0x111)
 * --------------------------------------------------------------------------*/
static void tache_CAN_2(void *argument) {
    ARM_CAN_MSG_INFO tx_msg_info = {0};
    tx_msg_info.rtr = 0;
    tx_msg_info.id  = ARM_CAN_STANDARD_ID(0x111);// a changer selon la carte

    uint16_t dist;
    uint8_t  data[2];

    while (1) {
        /* Attendre qu'une mesure soit disponible (bloquant) */
        osMessageQueueGet(ID_mailbox_dist2, &dist, NULL, osWaitForever);

        data[0] = dist ;
				
        /* Bloquer le contrôleur CAN avant d'envoyer */
        osMutexAcquire(ID_mutex_CAN, osWaitForever);

        Driver_CAN2.MessageSend(2, &tx_msg_info, &dist, 2);
        osThreadFlagsWait(CAN_EVENT_DONE, osFlagsWaitAny, osWaitForever);

        /* Libérer le contrôleur CAN */
        osMutexRelease(ID_mutex_CAN);
    }
}

/* --------------------------------------------------------------------------
 * Priorités
 * --------------------------------------------------------------------------*/
static const osThreadAttr_t cfg_high = {.priority = osPriorityAboveNormal};
static const osThreadAttr_t cfg_norm = {.priority = osPriorityNormal};

/* --------------------------------------------------------------------------
 * main
 * --------------------------------------------------------------------------*/
int main(void) {
    SystemCoreClockUpdate();

    init_CAN();
    init_I2C();

    osKernelInitialize();

    /* Création des mailboxes — 4 slots, éléments de type uint16_t */
    ID_mailbox_dist1 = osMessageQueueNew(4, sizeof(uint16_t), NULL);
    ID_mailbox_dist2 = osMessageQueueNew(4, sizeof(uint16_t), NULL);

    /* Création des Mutex */
    ID_mutex_I2C = osMutexNew(NULL);
    ID_mutex_CAN = osMutexNew(NULL);

    /* Création des tâches I2C (priorité haute) */
    ID_tacheI2C_1 = osThreadNew(tache_I2C_1, NULL, &cfg_high);
    ID_tacheI2C_2 = osThreadNew(tache_I2C_2, NULL, &cfg_high);

    /* Création des tâches CAN (priorité normale, bloquées sur la mailbox) */
    ID_tacheCAN_1 = osThreadNew(tache_CAN_1, NULL, &cfg_norm);
    ID_tacheCAN_2 = osThreadNew(tache_CAN_2, NULL, &cfg_norm);

    osKernelStart();

    while(1);
}
