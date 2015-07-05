#include <stdio.h>
#include <unistd.h>

#include "ecrt.h"

/****************************************************************************/
// EtherCAT
// Перемещение в заданную позицию

static ec_master_t*         gkMaster = NULL;
static ec_master_state_t    gkMasterState = {};
static ec_domain_t*         gkDomain1 = NULL;
static ec_domain_state_t    gkDomain1State = {};
static ec_slave_config_t*   gkSlaveStateDigOut = NULL;
static ec_slave_config_t*   gkSlaveStateDigIn = NULL;

/****************************************************************************/

static uint8_t* gkDomain1PD = NULL;

// offsets for PDO entries
static uint32_t gkOffOControl;
static uint32_t gkOffOPos;
static uint32_t gkOffIStatus;
static uint32_t gkOffIPos;
static uint32_t gkOffIVel;
static uint32_t gkOffITorq;
static uint32_t gkOffOVel;

const static ec_pdo_entry_reg_t gkDomain1Regs[] = {
    {0, 1, 0x00007595, 0x00000000, 0x6040, 0, &gkOffOControl},
//    {0, 1, 0x00007595, 0x00000000, 0x607a, 0, &gkOffOPos},
    {0, 1, 0x00007595, 0x00000000, 0x6041, 0, &gkOffIStatus},
    {0, 1, 0x00007595, 0x00000000, 0x6064, 0, &gkOffIPos},
//    {0, 1, 0x00007595, 0x00000000, 0x606c, 0, &gkOffIVel},
//    {0, 1, 0x00007595, 0x00000000, 0x6077, 0, &gkOffITorq},
    {0, 1, 0x00007595, 0x00000000, 0x60ff, 0, &gkOffOVel},
    {}
};

static char gkSlavesUp = 0;

/*****************************************************************************/

void check_domain1_state() {
    ec_domain_state_t ds;
    ecrt_domain_state(gkDomain1, &ds);
#if 0
    if (ds.working_counter != gkDomain1State.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != gkDomain1State.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);
#endif
    gkDomain1State = ds;
}

/*****************************************************************************/
void check_master_state() {
    ec_master_state_t ms;
    ecrt_master_state(gkMaster, &ms);
    if (ms.slaves_responding != gkMasterState.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != gkMasterState.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != gkMasterState.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    gkMasterState = ms;
}

/*****************************************************************************/
void check_slave_config_states() {
    ec_slave_config_state_t s;
    ecrt_slave_config_state(gkSlaveStateDigOut, &s);
    if (gkSlavesUp < 1 && s.al_state != 0x08) {
        printf("DigOut: State 0x%02X.\n", s.al_state);
    }
    if (gkSlavesUp < 1 && s.al_state == 0x08) {
        gkSlavesUp = 1;
    }
    ecrt_slave_config_state(gkSlaveStateDigIn, &s);
    if (gkSlavesUp < 2 && s.al_state != 0x08) {
        printf("DigIn: State 0x%02X.\n", s.al_state);
    }
    if (gkSlavesUp < 2 && s.al_state == 0x08) {
        gkSlavesUp = 2;
    }
}

/*****************************************************************************/
int main(int argc, char **argv)
{
    // Создаем мастер-объект
    gkMaster = ecrt_request_master(0);

    if (gkMaster) {
        fprintf(stdout, "1. Master created.\n");
    } else {
        fprintf(stderr, "Unable to get requested master.\n");
        return -1;
    }

    // Создаем объект для обмена PDO в циклическом режиме.
    gkDomain1 = ecrt_master_create_domain(gkMaster);

    if (gkDomain1) {
        fprintf(stdout, "2. Process data domain created.\n");
    } else {
        fprintf(stderr, "Unable to create process data domain.\n");
        return -1;
    }

    // Создаем объект конфигурации подчиненного.
    ec_slave_config_t* sc = ecrt_master_slave_config(gkMaster, 0, 0, 0x00007595, 0x00000000);

    if (sc) {
        fprintf(stdout, "3. Slave configuration object created.\n");
    } else {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    // Конфигурируем PDO подчиненного
    // TxPDO
    ec_pdo_entry_info_t l7na_tx_channel1[] = {
        {0x6041, 0, 16},    // Statusword
        {0x6061, 0, 8},     // The Modes of Operation Display
        {0x6062, 0, 32},    // The Position Demand Value
        {0x6063, 0, 32},    // The Position Actual Internal Value
        {0x6064, 0, 32},    // The Position Actual Value
        {0x606B, 0, 32},    // The Velocity Demand Value
        {0x606C, 0, 32},    // The Actual Velocity Value
        {0x2600, 0, 16},    // The Current Speed (RPM)
        {0x2601, 0, 16},    // The Command Speed (RPM)
    };

    ec_pdo_info_t l7na_tx_pdos[] = {
        {0x1A00, 9, l7na_tx_channel1}
    };

    // RxPDO
    ec_pdo_entry_info_t l7na_rx_channel1[] = {
        {0x6041, 0, 16},    // Controlword
        {0x6060, 0, 8},     // Modes of Operation
        {0x607A, 0, 32},    // The Target Position
        {0x60FF, 0, 32},    // The Target Velocity (in Profile Velocity (Pv) mode and Cyclic Synchronous Velocity (Csv) modes)
        {0x6071, 0, 16},    // The Target Torque
    };

    ec_pdo_info_t l7na_rx_pdos[] = {
        {0x1600, 5, l7na_rx_channel1}
    };

    // Конфигурация SyncManagers 2 (FMMU0) и 3 (FMMU1)
    // { sync_mgr_idx, sync_mgr_direction, pdo_num, pdo_ptr, watch_dog_mode }
    // { 0xFF - end marker}
    ec_sync_info_t l7na_syncs[] = {
        {2, EC_DIR_OUTPUT, 1, l7na_rx_pdos, EC_WD_DISABLE},
        {3, EC_DIR_INPUT, 1, l7na_tx_pdos, EC_WD_DISABLE},
        {0xFF}
    };

    if (ecrt_slave_config_pdos(sc, EC_END, l7na_syncs)) {
        fprintf(stderr, "Failed to configure slave pdo.\n");
        return -1;
    }

    fprintf(stdout, "4. Configuring slave PDOs.\n");

    ecrt_slave_config_sdo16( sc, 0x1C12, 1, 0x1600 ); /* list all RxPdo in 0x1C12:1-4 */
    ecrt_slave_config_sdo8( sc, 0x1C12, 0, 1 ); /* set number of RxPDO */

    ecrt_slave_config_sdo16( sc, 0x1C13, 1, 0x1A00 ); /* list all TxPdo in 0x1C13:1-4 */
    ecrt_slave_config_sdo16( sc, 0x1C13, 2, 0x1A01 ); /* list all TxPdo in 0x1C13:1-4 */
    ecrt_slave_config_sdo8( sc, 0x1C13, 0, 2 ); /* set number of TxPDO */

    fprintf(stdout, "5. Configuring slave SDOs and sync managers done.\n");

    // Регистируем PDO в домене
    if (ecrt_domain_reg_pdo_entry_list(gkDomain1, gkDomain1Regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    fprintf(stdout, "6. PDO entries registered in domain.\n");

    if (ecrt_master_activate(gkMaster)) {
        fprintf(stderr,"Master activation failed.\n");
        return -1;
    }

    fprintf(stdout, "7. Master activated.\n");

    if (!(gkDomain1PD = ecrt_domain_data(gkDomain1))) {
      fprintf(stderr,"Domain data initialization failed.\n");
      return -1;
    }

    fprintf(stdout, "8. Domain data registered.\n");

    check_master_state();
    check_domain1_state();

    uint32_t op_flag = 0, ipos = 0, istatus = 0;

    //ждать режим OP
    for(uint32_t j = 0; ; j++) {
        ecrt_master_receive(gkMaster);  //RECEIVE A FRAME
        ecrt_domain_process(gkDomain1); //DETERMINE THE DATAGRAM STATES
       // check_slave_config_states();
       if (! op_flag) {
          check_domain1_state();
       }
       if (gkDomain1State.wc_state == EC_WC_COMPLETE && !op_flag) {
          printf("Domain is up at %d cycles.\n", j);
          op_flag = 1;
       }
       ipos = EC_READ_U32(gkDomain1PD + gkOffIPos); //READ DATA 0x6064 position
       istatus = EC_READ_U16(gkDomain1PD + gkOffIStatus); //READ DATA 0x6041 status

       // send process data
       ecrt_domain_queue(gkDomain1); //MARK THE DOMAIN DATA AS READY FOR EXCHANGE
       ecrt_master_send(gkMaster);   //SEND ALL QUEUED DATAGRAMS
       usleep(1000); //WAIT 1mS

       if (op_flag) {
            printf("1-Position: %d Status: 0x%x\n", ipos, istatus);
            break;
       }
    }

    fprintf(stdout, "8. Got OP state.\n");

    if(argc > 1) {

        //перейти в позицию
        const int cmdpos = atoi(argv[1]);
        uint32_t cmd, val;
        printf("cmd pos: %d\n", cmdpos);

        ecrt_master_receive(gkMaster);
        ecrt_domain_process(gkDomain1);
        cmd = 15;
        EC_WRITE_U16(gkDomain1PD + gkOffOControl, cmd); //0x6040 ControlWord
        ecrt_domain_queue(gkDomain1);
        ecrt_master_send(gkMaster);
        usleep(1000);

        //wait
        for (uint32_t i = 0; i < 200; ++i) {
            ecrt_master_receive(gkMaster);
            ecrt_domain_process(gkDomain1);
            ecrt_domain_queue(gkDomain1);
            ecrt_master_send(gkMaster);
            usleep(1000);
        }


        ecrt_master_receive(gkMaster);
        ecrt_domain_process(gkDomain1);
        val = cmdpos;
        EC_WRITE_U32(gkDomain1PD + gkOffOVel, val);
        ecrt_domain_queue(gkDomain1);
        ecrt_master_send(gkMaster);
        usleep(1000);

        //wait
        for (uint32_t i = 0; i < 200; ++i) {
            ecrt_master_receive(gkMaster);
            ecrt_domain_process(gkDomain1);
            ecrt_domain_queue(gkDomain1);
            ecrt_master_send(gkMaster);
            usleep(1000);
        }

/*        ecrt_master_receive(gkMaster);
        ecrt_domain_process(gkDomain1);
        cmd = 31;
        EC_WRITE_U16(gkDomain1PD + gkOffOControl, cmd);
        //EC_WRITE_U32(gkDomain1PD + gkOffOPos, val);
        ecrt_domain_queue(gkDomain1);
        ecrt_master_send(gkMaster);
        usleep(1000);
*/
        //wait
        for (uint32_t i = 0; i < 1000; ++i) {
            ecrt_master_receive(gkMaster);
            ecrt_domain_process(gkDomain1);
            ecrt_domain_queue(gkDomain1);
            ecrt_master_send(gkMaster);
            usleep(1000);
        }

#if 1
        for (uint32_t j = 0; ; j++) {
           ecrt_master_receive(gkMaster);
           ecrt_domain_process(gkDomain1);
           uint32_t ipos_new = EC_READ_U32(gkDomain1PD + gkOffIPos); //READ DATA 0x6064 position
           uint32_t istatus_new = EC_READ_U16(gkDomain1PD + gkOffIStatus); //READ DATA 0x6041 status
            if (ipos_new != ipos || istatus_new != istatus) {
                ipos = ipos_new;
                istatus = istatus_new;
                printf("Position: %d Status: 0x%x\n", ipos, istatus);
            }
//          ecrt_domain_queue(gkDomain1);
//          ecrt_master_send(gkMaster);
           usleep(1000); //WAIT 1mS

           if(ipos == cmdpos) {
               printf("2-Position: %x Status: %x\n", ipos, istatus);
               break;
           }
        }
#endif
    }

    ecrt_master_receive(gkMaster);
    ecrt_domain_process(gkDomain1);

    printf("...Done. Releasing the master!\n");

    // Освобождаем мастер-объект
    ecrt_release_master(gkMaster);

    return 0;
}
