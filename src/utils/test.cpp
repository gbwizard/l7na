#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "ecrt.h"

/****************************************************************************/
// EtherCAT
// Перемещение в заданную позицию

static ec_master_t*         gkMaster = NULL;
static ec_master_state_t    gkMasterState = {};
static ec_domain_t*         gkDomain1 = NULL;
static ec_domain_state_t    gkDomain1State = {};

/****************************************************************************/

static uint8_t* gkDomain1PD = NULL;

// offsets for PDO entries
static uint32_t gkOffOControl;
static uint32_t gkOffOPos;
static uint32_t gkOffIStatus;
static uint32_t gkOffIPos;
static uint32_t gkOffIVel;
static uint32_t gkOffPVel;
static uint32_t gkOffITorq;
static uint32_t gkOffDVel, gkOffTVel;
static uint32_t gkOffDPos;
static uint32_t gkOffOMode, gkOffIMode;

const static uint32_t gkDriveNum = 0;

const static ec_pdo_entry_reg_t gkDomain1Regs[] = {
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6040, 0, &gkOffOControl},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x607a, 0, &gkOffOPos},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6041, 0, &gkOffIStatus},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6062, 0, &gkOffDPos},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6064, 0, &gkOffIPos},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x606b, 0, &gkOffDVel},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x606c, 0, &gkOffIVel},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6081, 0, &gkOffPVel},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6060, 0, &gkOffOMode},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6061, 0, &gkOffIMode},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x60ff, 0, &gkOffTVel},
    {0, gkDriveNum, 0x00007595, 0x00000000, 0x6077, 0, &gkOffITorq},
    {}
};

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
    ec_slave_config_t* sc = ecrt_master_slave_config(gkMaster, 0, gkDriveNum, 0x00007595, 0x00000000);

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
        {0x6064, 0, 32},    // The Position Actual Value
        {0x606B, 0, 32},    // The Velocity Demand Value
        {0x6081, 0, 32},    // The Profile Velocity
        {0x606C, 0, 32},    // The Actual Velocity Value
        {0x607A, 0, 32},    // The Target Position
        {0x6077, 0, 16},    // Actual torque value
//        {0x200F, 0, 16},    // Position Scale Denominator
    };

    ec_pdo_info_t l7na_tx_pdos[] = {
        {0x1A00, 9, l7na_tx_channel1}
    };

    // RxPDO
    ec_pdo_entry_info_t l7na_rx_channel1[] = {
        {0x6040, 0, 16},    // Controlword
        {0x6060, 0, 8},     // Modes of Operation
        {0x607A, 0, 32},    // The Target Position
        {0x606C, 0, 32},    // The Velocity Demand value
        {0x6081, 0, 32},    // The Profile Velocity
        {0x60FF, 0, 32},    // The Target Velocity (in Profile Velocity (Pv) mode and Cyclic Synchronous Velocity (Csv) modes)
        {0x6071, 0, 16},    // The Target Torque
    };

    ec_pdo_info_t l7na_rx_pdos[] = {
        {0x1600, 7, l7na_rx_channel1}
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

    fprintf(stdout, "4. Configuring slave PDOs and sync managers done.\n");

    // Регистируем PDO в домене
    if (ecrt_domain_reg_pdo_entry_list(gkDomain1, gkDomain1Regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    fprintf(stdout, "5. PDO entries registered in domain.\n");

    if (ecrt_master_activate(gkMaster)) {
        fprintf(stderr,"Master activation failed.\n");
        return -1;
    }

    fprintf(stdout, "6. Master activated.\n");

    if (!(gkDomain1PD = ecrt_domain_data(gkDomain1))) {
      fprintf(stderr,"Domain data initialization failed.\n");
      return -1;
    }

    fprintf(stdout, "7. Domain data registered.\n");

//goto end;

    check_master_state();
    check_domain1_state();

    int32_t op_flag = 0, ipos = 0;
    uint16_t istatus = 0;

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
        printf("cmd pos: %d\n", cmdpos);

        ecrt_master_receive(gkMaster);
        ecrt_domain_process(gkDomain1);
        EC_WRITE_U16(gkDomain1PD + gkOffOControl, 0xF); //0x6040 ControlWord
        EC_WRITE_U8(gkDomain1PD + gkOffOMode, 1); // 0x6060 Profile position mode // 3 - for velocity mode, 1- for position mode
        EC_WRITE_S32(gkDomain1PD + gkOffPVel, 1000000); // 0x60ff profile velocity // gkOffTVel - for velocity mode
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
/* comment 2 lines for velocity mode */
        EC_WRITE_S32(gkDomain1PD + gkOffOPos, cmdpos);
        EC_WRITE_U16(gkDomain1PD + gkOffOControl, 0x11F);
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
        EC_WRITE_S32(gkDomain1PD + gkOffOPos, cmdpos);
        ecrt_domain_queue(gkDomain1);
        ecrt_master_send(gkMaster);
        usleep(1000);*/

        //wait
/*        for (uint32_t i = 0; i < 1000; ++i) {
            ecrt_master_receive(gkMaster);
            ecrt_domain_process(gkDomain1);
            ecrt_domain_queue(gkDomain1);
            ecrt_master_send(gkMaster);
            usleep(1000);
        }
*/

        timespec tbegin, tend;
        ::clock_gettime(CLOCK_MONOTONIC, &tbegin);
        printf("Time begin: %lds/%ldns\n", tbegin.tv_sec, tbegin.tv_nsec);
        const uint32_t kIterationMax = 500000;
        uint32_t change_count = 0;

        bool target_reached = false;

        for (uint32_t j = 0; ; j++) {
           ecrt_master_receive(gkMaster);
           ecrt_domain_process(gkDomain1);
           int32_t ipos_new = EC_READ_S32(gkDomain1PD + gkOffIPos); //READ DATA 0x6064 position
           uint16_t istatus_new = EC_READ_U16(gkDomain1PD + gkOffIStatus); //READ DATA 0x6041 status
            int32_t imode = EC_READ_S8(gkDomain1PD + gkOffIMode);
            int32_t ipvel = EC_READ_S32(gkDomain1PD + gkOffPVel);
            int32_t idvel = EC_READ_S32(gkDomain1PD + gkOffDVel);
            int32_t iavel = EC_READ_S32(gkDomain1PD + gkOffIVel);
            int32_t idpos = EC_READ_S32(gkDomain1PD + gkOffDPos);
            int32_t itpos = EC_READ_S32(gkDomain1PD + gkOffOPos);
            int32_t icontrol = EC_READ_U16(gkDomain1PD + gkOffOControl);
            int16_t iatorq = EC_READ_S16(gkDomain1PD + gkOffITorq);
//            int32_t ipdenom = EC_READ_S16(gkDomain1PD + gkOffPDenom);
            if (ipos_new != ipos) {
                ipos = ipos_new;
                change_count++;
                printf("Position: %d Status: 0x%x Mode: %d ATorq: %d PVel: %d DVel: %d AVel: %d DPos: %d TPos: %d OControl: 0x%x\n", ipos, istatus, imode, iatorq, ipvel, idvel, iavel, idpos, itpos, icontrol);
            }

// position mode
            if(! target_reached && ((istatus_new >> 10) & 0x1)) {
                ::clock_gettime(CLOCK_MONOTONIC, &tend);
                printf("Target reached. Pos: %d Status: 0x%x TEnd=%lds/%ldns\n", ipos, istatus, tend.tv_sec, tend.tv_nsec);
                target_reached = true;
                //break;
           }

/* Velocity mode */
        if (j == kIterationMax) {
/*            clock_gettime(CLOCK_MONOTONIC, &tend);
            printf("Iterations=%d, change_count=%d. time_end=%lds/%ldns Stopping...\n", j, change_count, tend.tv_sec, tend.tv_nsec);
            EC_WRITE_U16(gkDomain1PD + gkOffOControl, 0x6);
            break;
*/
        }
            
          ecrt_domain_queue(gkDomain1);
          ecrt_master_send(gkMaster);
           usleep(100); //WAIT 1mS

        }
    }

    ecrt_master_receive(gkMaster);
    ecrt_domain_process(gkDomain1);

    printf("...Done. Releasing the master!\n");

    // Освобождаем мастер-объект
    ecrt_release_master(gkMaster);

    return 0;
}
