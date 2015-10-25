#include <stdio.h>
#include <unistd.h>

#include "ecrt.h"

/****************************************************************************/
// EtherCAT
// Чтение в цикле всех регистров текущего набора PDO
// отображение тек.позиции и статуса

static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static ec_slave_config_t *sc_dig_out = NULL;
static ec_slave_config_t *sc_dig_in = NULL;
/****************************************************************************/

static uint8_t *domain1_pd = NULL;

// offsets for PDO entries
static unsigned int off_dig_out;
static unsigned int off_dig_in;
static unsigned int off_dig_in1;
static unsigned int off_n[30];

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {0, 0, 0x00007595, 0x00000000, 0x6040, 0, &off_n[0]},
    {0, 0, 0x00007595, 0x00000000, 0x6071, 0, &off_n[1]},
    {0, 0, 0x00007595, 0x00000000, 0x607a, 0, &off_n[2]},
    {0, 0, 0x00007595, 0x00000000, 0x6060, 0, &off_n[3]},
    {0, 0, 0x00007595, 0x00000000, 0x60b8, 0, &off_n[4]},

    {0, 0, 0x00007595, 0x00000000, 0x6041, 0, &off_dig_in1},

    {0, 0, 0x00007595, 0x00000000, 0x6077, 0, &off_n[5]},

    {0, 0, 0x00007595, 0x00000000, 0x6064, 0, &off_dig_in},

    {0, 0, 0x00007595, 0x00000000, 0x60f4, 0, &off_n[6]},
    {0, 0, 0x00007595, 0x00000000, 0x60fd, 0, &off_n[7]},
    {0, 0, 0x00007595, 0x00000000, 0x6061, 0, &off_n[8]},
    {0, 0, 0x00007595, 0x00000000, 0x2601, 0, &off_n[9]},
    {0, 0, 0x00007595, 0x00000000, 0x2600, 0, &off_n[10]},
    {0, 0, 0x00007595, 0x00000000, 0x60b9, 0, &off_n[11]},
    {0, 0, 0x00007595, 0x00000000, 0x60ba, 0, &off_n[12]},
    {}
};
static char slaves_up = 0;

/*****************************************************************************/
void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
#if 0
    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);
#endif
    domain1_state = ds;
}

/*****************************************************************************/
void check_master_state(void)
{    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    master_state = ms;
}

/*****************************************************************************/
void check_slave_config_states()
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc_dig_out, &s);
    if (slaves_up < 1 && s.al_state != 0x08) {
    printf("DigOut: State 0x%02X.\n", s.al_state);
    }
    if (slaves_up < 1 && s.al_state == 0x08) {
      slaves_up = 1;
    }
    ecrt_slave_config_state(sc_dig_in, &s);
    if (slaves_up < 2 && s.al_state != 0x08) {
      printf("DigIn: State 0x%02X.\n", s.al_state);
    }
    if (slaves_up < 2 && s.al_state == 0x08) {
      slaves_up = 2;
    }
}

/*****************************************************************************/
int main(void) //(int argc, char **argv)
{
    unsigned int j, in, in1, out, op_flag;
    ec_slave_config_t *sc;
    //ec_master_state_t ms;

    master = ecrt_request_master(0);
    if (!master)
      { fprintf(stderr, "Unable to get requested master.\n");
        return -1;
      }

    printf("Step1\n");

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
      { fprintf(stderr, "Unable to create process data domain.\n");
        return -1;
      }
    printf("Step2\n");

    sc = ecrt_master_slave_config(master, 0, 0, 0x00007595, 0x00000000);
    if (!sc) {
      fprintf(stderr, "Failed to get l7n configuration.\n");
      return -1;
    }
    printf("Step3\n");

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
    printf("Step4\n");

    printf("Activating master...");
        if (ecrt_master_activate(master)) {
          fprintf(stderr,"activation failed.\n");
          return -1;
        }
    printf("Master ok!\n");

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
      fprintf(stderr,"Domain data initialization failed.\n");
      return -1;
    }
    printf("Domain data registered ok.\n");

    check_master_state();
    check_domain1_state();

    printf("Step5\n");

    in = 0; out = 0; op_flag = 0;
    in1 = 0;

    //for (j = 0; j < 3; j++) {
    for(;;) {
        ecrt_master_receive(master);  //RECEIVE A FRAME
        ecrt_domain_process(domain1); //DETERMINE THE DATAGRAM STATES
       // check_slave_config_states();
       if (!op_flag) {
          check_domain1_state();
       }
       if (domain1_state.wc_state == EC_WC_COMPLETE && !op_flag) {
          printf("Domain is up at %d cycles.\n", j);
          op_flag = 1;
       }
       in = EC_READ_U32(domain1_pd + off_dig_in); //READ DATA 0x6064 position
       in1 = EC_READ_U16(domain1_pd + off_dig_in1); //READ DATA 0x6041 status
       printf("Position: %x Status: %x, ofs1=%u, ofs2=%u\n", in, in1, off_dig_in, off_dig_in1);

       // send process data
       ecrt_domain_queue(domain1); //MARK THE DOMAIN DATA AS READY FOR EXCHANGE
       ecrt_master_send(master);   //SEND ALL QUEUED DATAGRAMS
       usleep(1000); //WAIT 1mS
    }
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    printf("...Done. Releasing the master!\n");
    ecrt_release_master(master); //RELEASE THE MASTER INSTANCE
    return 0;
}

