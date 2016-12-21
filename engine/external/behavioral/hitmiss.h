/*
This class implements the hit/miss protocol
*/
#include "comm.h"

typedef enum result_t {
    HIT = 0,
    MISS = 1
    } result_t;
    
typedef enum protocol_state_t 
    {
        IDLE,
        PRETRIAL, 
        LICKTRIAL,
        WAIT4RESPONSE, 
        WATERREWARD,
        ENDOFTRIAL,
    }  protocol_state_t ;

class HitMiss:public Comm {
    public:
        HitMiss(void);
        void run(void);
    private:
    //Protocol parameters
        float laser_voltage;
        float laser_duration;
        float pre_trial_interval;
        float reponse_window_time;
        float water_dispense_delay;
        float water_dispense_time;
        float drink_time;
    //Output
        float number_of_licks;
        result_t result;
    //Other
        protocol_state_t state;
        unsigned long t_wait_for_response;
        unsigned long now;
        unsigned long milliseconds(void);
};
