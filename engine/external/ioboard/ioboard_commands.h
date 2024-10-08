#include "comm.h"
#include "config.h"
#include "Arduino.h"

typedef enum state_t {
    ON,
    OFF
    } state_t;
    
#if ENABLE_STIMULUS_PHASE_LOCKING
typedef enum phase_lock_state_t {
   NOT_RUNNING,
   MEASURE_FPS,
   MEASURE_PHASE_ONLY,
   MEASURE_PHASE_ENABLE_LED   
} phase_lock_state_t;

#endif

typedef enum function_state_t {
   NO,
   ELONGATE_PULSE,
   FPS_MEASUREMENT,
   START_TRIGGER_DETECTOR,
   STOP_TRIGGER_DETECTOR,
} function_state_t;


class IOBoardCommands:public Comm {
    public:
        IOBoardCommands(void);
        void run(void);
        void isr(void);
        void waveform_isr(void);
        void int0_isr(void);
        void int1_isr(void);
    private:
        state_t read_state;
        state_t waveform_state;
        state_t elongate_state;
        float base_frequency;
        float frequency_range;
        float modulation_frequency;
        uint16_t waveform_frq_register;
        unsigned long time_ms;
        unsigned long phase_counter;
        int port;
        int port_last;
        float tmp;
        float tmp_isr;
        unsigned char debug;
        float elongate_output_pin;
        float elongate_duration;
        float elongate_delay;
        void set_pin(float pin,float value);
        void pulse(float pin,float duration);
        void waveform(float base_frequency, float frequency_range, float modulation_frequency);
        void start_read_pins(void);
        void stop_read_pins(void);
        void stop_waveform(void);
        void read_pins(unsigned char force);
        
        int enable_fps_measurement;
        unsigned long fps_buffer[TIMING_BUFFER_SIZE];
        unsigned char fps_buffer_index;
        unsigned long frame_interval_mean;
        unsigned long frame_interval_std_sqr;
        long frame_intervals[TIMING_BUFFER_SIZE];
        unsigned long pulse_counter, timestamp_buffer, timestamp_buffer_prev;
        int dt;
        function_state_t function_state;
        unsigned long last_pulse_ts, run_always_ts;
        void always_run(void);
};
