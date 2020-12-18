#ifndef __TIMERW_H__
#define __TIMERW_H__


#include "driver/timer.h"


// class Timer {
//     private:
//         timer_group_t _group;
//         timer_idx_t _timer;
//         timer_config_t _config;

//     public:
//         Timer(timer_group_t group=TIMER_GROUP_0, 
//             timer_idx_t timer=TIMER_0,
//             timer_alarm_t alarm=TIMER_ALARM_DIS,
//             timer_start_t counter=TIMER_PAUSE,
//             timer_intr_mode_t intr_type=TIMER_INTR_LEVEL,
//             timer_count_dir_t count_dir=TIMER_COUNT_UP,
//             timer_autoreload_t auto_relaod=TIMER_AUTORELOAD_DIS,
//             uint32_t divider=2) : /*range is from from 2 to 65536. */
//         _group(group), _timer(timer)
//         {
//             _config = {
//                 .alarm_en=alarm,
//                 .counter_en=counter,
//                 .intr_type=intr_type,
//                 .counter_dir=count_dir,
//                 .auto_reload=auto_relaod,
//                 .divider=divider,   
//                 // .timer_src_clk_t=TIMER_SRC_CLK_APB,
//             };
//             init();
//             set();
//         }

//         inline void init() {
//             timer_init(_group, _timer, &_config);
//         }

//         inline void set(uint64_t value= 0x00000000ULL) {
//             timer_set_counter_value(_group, _timer, value);
//         }
        
//         inline void start() {
//             timer_start(_group, _timer);
//         }

//         inline void pause() {
//             timer_pause(_group, _timer);
//         }

//         inline uint64_t get() {
//             uint64_t value;
//             timer_get_counter_value(_group, _timer, &value);
//             return value;
//         }

//         inline double get_time_sec() {
//             double value;
//             timer_get_counter_time_sec(_group, _timer, &value);
//             return value;
//         }

//         inline void set_divider(uint32_t divider) {
//             timer_set_divider(_group, _timer, divider);
//         }

//         inline void set_counter_mode(timer_count_dir_t counter_dir) {
//             timer_set_counter_mode(_group, _timer, counter_dir);
//         }

//         inline void set_auto_relaod(timer_autoreload_t reload) {
//             timer_set_auto_reload(_group, _timer, reload);
//         }

//         void print() {
//             uint64_t counter_value = get();
//             printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
//                 (uint32_t) (counter_value));
//             printf("Time   : %.8f s\n", (double) get_time_sec());        
//         }
// };


class Stopwatch {
    private:
        timer_group_t _group;
        timer_idx_t _timer;
        timer_config_t _config;

    public:
        Stopwatch(
            timer_group_t group=TIMER_GROUP_0, 
            timer_idx_t timer=TIMER_0,
            timer_start_t counter=TIMER_PAUSE,
            uint32_t divider=2) : /*range is from from 2 to 65536. */
        _group(group), _timer(timer)
        {
            _config = {
                .alarm_en=TIMER_ALARM_DIS,
                .counter_en=counter,
                .intr_type=TIMER_INTR_LEVEL,
                .counter_dir=TIMER_COUNT_UP,
                .auto_reload=TIMER_AUTORELOAD_DIS,
                .divider=divider,   
                // .timer_src_clk_t=TIMER_SRC_CLK_APB,
            };
            init();
        }

        inline void init() {
            timer_init(_group, _timer, &_config);
            reset();
        }

        inline void reset() {
            timer_set_counter_value(_group, _timer, 0x00000000ULL);
        }
        
        inline void start() {
            timer_start(_group, _timer);
        }

        inline void pause() {
            timer_pause(_group, _timer);
        }

        inline double stop() {
            pause();
            double t = get_time_sec();
            reset();
            return t;
        }

        inline uint64_t get() {
            uint64_t value;
            timer_get_counter_value(_group, _timer, &value);
            return value;
        }

        inline double get_time_sec() {
            double value;
            timer_get_counter_time_sec(_group, _timer, &value);
            return value;
        }

        inline void set_divider(uint32_t divider) {
            timer_set_divider(_group, _timer, divider);
        }

        void print() {
            uint64_t counter_value = get();
            printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                (uint32_t) (counter_value));
            printf("Time   : %.8f s\n", (double) get_time_sec());        
        }
};

#endif // __TIMERW_H__