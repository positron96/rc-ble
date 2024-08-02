#pragma once

#include "log.h"

#include <etl/vector.h>

#include <cstdint>
#include <cmath>
#include <cstddef>

namespace fn {

    // class Startable;
    // etl::vector<Startable*, 10> startables;

    // struct Startable {

    //     Startable() { startables.push_back(this); };
    //     virtual ~Startable() {
    //         auto it = std::find(startables.begin(), startables.end(), this);
    //         if(it != startables.end())
    //             startables.erase(it);
    //     }

    //     virtual void start() = 0;
    //     virtual void stop() = 0;
    // };

    struct Ticking {
        static constexpr size_t TICK_FREQ = 50;
        static constexpr size_t PERIOD_MS = 1000 / TICK_FREQ;
        static constexpr size_t ms_to_ticks(const size_t ms) { return ms/PERIOD_MS;};
        virtual void tick() = 0;
    };


    struct Hbridge {
        virtual void set(int8_t val) = 0;
    };

    struct Pin {
        virtual void set(bool val) = 0;
    };

    struct PwmPin: Pin {
        uint8_t val_on;

        virtual void set_pwm(uint8_t val) = 0;
        void set(bool val) override {
            set_pwm(val ? val_on : 0);
        }
    };

    struct Blinker: Ticking {
        Pin *pin;

        Blinker(Pin *p): pin{p}, half_period_ticks{ms_to_ticks(500)} {}

        void set(bool val) {
            if(val!=on) {
                //logf("blinker(%d) = %d\n", (size_t)pin, val);
                on = val;
                pin->set(val);
                phase = 0;
            }
        }
        void restart() {
            phase = 0;
        }
        void set_period(uint32_t ms) { half_period_ticks = ms_to_ticks(ms)/2; }
        void tick() override {
            phase++;
            if(phase == half_period_ticks) { pin->set(false);} else
            if(phase == half_period_ticks*2) { if(on){phase=0;pin->set(true);}}
        }
    private:
        size_t half_period_ticks;
        size_t phase = 0;
        bool on;
    };

    struct Servo {
        uint16_t center = 1500;
        int16_t half_range = 500;
        virtual void set_us(uint16_t val) = 0;
        void set(uint8_t val) {
            set_us(center + (val-127)*half_range/127);
        };
    };

    struct Fn {
        virtual void start() {};
        virtual void set(uint8_t val) = 0;
        virtual void stop() {};
    };

    constexpr int8_t to_signed(const uint8_t v) {
        return v - 128;
    }

    struct Driving: Fn, Ticking {
        Hbridge *hbridge;
        Pin *reverse_lights;
        Pin *brake_lights;
        size_t reverse_delay_ticks = ms_to_ticks(500);
        size_t brake_delay_ticks = ms_to_ticks(500);
        size_t reverse_ticks_left = 0;
        size_t brake_ticks_left = 0;
        int8_t nonzero_limit = 10;

        Driving(Hbridge *hbridge, Pin *rev_lights, Pin *brake_lights)
            : hbridge{hbridge}, reverse_lights{rev_lights}, brake_lights{brake_lights}
        {

        }

        void set(uint8_t val) override {
            // TODO: make it act like an actual gas pedal (to zero --> coast, negative -> break)
            int8_t last = value;
            value = to_signed(val);
            hbridge->set(value);
            if(value < -nonzero_limit) { reverse_lights->set(true); reverse_ticks_left = reverse_delay_ticks;}
            if (abs(value) < abs(last)) { brake_lights->set(true); brake_ticks_left = brake_delay_ticks; }
        }

        void tick() override {
            if(value < -nonzero_limit) reverse_ticks_left = reverse_delay_ticks;
            else {
                if(reverse_ticks_left==0) reverse_lights->set(false);
                else reverse_ticks_left --;
            }

            if(brake_ticks_left==0) {
                brake_lights->set(false);
            } else brake_ticks_left--;
        }
    private:
        int8_t value = 0;
    };

    struct Steering: Fn, Ticking {
        fn::Servo *servo;
        fn::Blinker *left;
        fn::Blinker *right;
        uint8_t nonzero_limit = 20;
        size_t blinker_ticks_left = 0;
        size_t blinker_timeout_ticks = ms_to_ticks(100);

        Steering(Servo *s, Blinker *l, Blinker *r) : servo{s}, left{l}, right{r} {}

        void set(uint8_t val) override {
            servo->set(val);
            value = to_signed(val);
            if (value > nonzero_limit) {
                left->set(true);
                right->set(false);
            } else
            if (value < -nonzero_limit) {
                right->set(true);
                left->set(false);
            } else {
                left->set(false);
                right->set(false);
            }
        }

        void reset_ticks() { blinker_ticks_left = blinker_timeout_ticks;}

        void tick() override {
            // if(abs(value) > nonzero_limit) reset_ticks();
            // if(blinker_ticks_left == 0) {
            //     right->set(false);
            //     left->set(false);
            // } else {
            //     blinker_ticks_left--;
            // }
        }
    private:
        int8_t value;
    };

    struct Simple: Fn {
        Pin *pin;

        Simple(Pin *p): pin{p} {}

        void set(uint8_t val) override {
            pin->set(val > 127);
        }
    };

};
