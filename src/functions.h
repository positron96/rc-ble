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
        virtual void set(uint8_t val, bool fwd) = 0;
    };

    struct Pin {
        virtual void set(bool val) = 0;
        void on() { set(true); }
        void off() { set(false); }
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
            on = false;
            set(true);
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
        virtual void wake() {};
        virtual void set(uint8_t val) = 0;
        virtual void sleep() {};
    };

    constexpr int8_t to_signed(const uint8_t v) {
        return v - 128;
    }

    class DelayedOff: public Ticking {
        size_t ticks_left = 0;
        const size_t reset_value;
    public:
        DelayedOff(size_t delay_ms): reset_value{ms_to_ticks(delay_ms)} {}
        void set(bool val = true) { if(val) { ticks_left = reset_value; } }
        void force_off() { ticks_left=0; }
        void tick() {
            if(ticks_left>0) {
                ticks_left--;
            }
        }
        bool get() { return ticks_left>0; }
    };

    /**
     * A pin that retains its ON state after some time of being switched to OFF.
     * 
     * Useful to not make reverse or brake lights too short.
     */
    class DelayedOffPin: public Pin, public DelayedOff {
        Pin &pin;
    public:
        DelayedOffPin(Pin &pin, size_t delay_ms): DelayedOff{delay_ms}, pin{pin} {}
        void set(bool val) { DelayedOff::set(val); if(val) { pin.set(true); } }
        void force_off() { DelayedOff::force_off(); pin.set(false);}
        void tick() {
            bool was_on = get();            
            DelayedOff::tick();            
            if(was_on && get()==false) { pin.set(false); }
        }
    };

    struct SmoothValue: Ticking {
        using extl_t = uint8_t;
        extl_t curr=0, target=0;
        extl_t rate=1;
        void reset() { curr=0; target=0; }
        void tick() override {
            int16_t err = curr - target;
            if(err!=0) {
                if(abs(err) > rate) { curr -= (err>0?1:-1)*rate; } else curr = target;
            }
        }
    };

    struct Driving: Fn, Ticking {
        Hbridge *hbridge;
        DelayedOffPin reverse_lights;
        DelayedOffPin brake_lights;
        int8_t deadzone = 5;
        int8_t current_input;
        uint8_t has_been_idle = 0; ///< used to check if we can go in reverse

        SmoothValue output;
        bool current_fwd;

        Driving(Hbridge *hbridge, Pin *rev_lights, Pin *brake_lights):
            hbridge{hbridge}, 
            reverse_lights{*rev_lights, 500}, 
            brake_lights{*brake_lights, 500}
        { }

        void sleep() override {
            reverse_lights.force_off();
            brake_lights.force_off();
            hbridge->set(0, true);
        }
        void wake() override {
            reverse_lights.force_off();
            brake_lights.force_off();
            hbridge->set(0, true);
            output.reset();
        }

        void set(uint8_t val) override {
            // TODO: make it act like an actual gas pedal (to zero --> coast, negative -> break)
            current_input = to_signed(val);
            uint8_t abs_input = abs(current_input);
            bool fwd = abs_input>0;
            if(abs_input < deadzone) {
                current_input = (abs_input - deadzone) * 128 / (128 - deadzone);
                if(!fwd) current_input *= -1;
            }
        }

        void tick() override {
            int16_t inp = current_input;
            
            // invert input when going backwards,
            // so positive is always accelerate, 
            // negative is brake or change dir
            if(!current_fwd) inp = -inp; 

            if(output.curr == 0 && inp == 0) has_been_idle = 1;
            else if(output.curr != 0) has_been_idle = 0;

            if(has_been_idle && inp < 0) {
                // only allow reversing when stopped and throttle idle
                current_fwd = !current_fwd;
            }

            if(current_input > 0) {
                // drive
                output.target = current_input;
                output.rate = 5;
            } else 
            if(current_input == 0) {
                // slowly stop
                output.target = 0;
                output.rate = 1;
            } else {
                // decelerate
                output.target = 0;
                output.rate = 1 + abs(inp)/5;
                // brake light on
                brake_lights.set(true);
            }

            output.tick();

            hbridge->set(output.curr, current_fwd);

            if(output.curr < 0) reverse_lights.set(true);
            
            brake_lights.tick();
            reverse_lights.tick();            
        }

    };

    struct Steering: Fn, Ticking {
        fn::Servo *servo;
        fn::Blinker *left;
        fn::Blinker *right;
        DelayedOff delay;
        uint8_t deadzone = 20;
        uint8_t light_on_limit = 40;
        static constexpr size_t BlinkerPeriod = 1000;

        Steering(Servo *s, Blinker *l, Blinker *r):
            servo{s}, left{l}, right{r}, delay{100}
        {}

        void sleep() override {
            left->set(false);
            right->set(false);
        }
        void wake() override {
            left->set_period(BlinkerPeriod);
            right->set_period(BlinkerPeriod);
            delay.force_off();
        }

        void set(uint8_t val) override {
            servo->set(val);
            value = to_signed(val);
            if (value > light_on_limit) {
                left->set(true); right->set(false);
                delay.set();
            } else
            if (value < -light_on_limit) {
                right->set(true); left->set(false);
                delay.set();
            } 
        }

        void tick() override {
            delay.tick();
            if(delay.get() == false) { left->set(false); right->set(false); }
            left->tick();
            right->tick();
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

        void sleep() override {
            pin->set(false);
        }

    };

};
