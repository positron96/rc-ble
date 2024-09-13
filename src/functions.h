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

    /** Rounded division. */
    template<typename T, typename U, std::enable_if_t<std::is_unsigned_v<U> >* =nullptr >
    T rdiv(const T x, const U y) {
        T quot = x / y;
        T rem = x % y;
        U ty = y-1;
        if (x >= 0) {
            return quot + (rem > (ty/2));
        } else {
            return quot - (rem < (-ty/2));
        }
    }

    struct Ticking {
        static constexpr size_t TICK_FREQ = 50;
        static constexpr size_t PERIOD_MS = 1000 / TICK_FREQ;
        static constexpr size_t ms_to_ticks(const size_t ms) { return ms/PERIOD_MS;};
        virtual void tick() = 0;
    };

    struct BasePin {
        virtual void set(bool val) = 0;
        void on() { set(true); }
        void off() { set(false); }
    };

    struct BaseAnalogPin: BasePin {
        uint8_t val_on = 255;

        virtual void set_pwm(uint8_t val) = 0;
        void set(bool val) override {
            set_pwm(val ? val_on : 0);
        }
    };

    struct BaseHbridge {
        bool inverted = false; ///< don't forget to use it in implementation
        virtual void set(uint8_t val, bool fwd) = 0;
    };

    struct Blinker: Ticking {
        BasePin *pin;

        Blinker(BasePin *p): pin{p}, half_period_ticks{ms_to_ticks(500)} {}

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

    struct BaseServo {
        uint16_t center_us = 1500;
        int16_t half_range = 500;
        int8_t val = 127;
        bool inverted = false;
        static constexpr uint8_t in_halfrange = 127;
        virtual void set_us(uint16_t val) = 0;
        void set(int8_t val) {
            this->val = val;
            if(inverted) val = -val;
            set_us(center_us + rdiv(val*half_range, in_halfrange));
        };
        void set_center_us(uint16_t v) {
            center_us = v;
            set(this->val);
        }
    };

    struct Wakeable {
        virtual void wake() {};
        virtual void sleep() {};
    };

    struct Fn: Wakeable {
        virtual void set(uint8_t val) = 0;
    };

    constexpr int8_t to_centered(const uint8_t v, const uint8_t deadzone = 0) {
        constexpr uint8_t center = 128;
        constexpr uint8_t in_range = 128;
        const uint8_t out_range = in_range - deadzone;
        int8_t ret = v - center;
        if(deadzone != 0) {
            uint8_t mag = abs(ret);
            if(mag < deadzone) return 0;
            else return (ret>0?1:-1) * rdiv((mag-deadzone)*in_range, out_range);
        } else {
            return ret;
        }
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
    class DelayedOffPin: public BasePin, public DelayedOff {
        BasePin &pin;
    public:
        DelayedOffPin(BasePin &pin, size_t delay_ms): DelayedOff{delay_ms}, pin{pin} {}
        void set(bool val) { DelayedOff::set(val); if(val) { pin.set(true); } }
        void force_off() { DelayedOff::force_off(); pin.set(false);}
        void tick() {
            bool was_on = get();
            DelayedOff::tick();
            if(was_on && get()==false) { pin.set(false); }
        }
    };

    template<typename int_t = uint8_t>
    struct SmoothValue: Ticking {
        int_t curr, target;
        int_t rate;
        SmoothValue(int_t rate = 1, int_t value = 0): curr{value}, target{value}, rate{rate} {}
        void reset() { curr=0; target=0; }
        void tick() override {
            int err = curr - target;
            if(err!=0) {
                if(abs(err) > rate) { curr -= (err>0?1:-1)*rate; } else curr = target;
            }
        }
    };

    struct Driving: Fn, Ticking {
        BaseHbridge *hbridge;
        DelayedOffPin reverse_lights;
        DelayedOffPin brake_lights;
        int8_t deadzone = 4;
        int8_t current_input;
        uint8_t has_been_idle = 0; ///< used to check if we can go in reverse

        SmoothValue<uint8_t> output;
        bool current_fwd;

        Driving(BaseHbridge *hbridge, BasePin *rev_lights, BasePin *brake_lights):
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
            current_input = to_centered(val, deadzone);
            //logf("current_input = %d\n", current_input);
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

            if(inp > 0) {
                // drive
                output.target = inp * 2; // 128 -> 256
                output.rate = 15;
            } else
            if(inp == 0) {
                // slowly stop
                output.target = 0;
                output.rate = 5;
            } else {
                // decelerate
                output.target = 0;
                output.rate = 1 + abs(inp)/5;
                // brake light on
                brake_lights.set(true);
            }

            output.tick();
            // if(current_input!=0 || output.curr!=0 || output.target!=0) {
            //     logf("in=%d;idle=%d ",
            //         current_input, has_been_idle);

            //     logf("%c %d -> %d\n",
            //         current_fwd ? '+' : '-',
            //         output.curr, output.target);
            // }

            hbridge->set(output.curr, current_fwd);

            if(!current_fwd && output.curr>0) reverse_lights.set(true);

            brake_lights.tick();
            reverse_lights.tick();
        }

    };

    struct Steering: Fn, Ticking {
        fn::BaseServo *servo;
        fn::Blinker *left;
        fn::Blinker *right;
        uint8_t deadzone = 4;
        uint8_t light_on_limit = 20;
        static constexpr size_t BlinkerPeriod = 1000;

        Steering(BaseServo *s, Blinker *l, Blinker *r):
            servo{s}, left{l}, right{r}, value{10}, blinker_delay{100}
        {}

        void sleep() override {
            left->set(false);
            right->set(false);
        }
        void wake() override {
            left->set_period(BlinkerPeriod);
            right->set_period(BlinkerPeriod);
            blinker_delay.force_off();
        }

        void set(uint8_t val) override {
            int8_t centered = to_centered(val, deadzone);
            value.target = centered;
        }

        void tick() override {
            value.tick();
            servo->set(value.curr);
            if (value.target > light_on_limit) {
                left->set(true); right->set(false);
                blinker_delay.set();
            } else
            if (value.target < -light_on_limit) {
                right->set(true); left->set(false);
                blinker_delay.set();
            }

            blinker_delay.tick();
            if(blinker_delay.get() == false) { left->set(false); right->set(false); }
            left->tick();
            right->tick();
        }
    private:
        SmoothValue<int8_t> value;
        DelayedOff blinker_delay;
    };

    struct Simple: Fn {
        BasePin *pin;

        Simple(BasePin *p): pin{p} {}

        void set(uint8_t val) override {
            pin->set(val > 127);
        }

        void sleep() override {
            pin->set(false);
        }

    };

    /**
     * Allows to output several "pins" to one analog pin.
     *
     * Setting each input pin will output a predefined analog value to the output.
     * This can be used to output both marker light (low power)
     *  and brake light (high power) to rear red lamps.
     */
    template<size_t N_INPUTS = 2>
    class MultiInputPin {
        using ThisClass = MultiInputPin<N_INPUTS>;
        BaseAnalogPin *output;

        class _Pin: public BasePin {
            ThisClass &parent;
            uint8_t pwm;
            bool on;
            friend class MultiInputPin;
        public:
            _Pin(ThisClass &parent, uint8_t pwm)
                : parent{parent}, pwm{pwm}
            {}

            void set(bool val) override {
                this->on = val;
                //logf("set(%d), pwm=%d\n", val, pwm);
                parent.refresh();
            };
        };
        etl::vector<_Pin, N_INPUTS> pins;
        friend class _Pin;
        void refresh() {
            uint8_t max_pwm = 0;
            for(auto &p: pins) {
                if(p.on && p.pwm > max_pwm) max_pwm = p.pwm;
                output->set_pwm(max_pwm);
            }
        }
    public:
        MultiInputPin(BaseAnalogPin *output): output{output} {}

        BasePin* create_pin(uint8_t pwm) {
            if(pins.available()==0) return nullptr;
            return &pins.emplace_back(*this, pwm);
        }

    };

    /** A virtual pin that controls several physical pins at once. */
    template <size_t N_OUTPUTS=2>
    class MultiOutputPin: public BasePin {
        etl::vector<BasePin*, N_OUTPUTS> outputs;
    public:
        MultiOutputPin(std::initializer_list<BasePin*> outputs)
            : outputs{outputs}
        {
        }
        void add_output(BasePin *pin) {outputs.push_back(pin);}
        void set(bool val) override {
            for(auto &pin: outputs) pin->set(val);
        }
    };

};
