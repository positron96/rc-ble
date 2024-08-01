#pragma once

#include <etl/vector.h>

#include <cstdint>
#include <cmath>

class Startable;
etl::vector<Startable*, 10> startables;

struct Startable {

    Startable() { startables.push_back(this); };
    virtual ~Startable() {
        auto it = std::find(startables.begin(), startables.end(), this);
        if(it != startables.end())
            startables.erase(it);
    }

    virtual void start() = 0;
    virtual void stop() = 0;
};

namespace outputs {

    struct Hbridge: Startable {
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

    struct Blinker {
        uint32_t period_ms;
        virtual void set(bool val) = 0;
        virtual void restart() = 0;
    };

    struct Servo: Startable {
        size_t center = 1500;
        size_t half_range = 500;
        virtual void set_us(uint16_t val) = 0;
        void set(int8_t val) {
            set_us(center + val*half_range/127);
        };
    };
}

constexpr size_t TICK_FREQ = 50;
constexpr size_t ms_to_ticks(const size_t ms) { return ms/TICK_FREQ;};

namespace fn {

    struct Fn {
        virtual void start() {};
        virtual void set(uint8_t val) = 0;
        virtual void stop() {};
        /** Executed at 100Hz rate. */
        virtual void tick() {}
    };

    int8_t to_centered(uint8_t v) {
        return v - 128;
    }

    struct Drive: Fn {
        int8_t value = 0;
        outputs::Hbridge *h_bridge;
        outputs::Pin *reverse_lights;
        outputs::Pin *brake_lights;
        size_t reverse_delay_ticks = ms_to_ticks(500);
        size_t brake_delay_ticks = ms_to_ticks(500);
        size_t reverse_ticks_left = 0;
        size_t brake_ticks_left = 0;
        int8_t nonzero_limit = 10;

        void set(uint8_t val) override {
            // TODO: make it act like an actual gas pedal (to zero --> coast, negative -> break)
            int8_t last = value;
            value = to_centered(val);
            h_bridge->set(value);
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
    };

    struct Steer: Fn {
        int8_t value;
        outputs::Servo *servo;
        outputs::Blinker *left;
        outputs::Blinker *right;
        uint8_t nonzero_limit = 20;
        size_t blinker_ticks_left = 0;
        size_t blinker_timeout_ticks = ms_to_ticks(100);

        void set(uint8_t val) override {
            value = to_centered(val);
            servo->set(value);
            if (value > nonzero_limit) {
                left->set(true);
            } else
            if (value < -nonzero_limit) {
                right->set(true);
            }
        }

        void reset_ticks() { blinker_ticks_left = blinker_timeout_ticks;}

        void tick() override {
            if(abs(value) > nonzero_limit) reset_ticks();
            if(blinker_ticks_left == 0) {
                right->set(false);
                left->set(false);
            } else {
                blinker_ticks_left--;
            }
        }
    };

    struct Pin: Fn {
        outputs::Pin *pin;
        
        void set(uint8_t val) override {
            pin->set(val > 127);
        }
    };

};
