#pragma once

#include "log.h"

#include <nordic/nrfx/hal/nrf_timer.h>
#include <nordic/nrfx/hal/nrf_gpio.h>

#include <cstdint>

constexpr size_t SERVO_MIN = 500;
constexpr size_t SERVO_MAX = 2500;

NRF_TIMER_Type *servo_timer = NRF_TIMER1;
constexpr IRQn_Type servo_irq = TIMER1_IRQn;
#define servo_isr  TIMER1_IRQHandler

constexpr size_t CHANNELS = 1;
uint16_t sequence[CHANNELS+1] = {0, 20000};
uint32_t pins[CHANNELS] = {5};
size_t idx = 0;

void timer_init() {

    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_CLEAR);
    nrf_timer_bit_width_set(servo_timer, NRF_TIMER_BIT_WIDTH_16);
    nrf_timer_mode_set(servo_timer, NRF_TIMER_MODE_TIMER);
    nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_1MHz); // 1us resolution
    //nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_125kHz); // debug

    nrf_timer_int_enable(servo_timer, NRF_TIMER_INT_COMPARE0_MASK);
    nrf_timer_shorts_enable(servo_timer, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
    //nrf_timer_shorts_enable(servo_timer, NRF_TIMER_SHORT_COMPARE0_STOP_MASK);

    //NVIC_SetPriority(servo_irq, 5);

		// nrf_timer->TASKS_STOP  = 1;  // Stop counter, just in case already running
		// nrf_timer->TASKS_CLEAR = 1;  // counter to zero

		// nrf_timer->BITMODE   = 3UL;       // 32 bit
		// nrf_timer->MODE      = 0UL;       // timer, not counter
		// nrf_timer->PRESCALER = 4UL;       // freq = 16Mhz / 2^prescaler = 1Mhz
		// nrf_timer->CC[0]     = period_us; // Counter is compared to this
		// nrf_timer->INTENSET  = 1UL << TIMER_INTENSET_COMPARE0_Pos;     // interrupt on compare event.
		// nrf_timer->SHORTS    = 1UL << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // clear counter on compare event.

		// It's also possible to add a SHORT to STOP counter upon compare.
		// I leave counter running to keep period constant. The downside is that
		// the ISR must complete before period expires, or it will lock up.

    nrf_gpio_cfg_output(pins[0]);
}

void servo_start() {
    idx = CHANNELS;
    nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL0, 1000);
    NVIC_EnableIRQ(servo_irq);
	nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
}

void servo_stop() {
    NVIC_DisableIRQ(servo_irq);
    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);

    for(const auto &pin: pins) {
        nrf_gpio_pin_clear(pin);
    }
}

void servo_set(uint16_t us) {
    if (sequence[0] == us) return;
    logf("servo_set %d\n", us);
    sequence[0] = us;
    sequence[1] = 20000 - us;
    //nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL0, sequence[idx]);
}


extern "C" void servo_isr() {

    // make sure this is a compare event
    if (nrf_timer_event_check(servo_timer, NRF_TIMER_EVENT_COMPARE0)) {
        // clear event flag
        nrf_timer_event_clear(servo_timer, NRF_TIMER_EVENT_COMPARE0);

        // Counter is cleared automatically via SHORTS
        if(idx<CHANNELS) nrf_gpio_pin_clear(pins[idx]);
        idx += 1;
        if(idx>CHANNELS) idx = 0;
        if(idx<CHANNELS) nrf_gpio_pin_set(pins[idx]);
        nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL0, sequence[idx]);

        //nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
	}

}
