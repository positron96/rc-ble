#pragma once

#include "log.h"

#include <nordic/nrfx/hal/nrf_timer.h>
#include <nordic/nrfx/hal/nrf_gpio.h>

#include <cstdint>

constexpr size_t SERVO_MIN = 500;
constexpr size_t SERVO_MAX = 2500;

NRF_TIMER_Type *servo_timer = NRF_TIMER0;
constexpr IRQn_Type servo_irq = TIMER0_IRQn;

constexpr size_t CHANNELS = 1;
uint16_t sequence[CHANNELS+1] = {0, 20000};
uint8_t pins[CHANNELS] = {5};
volatile size_t idx = 0;

void timer_init() {

//NVIC_SetVector(servo_irq, (uint32_t)ISR_LIST[timer_id]);

    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_CLEAR);
    nrf_timer_bit_width_set(servo_timer, NRF_TIMER_BIT_WIDTH_16);
    nrf_timer_mode_set(servo_timer, NRF_TIMER_MODE_TIMER);
    //nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_1MHz); // 1us resolution
    nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_125kHz); // debug

    nrf_timer_int_enable(servo_timer, TIMER_INTENSET_COMPARE0_Msk);
    nrf_timer_shorts_enable(servo_timer, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

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
    nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL0, 1);
    NVIC_EnableIRQ(servo_irq);
	nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
}

void servo_stop() {
    NVIC_DisableIRQ(servo_irq);
    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
}

void servo_set(size_t pin, size_t us) {
    logf("servo_set %d\n", us);
    sequence[0] = us;
    sequence[1] = 20000 - us;
}


extern "C" void TIMER0_IRQHandler() {

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
	}

}
