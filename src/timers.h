#pragma once

#include "log.h"

#include <nordic/nrfx/hal/nrf_timer.h>
#include <nordic/nrfx/hal/nrf_gpio.h>
#include <nordic/nrfx/hal/nrf_gpiote.h>
#include <nordic/nrfx/hal/nrf_ppi.h>

#include <cstdint>

constexpr size_t SERVO_MIN = 500;
constexpr size_t SERVO_MAX = 2500;
constexpr size_t SERVO_CENTER = (SERVO_MIN + SERVO_MAX) / 2;
constexpr size_t SERVO_TOTAL = 20000;

NRF_TIMER_Type *servo_timer = NRF_TIMER1;
constexpr IRQn_Type servo_irq = TIMER1_IRQn;
#define servo_isr  TIMER1_IRQHandler

constexpr size_t CHANNELS = 1;
uint16_t sequence[CHANNELS+1] = {0, SERVO_TOTAL};
uint32_t pins[CHANNELS] = {5};
nrf_ppi_channel_t ppis[CHANNELS*2] = {NRF_PPI_CHANNEL6, NRF_PPI_CHANNEL7};
constexpr size_t TIMER_RESET_IDX = 3;

size_t idx = 0;

void timer_init() {

    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_CLEAR);
    nrf_timer_bit_width_set(servo_timer, NRF_TIMER_BIT_WIDTH_16);
    nrf_timer_mode_set(servo_timer, NRF_TIMER_MODE_TIMER);
    nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_1MHz); // 1us resolution
    //nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_125kHz); // debug

    //nrf_timer_int_enable(servo_timer, NRF_TIMER_INT_COMPARE3_MASK);

    nrf_timer_shorts_enable(servo_timer, NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK);

    nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL3, SERVO_TOTAL);

    //nrf_gpio_cfg_output(6);

    for(size_t i=0; i<CHANNELS; i++) {
        nrf_gpio_cfg_output(pins[i]);
        size_t gpiote_ch = i;

        nrf_gpiote_task_configure(NRF_GPIOTE, gpiote_ch,
            pins[i], NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW
        );
        nrf_gpiote_task_enable(NRF_GPIOTE, gpiote_ch);
        nrf_ppi_channel_endpoint_setup(
            NRF_PPI, ppis[i*2],
            //(uint32_t)&servo_timer->EVENTS_COMPARE[i],
            //(uint32_t)&NRF_GPIOTE->TASKS_CLR[gpiote_ch]
            nrf_timer_event_address_get(
                servo_timer, nrf_timer_compare_event_get(i)),
            nrf_gpiote_task_address_get(
                NRF_GPIOTE, nrf_gpiote_clr_task_get(gpiote_ch))
        );
        nrf_ppi_channel_enable(NRF_PPI, ppis[i*2]);

        nrf_ppi_channel_endpoint_setup(
            NRF_PPI, ppis[i*2+1],
            nrf_timer_event_address_get(servo_timer, NRF_TIMER_EVENT_COMPARE3),
            //(uint32_t)&NRF_GPIOTE->TASKS_SET[gpiote_ch]
            nrf_gpiote_task_address_get(
                NRF_GPIOTE, nrf_gpiote_set_task_get(gpiote_ch))
        );
        nrf_ppi_channel_enable(NRF_PPI, ppis[i*2+1]);
    }

}

void servo_start() {
    idx = CHANNELS;
    nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL0, 1);
    //NVIC_EnableIRQ(servo_irq);
	nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
}

void servo_stop() {
    //NVIC_DisableIRQ(servo_irq);
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
    nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL0, sequence[0]);
}


// extern "C" void servo_isr() {

//     nrf_gpio_pin_toggle(6);

//     // make sure this is a compare event
//     if (nrf_timer_event_check(servo_timer, NRF_TIMER_EVENT_COMPARE3)) {
//         // clear event flag
//         nrf_timer_event_clear(servo_timer, NRF_TIMER_EVENT_COMPARE3);



//         // Counter is cleared automatically via SHORTS
//         // if(idx<CHANNELS) nrf_gpio_pin_clear(pins[idx]);
//         // idx += 1;
//         // if(idx>CHANNELS) idx = 0;
//         // if(idx<CHANNELS) nrf_gpio_pin_set(pins[idx]);
//         // nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL0, sequence[idx]);

//         //nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
// 	}

// }
