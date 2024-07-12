#pragma once

#include "log.h"

#include <etl/array.h>

#include <nordic/nrfx/hal/nrf_pwm.h>

#include <cstdint>

constexpr size_t N_OUTPUTS = 7;
constexpr etl::array<size_t, N_OUTPUTS> HW_PINS{5, 6, 9, 10, 14, 16, 20};

constexpr etl::array<size_t, 2> MOTOR_PINS{25, 28};

NRF_PWM_Type *pwm = NRF_PWM0;
etl::array<uint16_t, PWM0_CH_NUM> pwm_data;
size_t MAX_PWM = 255;


void pwm_stop() {
    nrf_pwm_disable(pwm);
}


void pwm_init() {

    uint32_t pins[]{MOTOR_PINS[0], MOTOR_PINS[1], HW_PINS[0], HW_PINS[1]};
    nrf_pwm_pins_set(pwm, pins);
    nrf_pwm_enable(pwm);
    nrf_pwm_configure(pwm, NRF_PWM_CLK_1MHz, NRF_PWM_MODE_UP, MAX_PWM);

    nrf_pwm_decoder_set(pwm, NRF_PWM_LOAD_INDIVIDUAL, NRF_PWM_STEP_AUTO); //NRF_PWM_STEP_TRIGGERED
    nrf_pwm_seq_ptr_set(pwm, 0, pwm_data.data());
    nrf_pwm_seq_cnt_set(pwm, 0, pwm_data.size());
    nrf_pwm_seq_refresh_set(pwm, 0, 1);
    nrf_pwm_seq_end_delay_set(pwm, 0, 0);
    nrf_pwm_loop_set(pwm, 0);

    nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);
    //nrf_pwm_shorts_enable(pwm, PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk);
}

void set_motor(int16_t v) {
    if(v==0) { // coast
        pwm_data[0] = 0;
        pwm_data[1] = 0;
    } else
    if(v>0) {
        pwm_data[0] = 0;
        pwm_data[1] = v;
    } else {
        pwm_data[0] = v;
        pwm_data[1] = 0;
    }
}

bool pwm_set(size_t pin, uint8_t val) {
    if(pin>=2) return false;
    pwm_data[2 + pin] = val | 0x8000;
    nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);
    return true;
}
