#ifndef BATTERY_H_
#define BATTERY_H_

#include <cstdint>

#include <nrf_saadc.h>


uint32_t analogRead(const nrf_saadc_input_t ch) {
    nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_10BIT);
    nrf_saadc_enable();
    for(size_t i=0; i<NRF_SAADC_CHANNEL_COUNT; i++) {
        // need to disable scan mode
        nrf_saadc_channel_pos_input_set(i, NRF_SAADC_INPUT_DISABLED);
    }
    nrf_saadc_channel_config_t cfg {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain = NRF_SAADC_GAIN1_5,
        .reference = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time = NRF_SAADC_ACQTIME_40US, // >=10us recommended for 100k resistance
        .mode = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst = NRF_SAADC_BURST_DISABLED,
        .pin_p = ch,
        .pin_n = ch,
    };
    nrf_saadc_channel_init(0, &cfg);

    volatile nrf_saadc_value_t value = 0;
    nrf_saadc_buffer_init((nrf_saadc_value_t*)&value, 1);

    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

    while(!nrf_saadc_event_check(NRF_SAADC_EVENT_END)) {}
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_disable();
    if(value<0) value=0;
    return value;
}



#endif // BATTERY_H_