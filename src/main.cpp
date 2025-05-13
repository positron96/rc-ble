#include <algorithm>

#include <nrf_delay.h>
#include <app_timer.h>

#include <etl/vector.h>
#include <etl/string_view.h>
#include <etl/optional.h>
#include <etl/to_arithmetic.h>
#include <etl/string_utilities.h>
#include <etl/expected.h>


#include <outputs.hpp>
#include <functions/car_functions.hpp>
#include "nrf_outputs.hpp"
#include "nrf_outputs_pdm.hpp"
#include "nrf_outputs_uart.hpp"
#include "line_processor.h"
#include "battery.h"
#include "ble_sd.hpp"
#include "bootloader.h"
#include "storage.hpp"
#include "uart.hpp"

#define delay nrf_delay_ms

volatile uint8_t val;

etl::vector<fn::Fn*, 10> functions;

constexpr size_t BAT_PIN = 30;
constexpr nrf_saadc_input_t BAT_ADC_CH = NRF_SAADC_INPUT_AIN6;
constexpr size_t D1 = 5;
constexpr size_t D2 = 6;
constexpr size_t D3 = 9;
constexpr size_t D4 = 10; // not on devboard
constexpr size_t D5 = 14;
constexpr size_t D6 = 16;
constexpr size_t D7 = 20;
constexpr size_t M1 = 25;
constexpr size_t M2 = 28;

nrf::HBridge hbridge{M1, M2};
nrf::Servo steer_servo{D7};
nrf::Pin pin_light_left_hw{D5};
nrf::Pin pin_light_right_hw{D6};
// nrf::UartAnalogPin pin_light_left_uart{0};
// nrf::UartAnalogPin pin_light_right_uart{1};
// outputs::MultiOutputPin pin_light_left{&pin_light_left_hw, &pin_light_left_uart};
// outputs::MultiOutputPin pin_light_right{&pin_light_right_hw, &pin_light_right_uart};
auto &pin_light_left = pin_light_left_hw;
auto &pin_light_right = pin_light_right_hw;

nrf::Pin pin_light_main_hw{D1};
//nrf::PdmPin pin_light_main{D1};

nrf::PwmPin pin_light_rear_red{D2};
nrf::Pin pin_light_rev_hw{D3};
// nrf::UartAnalogPin pin_light_rev_uart{4};
// nrf::UartAnalogPin pin_light_main_uart{3};

outputs::MultiInputPin pin_light_red{&pin_light_rear_red};

// outputs::MultiOutputPin pin_light_main{&pin_light_main_hw, pin_light_red.create_pin(32), &pin_light_main_uart};
// outputs::MultiOutputPin pin_light_rev{&pin_light_rev_hw, &pin_light_rev_uart};
outputs::MultiOutputPin pin_light_main{&pin_light_main_hw, pin_light_red.create_pin(32)};
outputs::MultiOutputPin pin_light_rev{&pin_light_rev_hw};

fn::Blinker bl_left{&pin_light_left};
fn::Blinker bl_right{&pin_light_right};

// nrf::UartAnalogPin pin_brake_uart{2};
// outputs::MultiOutputPin pin_brake{pin_light_red.create_pin(255), &pin_brake_uart};
outputs::MultiOutputPin pin_brake{pin_light_red.create_pin(255)};

fn::SimpleDriving driver{&hbridge, &pin_light_rev, &pin_brake};
fn::Steering steering{&steer_servo, &bl_left, &bl_right};
fn::Simple main_light{&pin_light_main};
//fn::Simple marker_light{&pin_light_marker};

nrf::ServoTimer servo_timer;
nrf::PWM pwm;
// nrf::UartOutputs<5> uart_pins;

fn::BinarySelector fn_blinker{&bl_left, &bl_right};

extern "C" void TIMER1_IRQHandler() {
    servo_timer.isr();
};

uint32_t millis(void) {
    return app_timer_cnt_get() * 1000 / 32768;
}


void setup() {
    uart::init(UART_PIN);
    uart::puts("uart init\n");
    log_init();
    logs("\nBLE-RC\n");
    app_timer_init();

    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    storage::init();

    pwm.add_hbridge(hbridge);
    pwm.add_pin(pin_light_rear_red);

    // uart_pins.add_pin(pin_light_left_uart);
    // uart_pins.add_pin(pin_light_right_uart);
    // uart_pins.add_pin(pin_light_main_uart);
    // uart_pins.add_pin(pin_brake_uart);
    // uart_pins.add_pin(pin_light_rev_uart);

    functions.push_back(&driver);

    steer_servo.inverted = true;

    if(!servo_timer.add_servo(steer_servo)) {
        logln("servo add err");
        while(1){}
    }

    functions.push_back(&steering);
    functions.push_back(&main_light);
    //functions.push_back(&marker_light);
    functions.push_back(&fn_blinker);

    servo_timer.init();
    pwm.init();

    ble::start();
}

template<typename T = uint8_t>
etl::expected<T, etl::to_arithmetic_status> parse(const etl::string_view &str) {
    auto data = etl::trim_view_whitespace(str);
    const auto val = etl::to_arithmetic<T>(data);
    if(val.has_value()) return val.value();
    return etl::unexpected(val.error());
}

#define FMT_SV(sv)  (int)((sv).length()), (sv).data()

void process_str(etl::string_view in) {
    //logf("processing '%.*s'\n", FMT_SV(in));
    if(in.starts_with("!")) {
        if (in.compare("!dfu") == 0) {
            reboot_to_bootloader();
            return;
        } else
        if(in.starts_with("!trim_steer=")) {
            auto val = parse<uint16_t>(in.substr(12));
            if(val.has_value()) {
                logf("trim %d\n", val.value());
                steer_servo.set_center_us(val.value());
            } else {
                logf("parse error '%s': %s\n", in, val.error().c_str());
            }
        } else
        if(in.starts_with("!invert_drive=")) {
            hbridge.inverted = in.at(14) == '1';
        } else
        if(in.starts_with("!name=")) {
            const auto new_name = in.substr(6);
            logf("name:='%.*s'[%d]\n", FMT_SV(new_name), new_name.length());
            bool v = storage::set_dev_name(new_name);
            if(v) {
                ble::update_dev_name();
            } else {
                logf("set name failed\n");
            }

        } else {
            logf("Unknown cmd: '%.*s'\n", FMT_SV(in));
        }
        return;
    }
    etl::optional<etl::string_view> token;
    token = etl::get_token(in, "=", token, true);
    if(!token) {
        logf("no '=': '%.*s'\n", FMT_SV(in));
        return;
    }
    const auto ch_opt = parse(token.value());
    if(!ch_opt.has_value()) {
        logf("bad ch: '%.*s'\n", FMT_SV(in));
        return;
    }
    const auto ch_num = ch_opt.value();
    if(ch_num >= functions.size()) {
        logf("bad ch: %d\n", ch_num);
        return;
    }

    token = etl::get_token(in, ", ", token, true);
    if(!token) {
        logf("no val: '%.*s'\n", FMT_SV(in));
        return;
    }

    const auto val_opt = parse(token.value());
    if(!val_opt.has_value()) {
        logf("val parsing failed: '%.*s'\n", FMT_SV(in));
        return;
    }
    if(ch_num>1) logf("ch %d = %d\n", ch_num, val_opt.value());
    functions[ch_num]->set(val_opt.value());

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_battery(void * p_context) {
    uint32_t v = analogRead(BAT_ADC_CH);
    logf("got ADC, %d\n", v);
    v = v * (27+68)/68 * 600 * 5 / 1024;  // mV at VCC, 0.6V ref, 1/5 gain
    //v = v * 600 * 5 / 1024;  // in mV at ADC pin, 0.6V ref, 1/5 gain
    v = std::clamp(v, 3300ul, 4200ul);
    v = map(v, 3300, 4200, 0, 100);
    //v = map(v, 0, 3000, 0, 100);
    //v = v / 100; // 3000mV -> 30
    ble::set_bas(v);
}


enum class State {
    Running, ///< there are connected clients
    RecentlyDisconnected, ///< client disconnected not long ago,
    Hibernation, ///< show almost no signs of life (except BLE)
};

line_processor::LineProcessor<> serial_rx(line_processor::callback_t::create<process_str>());

void timer_tick(void * p_context) {

    static State state = State::RecentlyDisconnected;

    static size_t last_clients = 1; // do disconnection logic on boot
    static size_t disconnect_time = 0;
    static size_t ticks;

    size_t clients = ble::get_connected_clients_count();

    if(clients == 0 && last_clients != 0) {
        for(auto &fn: functions) fn->sleep();
        disconnect_time = millis();
        bl_left.set_period(500);
        bl_right.set_period(500);
        bl_left.restart();
        bl_right.restart();
        servo_timer.sleep();
        pwm.sleep();
        state = State::RecentlyDisconnected;
    } else
    if(clients != 0 && last_clients==0) {
        state = State::Running;
        servo_timer.wake();
        pwm.wake();
        for(auto &fn: functions) fn->wake();
    }

    last_clients = clients;

    switch(state) {
    case State::Hibernation: {
        static size_t last_flash = 0;
        if(millis() - last_flash > 10000) {
            bl_left.pin->set(true);
            bl_right.pin->set(true);
            delay(10);
            bl_left.pin->set(false);
            bl_right.pin->set(false);
            last_flash = millis();
        }
        break;
    }
    case State::RecentlyDisconnected:
        if(millis() - disconnect_time > 5000) {
            bl_left.set(false);
            bl_right.set(false);
            state = State::Hibernation;
        }
        break;
    case State::Running:
        driver.tick();
        steering.tick();
        break;
    }

    ticks++;
    bl_right.tick();
    bl_left.tick();
    // uart_pins.tick();
}

// void app_error_handler(ret_code_t err, uint32_t line, const uint8_t * filename) {
//     logf("ERROR %d %s:%d\n", err, filename, line);
// }

APP_TIMER_DEF(m_tick_timer);
APP_TIMER_DEF(m_battery_timer);
// APP_TIMER_DEF(m_pdm_timer);

// void update_pdm(void * ctx) {
//     pin_light_main.tick();
// }

int main() {
    setup();
    update_battery(nullptr);

    uint32_t err_code;
    err_code = app_timer_create(&m_tick_timer, APP_TIMER_MODE_REPEATED, timer_tick);
    APP_ERROR_CHECK(err_code);
    app_timer_start(m_tick_timer, APP_TIMER_TICKS(fn::Ticking::PERIOD_MS), nullptr);

    err_code = app_timer_create(&m_battery_timer, APP_TIMER_MODE_REPEATED, update_battery);
    APP_ERROR_CHECK(err_code);
    app_timer_start(m_battery_timer, APP_TIMER_TICKS(15000), nullptr);

    // err_code = app_timer_create(&m_pdm_timer, APP_TIMER_MODE_REPEATED, update_pdm);
    // APP_ERROR_CHECK(err_code);
    // app_timer_start(m_pdm_timer, APP_TIMER_TICKS(1), nullptr);

    while(1) {
        nrf_pwr_mgmt_run();
    }
    return 0;
}
