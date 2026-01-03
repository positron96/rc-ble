#include "nrf_outputs.hpp"
#include "nrf_outputs_uart.hpp"
#include "nrf_outputs_servo.hpp"
#include "line_processor.h"
#include "battery.h"
#include "ble_sd.hpp"
#include "bootloader.h"
#include "storage.hpp"
#include "uart.hpp"

#include <outputs.hpp>
#include <outputs_pdm.hpp>
#include <timed_utils.hpp>
#include <functions/base_functions.hpp>
#include <functions/car_functions.hpp>
#include <functions/blinkers.hpp>
#include <functions/light_fx.hpp>

#include <etl/vector.h>
#include <etl/string_view.h>
#include <etl/optional.h>
#include <etl/to_arithmetic.h>
#include <etl/string_utilities.h>
#include <etl/expected.h>

#include <nrf_delay.h>
#include <app_timer.h>

#include <algorithm>

#define delay nrf_delay_ms

volatile uint8_t val;

etl::vector<fn::Fn*, 10> functions;

#if defined(BLE_RC_V12)  // BLE-RC 1.2

// 1 marker
// 2 front
// 3 reverse
// 4 brake

// 5 left
// 6 right

// 7 steering servo
// 8 uart tx
// 9 extra

// 10, 11, motor

constexpr size_t BAT_PIN = 30;
constexpr nrf_saadc_input_t BAT_ADC_CH = NRF_SAADC_INPUT_AIN6;
constexpr size_t D0 = 0; // not on devboard
constexpr size_t D1 = 1;
constexpr size_t D2 = 4;
constexpr size_t D3 = 5;
constexpr size_t D4 = 6;
constexpr size_t D5 = 14;
constexpr size_t D6 = 16;
constexpr size_t D7 = 18;
constexpr size_t D8 = 20;
constexpr size_t DA = 9;
constexpr size_t DB = 10;
constexpr size_t M1 = 25;
constexpr size_t M2 = 28;
constexpr size_t PIN_AUX1       = D0;
constexpr size_t PIN_AUX2       = D1;
constexpr size_t PIN_AUX3       = D2;
constexpr size_t PIN_LEFT       = D3;
constexpr size_t PIN_RIGHT      = D4;
constexpr size_t PIN_HEADLIGHTS = D5;
constexpr size_t PIN_MARKERS    = D6;
constexpr size_t PIN_REV       = D7;
constexpr size_t PIN_BRAKES     = D8;
constexpr size_t PIN_SERVO      = DA;
constexpr size_t PIN_COMM       = DB;

#else // BLE_RC_V10 or devboard
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
constexpr size_t PIN_SERVO = D7;
#ifdef UART_PIN
  constexpr size_t PIN_COMM_TX = UART_PIN;
#else
  constexpr size_t PIN_COMM_TX = D4;
#endif
#endif

APP_TIMER_DEF(m_tick_timer);
APP_TIMER_DEF(m_battery_timer);

timed::TickList<10> ticklist;
// things like PDM and DitheredMotor need this:
// timed::DelegateList<5> fastticklist;

nrf::PWM::DualPwmMotor motor{M1, M2};
nrf::ServoTimer::Servo steer_servo{PIN_SERVO};
nrf::Pin pin_light_left_hw{PIN_LEFT};
nrf::Pin pin_light_right_hw{PIN_RIGHT};
nrf::UartAnalogPin pin_light_left_uart{0};
nrf::UartAnalogPin pin_light_right_uart{1};
outputs::MultiOutputPin pin_light_left{&pin_light_left_hw, &pin_light_left_uart};
outputs::MultiOutputPin pin_light_right{&pin_light_right_hw, &pin_light_right_uart};
// auto &pin_light_left = pin_light_left_hw;
// auto &pin_light_right = pin_light_right_hw;

nrf::Pin pin_light_main_hw{PIN_HEADLIGHTS};
// outputs::PdmPin pin_light_main{&pin_light_main_hw, &fastticklist};

nrf::PWM::Pin pin_light_rear_red{PIN_BRAKES};
nrf::Pin pin_light_rev_hw{PIN_REV};
nrf::UartAnalogPin pin_light_rev_uart{4};
nrf::UartAnalogPin pin_light_main_uart{3};

outputs::MultiInputPin pin_light_red{&pin_light_rear_red};

outputs::MultiOutputPin pin_light_main{&pin_light_main_hw, pin_light_red.create_pin(32), &pin_light_main_uart};
outputs::MultiOutputPin pin_light_rev{&pin_light_rev_hw, &pin_light_rev_uart};
// outputs::MultiOutputPin pin_light_main{&pin_light_main_hw, pin_light_red.create_pin(32)};
// outputs::MultiOutputPin pin_light_rev{&pin_light_rev_hw};

fn::Blinkers blinkers{&pin_light_left, &pin_light_right};
auto &bl_left = blinkers.fn_left();
auto &bl_right = blinkers.fn_right();
auto &fn_hazard = blinkers.fn_hazard();
//fn::Blinker bl_left{&pin_light_left};
//fn::Blinker bl_right{&pin_light_right};

nrf::UartAnalogPin pin_brake_uart{2};
outputs::MultiOutputPin pin_brake{pin_light_red.create_pin(255), &pin_brake_uart};
// outputs::MultiOutputPin pin_brake{pin_light_red.create_pin(255)};

fn::SimpleDriving fn_driver{&motor, &pin_light_rev, &pin_brake};
fn::Steering fn_steering{&steer_servo, &bl_left, &bl_right};

fn::PinFn fn_main_light{pin_light_main};

nrf::Pin pin_light_marker{PIN_MARKERS};
fn::PinFn fn_markers{pin_light_marker};

nrf::Pin pin_aux1{PIN_AUX1};
nrf::Pin pin_aux2{PIN_AUX2};
lightfx::DualLightController fn_light{&pin_aux1, &pin_aux2};


nrf::ServoTimer servo_timer;
nrf::PWM pwm;
nrf::UartOutputs<5> uart_pins;

//* To control blinkers with 2 bool functions, use this:
fn::FlipFlopFn fn_bl_l_ff{&bl_left};
fn::FlipFlopFn fn_bl_r_ff{&bl_right};
//* To control blinkers with one analog function, use this:
//fn::BinarySelector fn_blinker{&bl_left, &bl_right};


extern "C" void TIMER1_IRQHandler() {
    servo_timer.isr();
};

uint32_t millis(void) {
    return app_timer_cnt_get() * 1000 / 32768;
}

void set_tick_timer(bool running) {
    if(running) {
        app_timer_start(m_tick_timer, APP_TIMER_TICKS(fn::Tickable::TICK_PERIOD_MS), nullptr);
    } else {
        app_timer_stop(m_tick_timer);
    }
}


void setup() {
    uart::init(PIN_COMM_TX);
    uart::puts("uart init\n");
    log_init();
    logln("BLE-RC");
    app_timer_init();

    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    storage::init();

    pwm.add_hbridge(motor);
    pwm.add_pin(pin_light_rear_red);

    uart_pins.add_pin(pin_light_left_uart);
    uart_pins.add_pin(pin_light_right_uart);
    uart_pins.add_pin(pin_light_main_uart);
    uart_pins.add_pin(pin_brake_uart);
    uart_pins.add_pin(pin_light_rev_uart);
    ticklist.add(&uart_pins);

    steer_servo.inverted = true;
    if(!servo_timer.add_servo(steer_servo)) {
        logln("servo add err");
        while(1){}
    }

    fn_steering.auto_blinker_on = true;

    functions.push_back(&fn_driver);
    functions.push_back(&fn_steering);
    functions.push_back(&fn_main_light);
    functions.push_back(&fn_markers);
    //functions.push_back(&fn_blinker);
    functions.push_back(&fn_bl_r_ff);
    functions.push_back(&fn_bl_l_ff);
    functions.push_back(&fn_hazard);

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

/// @brief Process non-channel command
/// @param in - command string
void process_extra_cmd(const etl::string_view in) {
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
        motor.inverted = in.at(14) == '1';
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
}

void process_str(etl::string_view in) {
    //logf("processing '%.*s'\n", FMT_SV(in));
    if(in.starts_with("!")) {
        process_extra_cmd(in);
        return;
    }

    etl::optional<etl::string_view> token;
    token = etl::get_token(in, "=", token, true);
    if(!token || token.value().length()<1) {
        logf("no channel: '%.*s'\n", FMT_SV(in));
        return;
    }
    fn::Fn *fn = nullptr;
    switch(token.value().at(0)) {
        case 'D': fn = &fn_driver; break;
        case 'S': fn = &fn_steering; break;
        case 'H': fn = &fn_main_light; break;
        case 'E': fn = &fn_hazard; break;
        case 'L': fn = &fn_bl_l_ff; break;
        case 'R': fn = &fn_bl_r_ff; break;
        //case 'M': fn = &fn_marker; break;
        default: {
            const auto ch_opt = parse(token.value());
            if(ch_opt.has_value()) {
                const auto ch_num = ch_opt.value();
                if(ch_num < functions.size()) fn = functions[ch_num];
            }
        }
    }
    if(fn == nullptr) {
        logf("bad channel: '%.*s'\n", FMT_SV(in));
        return;
    }

    token = etl::get_token(in, ", ", token, true);
    if(!token) { logf("no value: '%.*s'\n", FMT_SV(in));  return; }
    const auto val_opt = parse<fn::val_t>(token.value());
    if(!val_opt) { logf("val parsing failed: '%.*s'\n", FMT_SV(in)); return; }

    auto val = val_opt.value();
    if(fn!=&fn_driver && fn!=&fn_steering)
        logf("ch %d = %d\n",
            std::distance(
                functions.begin(),
                std::find(functions.begin(), functions.end(), fn)),
            val);
    fn->set(fn::val_constrain(val));

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_battery(void * p_context) {
    uint32_t v = analogRead(BAT_ADC_CH);
    //logf("got ADC, %d\n", v);
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

    size_t clients = ble::get_connected_clients_count();

    if(clients == 0 && last_clients != 0) {
        for(auto &fn: functions) fn->sleep();
        disconnect_time = millis();
        blinkers.set_period(500);
        blinkers.fn_hazard().set(true);
        servo_timer.sleep();
        pwm.sleep();
        state = State::RecentlyDisconnected;
    } else
    if(clients != 0 && last_clients==0) {
        state = State::Running;
        servo_timer.wake();
        pwm.wake();
        blinkers.set_period(1000);
        blinkers.set_off();
        for(auto &fn: functions) fn->wake();
    }

    last_clients = clients;

    switch(state) {
    case State::Hibernation: {
        // do short flash every 10s
        static size_t last_flash = 0;
        if(millis() - last_flash > 10000) {
            blinkers.set_all(true);
            delay(10);
            blinkers.set_all(false);
            last_flash = millis();
        }
        break;
    }
    case State::RecentlyDisconnected:
        if(millis() - disconnect_time > 5000) {
            blinkers.set_off();
            state = State::Hibernation;
        }
        break;
    case State::Running:
        break;
    }

    ticklist.call_all();
}

// void app_error_handler(ret_code_t err, uint32_t line, const uint8_t * filename) {
//     logf("ERROR %d %s:%d\n", err, filename, line);
// }

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
    ticklist.evt_nonempty_cb = timed::list_nonempty_callback_t::create<set_tick_timer>();

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


extern "C" void _exit(int status) {
    __disable_irq();

    while (1) { }
}
