#include <nrf_delay.h>

#include <app_timer.h>

#include <etl/vector.h>
#include <etl/string_view.h>
#include <etl/optional.h>
#include <etl/to_arithmetic.h>
#include <etl/string_utilities.h>


#include "functions.h"
#include "outputs_nrf.h"
#include "line_processor.h"

#include "simple_ble.h"
#include "bootloader.h"

#define delay nrf_delay_ms

volatile uint8_t val;

etl::vector<fn::Fn*, 10> functions;

constexpr size_t BAT_PIN = 30;
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
nrf::Pin pin_light_left{D5};
nrf::Pin pin_light_right{D6};
nrf::Pin pin_light_main{D1};
nrf::Pin pin_light_brake{D2};
nrf::Pin pin_light_reverse{D3};

fn::Blinker bl_left{&pin_light_left};
fn::Blinker bl_right{&pin_light_right};

fn::Driving driver{&hbridge, &pin_light_reverse, &pin_light_brake};
fn::Steering steering{&steer_servo, &bl_left, &bl_right};
fn::Simple main_light{&pin_light_main};

nrf::ServoTimer servo_timer;
nrf::PWM pwm;

extern "C" void TIMER1_IRQHandler() {
    servo_timer.isr();
};


uint32_t millis(void) {
    return app_timer_cnt_get() * 1000 / 32768;
}


void setup() {
    // Serial.setPins(15, 18);
    // Serial.begin(9600);
    // Serial.println("\nStarting NimBLE Server");

    pwm.add_hbridge(hbridge);

    functions.push_back(&driver);

    if(!servo_timer.add_servo(steer_servo)) {
        logln("servo add err");
        while(1){}
    }

    functions.push_back(&steering);

    functions.push_back(&main_light);

    // servo_timer.init();
    // pwm.init();

    ble_start();
}


void process_str(const char* buf, size_t len) {
    //logf("processing '%s'(%d)\n", buf, len);
    etl::string_view in{buf, len};
    if (in.compare("!")) {
        reboot_to_bootloader();
        return;
    }
    etl::optional<etl::string_view> token;
    token = etl::get_token(in, "=", token, true);
    if(!token) {
        logf("no ch: '%s'\n", buf);
        return;
    }
    auto data = etl::trim_view_whitespace(token.value());
    const auto ch_num = etl::to_arithmetic<int8_t>(data);
    if(!ch_num.has_value()) {
        logf("ch parsing failed: '%s'\n", buf);
        return;
    }

    token = etl::get_token(in, ", ", token, true);
    if(!token) {
        logf("no val: '%s'\n", buf);
        return;
    }
    data = etl::trim_view_whitespace(token.value()) ;
    const auto val = etl::to_arithmetic<uint8_t>(data);
    if(!val.has_value()) {
        logf("val parsing failed: '%s'\n", buf);
        return;
    }
    //logf("ch %d = %d\n", ch_num.value(), val.value());
    functions[ch_num.value()]->set(val.value());

}

void update_battery() {
    // static size_t last_time=0;
    // constexpr size_t INTERVAL_S = 60;
    // if(millis() - last_time > INTERVAL_S*1000) {
    //     last_time = millis();

    //     uint32_t v = analogRead(BAT_PIN);
    //     //logf("got ADC, %d, ", v);
    //     v = v * (27+68)/68 * 600 * 5 / 1024;  // 0.6V ref, 1/5 gain
    //     uint8_t char_data[3] = {0b0100'0000, uint8_t((v>>8) & 0xFF), uint8_t(v & 0xFF)};
    //     pBatEnergyChar->setValue(char_data, 3);
    //     //logf("%X\n", char_data);
    //     v = constrain(v, 3300, 4200);
    //     v = map(v, 3300, 4200, 0, 100);
    //     pBatChar->setValue<uint8_t>(v);
    // }
}


enum class State {
    Running, ///< there are connected clients
    RecentlyDisconnected, ///< client disconnected not long ago,
    Hibernation, ///< show almost no signs of life (except BLE)
};

line_processor::LineProcessor<> serial_rx(line_processor::callback_t::create<process_str>());

void loop() {
    //static uint32_t last_t;
    //int voltage = analogRead(30);

    // if(Serial.available()) {
    //     while(Serial.available()>0) {serial_rx.add(Serial.read());}
    // }

    static State state = State::RecentlyDisconnected;

    static size_t last_clients = 1; // do disconnection logic on boot
    static size_t disconnect_time = 0;
    static size_t ticks;

    size_t clients = get_connected_clients_count();

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
            delay(50);
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

    update_battery();

    delay(fn::Ticking::PERIOD_MS);
}

int main() {
    setup();
    while(1) { loop(); }
    return 0;
}