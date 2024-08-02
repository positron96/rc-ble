#include <Arduino.h>

#include <etl/vector.h>

#include "functions.h"
#include "outputs_nrf.h"

constexpr size_t LED_PIN = 18;

volatile uint8_t val;

etl::vector<fn::Fn*, 10> functions;

constexpr size_t D1 = 5;
constexpr size_t D2 = 6;
constexpr size_t D3 = 9;
constexpr size_t D4 = 10;
constexpr size_t D5 = 14;
constexpr size_t D6 = 16;
constexpr size_t D7 = 20;
constexpr size_t M1 = 25;
constexpr size_t M2 = 28;

nrf::HBridge hbridge{M1, M2};
nrf::Servo steer_servo{D7};
nrf::Pin pin_light_left{D1};
nrf::Pin pin_light_right{D2};
nrf::Pin pin_light_reverse{D3};
nrf::Pin pin_light_brake{D4};
nrf::Pin pin_light_main{D5};

fn::Blinker bl_left{&pin_light_left};
fn::Blinker bl_right{&pin_light_right};

fn::Driving drive{&hbridge, &pin_light_reverse, &pin_light_brake};
fn::Steering steering{&steer_servo, &bl_left, &bl_right};
fn::Simple main_light{&pin_light_main};

nrf::ServoTimer servo_timer;

extern "C" void TIMER1_IRQHandler() {
    servo_timer.isr();
};


void setup() {
    Serial.setPins(30, 28);
    Serial.begin(9600);
    Serial.println("\nStarting NimBLE Server");

    nrf::PWM pwm;
    //pwm.add_hbridge(hbridge);

    functions.push_back(&drive);

    if(!servo_timer.add_servo(steer_servo)) {
        Serial.println("servo add err");
        while(1){}
    }

    functions.push_back(&steering);

    functions.push_back(&main_light);

    servo_timer.init();
}



void loop() {
    //static uint32_t last_t;

    if(Serial.available()) {
        String s = Serial.readString();
        s.trim();
        val = s.toInt();
        Serial.println(String("Got: ")+val);

        //functions[0]->set(val);
        //steer_servo.set(val);
        steering.set(val);
        //main_light.set(val);
        //bl_right.set(val>127);
        //bl_left.set(val>127);

        //last_t = millis();
        //servo_start();
    }

    delay(fn::Ticking::PERIOD_MS);
    bl_right.tick();
    bl_left.tick();

}
