#include <Arduino.h>

#include <etl/vector.h>

#include "functions.h"
#include "outputs_nrf.h"

constexpr size_t LED_PIN = 18;

volatile uint8_t val;

etl::vector<fn::Fn*, 10> functions;

constexpr size_t D1 = 14;
constexpr size_t D2 = 16;
constexpr size_t D3 = 20;
constexpr size_t D4 = 5;
constexpr size_t D5 = 6;
constexpr size_t D6 = 9;
constexpr size_t D7 = 10;
constexpr size_t M1 = 25;
constexpr size_t M2 = 28;

nrf::HBridge hbridge{M1, M2};
nrf::Servo steer_servo{D6};
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


void setup() {
    Serial.setPins(30, 28);
    Serial.begin(9600);
    Serial.println("\nStarting NimBLE Server");

    nrf::PWM pwm;
    pwm.setHBridge(hbridge);

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
        steer_servo.set(val);

        //last_t = millis();
        //servo_start();
    }

  /** Do your thing here, this just spams notifications to all connected clients */
    // if(pServer->getConnectedCount()) {
    //     NimBLEService* pSvc = pServer->getServiceByUUID("BAAD");
    //     if(pSvc) {
    //         NimBLECharacteristic* pChr = pSvc->getCharacteristic("F00D");
    //         if(pChr) {
    //             pChr->notify(true);
    //         }
    //     }
    // }

    delay(fn::Ticking::PERIOD_MS);
}
