#include <Arduino.h>

#include <etl/vector.h>

#include "functions.h"
#include "pins.h"
#include "timers.h"
#include "outputs_nrf.h"

constexpr size_t LED_PIN = 18;

volatile uint8_t val;

etl::vector<fn::Fn*, 10> functions;

void setup() {
    Serial.setPins(30, 28);
    Serial.begin(115200);
    Serial.println("\nStarting NimBLE Server");    
    //pwm_init();

    nrf::PWM pwmgen;
    fn::Drive *drive = new fn::Drive();
    drive->h_bridge = pwmgen.createHBridge(10, 11);
    drive->reverse_lights = pwmgen.createPin(12);
    drive->brake_lights = pwmgen.createPin(13);

    functions.push_back(drive);
    

    timer_init();
    val = 127;
    servo_start();
    servo_set(val2us(val));

}



void loop() {
    //static uint32_t last_t;

    if(Serial.available()) {
        String s = Serial.readString();
        s.trim();
        val = s.toInt();
        Serial.println(String("Got: ")+val);

        //last_t = millis();
        //servo_start();
    }

    servo_set(val2us(val));

    //pwm_set(0, val);


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

  delay(100);
}
