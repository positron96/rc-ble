//
// Copyright (c) 2015-2016 Intel Corporation
//
// SPDX-License-Identifier: Apache-2.0
//

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include <devicetree.h>
#include <nrfx_pwm.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   50

#define PWM_COUNTERTOP  100

static nrfx_pwm_t my_pwm = NRFX_PWM_INSTANCE(0);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// Set Advertisement data. Based on the Eddystone specification:
// https://github.com/google/eddystone/blob/master/protocol-specification.md
// https://github.com/google/eddystone/tree/master/eddystone-url

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
    BT_DATA_BYTES(BT_DATA_SVC_DATA16,
                  0xaa, 0xfe,
                  0x10, // Eddystone-URL frame type
                  0x00, // Calibrated Tx power at 0m
                  0x00, // URL Scheme Prefix http://www.
                  'z', 'e', 'p', 'h', 'y', 'r',
                  'p', 'r', 'o', 'j', 'e', 'c', 't',
                  0x08) // .org
};

// Set Scan Response data
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void bt_ready(int err) {
  if (err)  {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  printk("Bluetooth initialized\n");

  // Start advertising
  err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
                        sd, ARRAY_SIZE(sd));
  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }

  printk("Beacon started\n");
}


static int pwm_init(void) {
	static nrfx_pwm_config_t const config0 =  {
        .output_pins = {
            30,						// channel 0
            31,						// channel 1
            NRFX_PWM_PIN_NOT_USED,	// channel 2
            NRFX_PWM_PIN_NOT_USED	// channel 3
        },
        .irq_priority = 5,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = PWM_COUNTERTOP,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    return (nrfx_pwm_init(&my_pwm, &config0, NULL, NULL) == NRFX_SUCCESS) ? 0 : -1;

	// If PWM callbacks are to be used, remember to configure the interrupts correctly
	//IRQ_DIRECT_CONNECT(PWM1_IRQn, 0, nrfx_pwm_1_irq_handler, 0);
	//irq_enable(PWM1_IRQn);
}


void main(void) {
  int err;

  printk("Starting Beacon Demo\n");

  if(pwm_init() != 0) {
		printk("Error initializing PWM\n");
		return;
	}

  // Initialize the Bluetooth Subsystem
  err = bt_enable(bt_ready);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
  }
}