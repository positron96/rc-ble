# rc-ble

Firmware for BLE-based RC receiver for miniature RC vehicles.

The board is based on nRF52810 MCU.

More details about the project: https://positron96.gitlab.io/projects/rc-ble/

The project uses PlatformIO build system,
 which in turn uses custom platform and framework based on Nordic nRF5 SDK.
PlatformIO will download these components automatically,
 but here are the links to the projects anyway:
 https://github.com/positron96/platform-nordicnrf52,
 https://github.com/positron96/framework-nrf5sdk.

The firmware supports OTA updates.
For this the MCU needs to have a bootloader flashed,
 so there are 3 components in the flash: SoftDevice, Bootloader and this firmware.

Example "open_bootloader" from the SDK
 (version adapter for platformIO is in platform repository)
 was used.
Instructions for flashing will come later.

## Notes and links

Example of TIMER+PPI+GPIOTE PWM implementation

https://github.com/NordicPlayground/nrf52-timer-gpiote-ppi-hands-on/blob/master/main.c
