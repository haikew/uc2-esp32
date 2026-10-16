#pragma once
#include "Arduino.h"
#include "PinConfigDefault.h"
#undef PSXCONTROLLER

// Same firmware as UC2_canopen_slave_gpio, but for a classic ESP32 DevKit
// (ESP32-WROOM-32, CP2102, 38 pins). Used to test the collision sensor on a
// spare board. Only the pin numbers differ from the XIAO ESP32S3 build.

// ── Controller modules enabled on this slave ───────────────────────────────
#define DIGITAL_IN_CONTROLLER
#define DIGITAL_OUT_CONTROLLER
#define MESSAGE_CONTROLLER
#define CAN_BUS_ENABLED
#define CAN_CONTROLLER_CANOPEN
#define GPIO_CAN_SLAVE_CONTROLLER  // wires GpioCanSlave::setup/loop into main.cpp
#define I2C_BRIDGE_CONTROLLER      // generic raw-I2C passthrough (0x2350-0x2354)

struct UC2_canopen_slave_gpio_esp32dev : PinConfig
{
    /*
     ESP32 DevKit pin map:

       GPIO34  – Collision/resistance sensor ADC (ADC1_CH6, input only)
       GPIO25  – Remote-controllable output 1   (master writes x2301:01)
       GPIO26  – Remote-controllable output 2   (master writes x2301:02)
       GPIO27  – Digital input 1
       GPIO17  – CAN TX  (same as UC2_canopen_master)
       GPIO18  – CAN RX
       GPIO21  – I2C SDA
       GPIO22  – I2C SCL

     Notes:
       * GPIO9 (the XIAO's ADC pin) is SD2 here, wired to the SPI flash; do
         not use it. GPIO34 is the ADC1 equivalent.
       * Without a CAN transceiver the CAN side goes bus-off; the ADC read
         and the "GpioCanSlave loop:" debug line are unaffected.
       * Classic ESP32 ADC is less linear than the S3's (11 dB, top ~0.2 V
         compressed), so counts are not directly comparable to the XIAO.
    */

    const char *pindefName = "UC2_canopen_slave_gpio_esp32dev";
    const unsigned long BAUDRATE = 115200;

    // ── CAN bus ──────────────────────────────────────────────────────────
    int8_t CAN_TX = GPIO_NUM_17;
    int8_t CAN_RX = GPIO_NUM_18;
    uint32_t CAN_ID_CURRENT = CAN_ID_GPIO_0; // default node-id 60

    // ── Remote-controllable digital outputs ──────────────────────────────
    int8_t DIGITAL_OUT_1 = GPIO_NUM_25;
    int8_t DIGITAL_OUT_2 = GPIO_NUM_26;
    int8_t DIGITAL_OUT_3 = disabled;

    // ── Digital inputs ───────────────────────────────────────────────────
    int8_t DIGITAL_IN_1 = GPIO_NUM_27;
    int8_t DIGITAL_IN_2 = disabled;
    int8_t DIGITAL_IN_3 = disabled;

    int8_t GPIO_ESTOP_PIN = disabled;

    // ── Collision sensor (resistance → voltage divider on ADC) ───────────
    // Same detection settings as the XIAO build.
    int8_t   GPIO_COLLISION_ADC                 = GPIO_NUM_34; // ADC1_CH6
    uint16_t GPIO_COLLISION_THRESHOLD_DEFAULT   = 150;         // deviation band, ADC counts
    uint8_t  GPIO_COLLISION_SENSITIVITY_DEFAULT = 4;           // consecutive samples
    uint16_t GPIO_ADC_FILTER_ALPHA_X1024        = 128;         // EWMA alpha * 1024

    int8_t analogin_PIN_0 = GPIO_NUM_34;

    // ── Routing — everything stays LOCAL on this slave ───────────────────
    int8_t ROUTE_MOTOR[4]  = {2, 2, 2, 2};
    int8_t ROUTE_HOME[4]   = {2, 2, 2, 2};
    int8_t ROUTE_TMC[4]    = {2, 2, 2, 2};
    int8_t ROUTE_LASER[4]  = {2, 2, 2, 2};
    int8_t ROUTE_LASER_4   = 2;
    int8_t ROUTE_LED       = 2;
    int8_t ROUTE_GALVO     = 2;

    // ── I2C — generic sensor passthrough bus (I2cBridge) ─────────────────
    int8_t I2C_SDA = GPIO_NUM_21;
    int8_t I2C_SCL = GPIO_NUM_22;
};

const UC2_canopen_slave_gpio_esp32dev pinConfig;
