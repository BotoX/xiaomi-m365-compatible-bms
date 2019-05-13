#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "bq769x0.h"

/* Software I2C */
#define I2C_TIMEOUT 100
#define SDA_PORT PORTD
#define SDA_PIN 6
#define SCL_PORT PORTD
#define SCL_PIN 7
/* Software I2C */

#define BMS_ALERT_PIN 17 // 17 = ALERT, PC3, PCINT11
#define BMS_BOOT_PIN 14 // 14 = BOOT, PC0
#define BMS_I2C_FET_PIN 8 // 8 = I2C Pull-Up FET Gate, PB0
#define BMS_K1_PIN 13 // 13 = PB5 = K1 connector
#define BMS_VDD_EN_PIN 15 // 15 = PC1 = VDD Enable

#define M365BMS_RADDR 0x22
#define M365BMS_WADDR 0x25

struct BMSSettings
{
    uint8_t header[2] = {0xB0, 0x0B};
    uint16_t version = 1;
    char serial[14] = "BOTOX001";
    uint32_t capacity = 7800; // mAh
    uint16_t nominal_voltage = 3600; // mV
    uint16_t full_voltage = 4150; // mV
    uint16_t num_cycles = 0;
    uint16_t num_charged = 0;
    uint16_t date = (18 << 9) | (10 << 5) | 1; // MSB (7 bits year, 4 bits month, 5 bits day) LSB

    // setShuntResistorValue
    uint16_t shuntResistor_uOhm = 1000;

    // setThermistorBetaValue
    uint16_t thermistor_BetaK = 3435;

    // setTemperatureLimits
    int16_t temp_minDischargeC = -20; // °C
    int16_t temp_maxDischargeC = 60; // °C
    int16_t temp_minChargeC = 0; // °C
    int16_t temp_maxChargeC = 45; // °C

    // setShortCircuitProtection
    uint32_t SCD_current = 80000; // mA
    uint16_t SCD_delay = 200; // us

    // setOvercurrentChargeProtection
    uint32_t OCD_current = 6000; // mA
    uint16_t OCD_delay = 3000; // ms

    // setOvercurrentDischargeProtection
    uint32_t ODP_current = 35000; // mA
    uint16_t ODP_delay = 1280; // ms

    // setCellUndervoltageProtection
    uint16_t UVP_voltage = 2800; // mV
    uint16_t UVP_delay = 2; // s

    // setCellOvervoltageProtection
    uint16_t OVP_voltage = 4200; // mV
    uint16_t OVP_delay = 2; // s

    // setBalancingThresholds
    uint16_t balance_minIdleTime = 1800; // s
    uint16_t balance_minVoltage = 3600; // mV
    uint16_t balance_maxVoltageDiff = 10; // mV

    // setIdleCurrentThreshold
    uint16_t idle_currentThres = 500; // mA

    // enableAutoBalancing
    uint16_t balance_enabled = 1;

    // adjADCPackOffset
    int16_t adcPackOffset = 0;

    // adjADCCellsOffset
    int16_t adcCellsOffset[15] = {0};

} __attribute__((packed));

/*** DON'T CHANGE ANYTHING BELOW THIS LINE ***/
/*** DON'T CHANGE ANYTHING BELOW THIS LINE ***/
/*** DON'T CHANGE ANYTHING BELOW THIS LINE ***/

struct M365BMS
{ // little endian
/*00-1F*/   uint16_t unk1[16] = {0x5A, 0x5A, 0x00};
/*20-2D*/   char serial[14] = "";
/*2E-2F*/   uint16_t version = 0x900; // 0x115 = 1.1.5
/*30-31*/   uint16_t design_capacity = 0; // mAh
/*32-33*/   uint16_t real_capacity = 0; // mAh
/*34-35*/   uint16_t nominal_voltage = 0; // mV
/*36-37*/   uint16_t num_cycles = 0;
/*38-39*/   uint16_t num_charged = 0;
/*3A-3B*/   uint16_t max_voltage = 0; // V/100
/*3C-3D*/   uint16_t max_discharge_current = 0; // A/100
/*3E-3F*/   uint16_t max_charge_current = 0; // A/100
/*40-41*/   uint16_t date = 0; // MSB (7 bits year, 4 bits month, 5 bits day) LSB
/*42-47*/   uint8_t errors[6] = {0};
/*48-5F*/   uint16_t unk3[12] = {0};
/*60-61*/   uint16_t status = 1; // b0 = config valid, b6 = charging, b9 = overvoltage, b10 = overheat
/*62-63*/   uint16_t capacity_left = 0; // mAh
/*64-65*/   uint16_t percent_left = 0;
/*66-67*/   int16_t current = 0; // A/100
/*68-69*/   uint16_t voltage = 0; // V/100
/*6A-6B*/   uint8_t temperature[2] = {0, 0}; // °C - 20
/*6C-6D*/   uint16_t balance_bits = 0;
/*6E-75*/   uint16_t unk5[4] = {0};
/*76-77*/   uint16_t health = 100; // %, <60% = battery bad
/*78-7F*/   uint16_t unk6[4] = {0};
/*80-9D*/   uint16_t cell_voltages[15] = {0}; // mV
/*9E-A1*/   uint16_t unk7[2] = {0};
#if 0
/*A2-A3*/   uint16_t unk8 = 1; // 1 ?
/*A4-DF*/   uint16_t unk9[30] = {0};
/*E0-E0*/   uint8_t unk10 = 0x3F; // BMS specific value ?
/*E1-E1*/   uint8_t unk11 = 0; // 0 ?
/*E2-E2*/   uint8_t unk12 = 0x3C; // BMS specific value ?
/*E3-E4*/   uint16_t unk13 = 1; // 1 ?
/*E5-EB*/   char unk_serial[7] = "G55179"; // BMS specific value ?
/*EC-FF*/   uint8_t unk14[19];
#endif
} __attribute__((packed));

struct NinebotMessage
{
    uint8_t header[2]; // 0x55, 0xAA

    uint8_t length; // length of data + 2
    uint8_t addr; // receiver address
    uint8_t mode; // read = 1 / write = 3
    uint8_t offset; // data offset/index in array
    uint8_t data[253]; // write = data, read = uint8_t read length

    uint16_t checksum; // (bytes without header) XOR 0xFFFF
};

void alertISR();

void loadSettings();
void saveSettings();
void applySettings();

void onNinebotMessage(NinebotMessage &msg);
void ninebotSend(NinebotMessage &msg);
void ninebotRecv();

void debug_print();

extern bool g_Debug;
extern bq769x0 g_BMS;
extern volatile bool g_interruptFlag;
extern unsigned long g_lastActivity;
extern unsigned long g_lastUpdate;

#endif // MAIN_H
