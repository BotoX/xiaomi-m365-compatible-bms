/* Battery management system based on bq769x0 for ARM mbed
 * Copyright (c) 2015-2018 Martin Jäger (www.libre.solar)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BQ769X0_H
#define BQ769X0_H

#include <Arduino.h>
#include "bq769x0_registers.h"

#define MAX_NUMBER_OF_CELLS 15
#define MAX_NUMBER_OF_THERMISTORS 3
#define NUM_OCV_POINTS 21

// IC type/size
#define bq76920 1
#define bq76930 2
#define bq76940 3

// output information to serial console for debugging
#define BQ769X0_DEBUG 1

enum {
    ERROR_XREADY = 0,
    ERROR_ALERT = 1,
    ERROR_UVP = 2,
    ERROR_OVP = 3,
    ERROR_SCD = 4,
    ERROR_OCD = 5,
    ERROR_USER_SWITCH = 6,
    ERROR_USER_DISCHG_TEMP = 7,
    ERROR_USER_CHG_TEMP = 8,
    ERROR_USER_CHG_OCD = 9,
    NUM_ERRORS
};

class bq769x0 {

public:
    // initialization, status update and shutdown
    bq769x0(uint8_t bqType = bq76940, uint8_t bqI2CAddress = 0x08, bool crc = true);
    void debug(bool debug);
    void boot(uint8_t bootPin);
    uint8_t begin(uint8_t bootPin = 0xFF);
    uint8_t checkStatus();  // returns 0 if everything is OK
    void checkUser();
    void clearErrors();
    uint8_t update(void);  // returns checkStatus retval
    void shutdown(void);

    // charging control
    bool enableCharging(uint16_t flag=(1 << ERROR_USER_SWITCH));
    void disableCharging(uint16_t flag=(1 << ERROR_USER_SWITCH));
    bool enableDischarging(uint16_t flag=(1 << ERROR_USER_SWITCH));
    void disableDischarging(uint16_t flag=(1 << ERROR_USER_SWITCH));

    // hardware settings
    void setShuntResistorValue(uint32_t res_uOhm = 1000);
    void setThermistors(uint8_t bitflag);
    void setThermistorBetaValue(uint16_t beta_K = 3435); // typical value for Semitec 103AT-5 thermistor

    void resetSOC(int percent = -1); // 0-100 %, -1 for automatic reset based on OCV
    void setBatteryCapacity(int32_t capacity_mAh, uint16_t nomVoltage_mV = 3600, uint16_t fullVoltage_mV = 4200);
    void setOCV(uint16_t voltageVsSOC[NUM_OCV_POINTS]);

    void adjADCPackOffset(int16_t offset);
    int16_t getADCPackOffset();
    void adjADCCellsOffset(int16_t offsets[MAX_NUMBER_OF_CELLS]);
    int16_t getADCCellOffset(uint8_t cell);

    uint8_t getNumberOfCells(void);
    uint8_t getNumberOfConnectedCells(void);

    // limit settings (for battery protection)
    void setTemperatureLimits(int16_t minDischarge_degC, int16_t maxDischarge_degC, int16_t minCharge_degC, int16_t maxCharge_degC);
    uint32_t setShortCircuitProtection(uint32_t current_mA, uint16_t delay_us = 70);
    uint32_t setOvercurrentChargeProtection(uint32_t current_mA, uint16_t delay_ms = 8);
    uint32_t setOvercurrentDischargeProtection(uint32_t current_mA, uint16_t delay_ms = 8);
    uint16_t setCellUndervoltageProtection(uint16_t voltage_mV, uint16_t delay_s = 1);
    uint16_t setCellOvervoltageProtection(uint16_t voltage_mV, uint16_t delay_s = 1);

    // balancing settings
    void setBalancingThresholds(uint16_t idleTime_s = 1800, uint16_t absVoltage_mV = 3400, uint8_t voltageDifference_mV = 20);
    void setIdleCurrentThreshold(uint32_t current_mA = 30);
    void setBalanceCharging(bool charging); // balance if charging, ignores idle time

    // automatic balancing when battery is within balancing thresholds
    void enableAutoBalancing(void);
    void disableAutoBalancing(void);

    // battery status
    int32_t getBatteryCurrent(bool raw = false);
    uint32_t getBatteryVoltage(bool raw = false);
    uint16_t getCellVoltage(uint8_t idCell, bool raw = false); // logical ids -> without gaps
    uint16_t getCellVoltage_(uint8_t i, bool raw = false); // physical ids -> possible gaps
    uint16_t getMinCellVoltage(void);
    uint16_t getMaxCellVoltage(void);
    uint16_t getAvgCellVoltage(void);
    float getTemperatureDegC(uint8_t channel = 1);
    float getTemperatureDegF(uint8_t channel = 1);
    int16_t getLowestTemperature(); // °C/10
    int16_t getHighestTemperature(); // °C/10
    float getSOC(void);
    uint16_t getBalancingStatus(void);

    // interrupt handling (not to be called manually!)
    void setAlertInterruptFlag(void);

    #if BQ769X0_DEBUG
    void printRegisters(void);
    #endif

    // public variables
    uint8_t batCycles_;
    uint8_t chargedTimes_;
    uint8_t errorCounter_[NUM_ERRORS];

private:

    // Variables
    bool debug_;
    uint8_t I2CAddress_;
    uint8_t type_;
    bool crcEnabled_;

    regSYS_STAT_t errorStatus_;
    uint32_t errorTimestamps_[NUM_ERRORS];

    bool chargingEnabled_;
    bool dischargingEnabled_;
    uint16_t chargingDisabled_;
    uint16_t dischargingDisabled_;

    uint32_t shuntResistorValue_uOhm_;
    uint16_t thermistorBetaValue_; // typical value for Semitec 103AT-5 thermistor: 3435
    uint16_t *OCV_; // Open Circuit Voltage of cell for SOC 100%, 95%, ..., 5%, 0%

    // indicates if a new current reading or an error is available from BMS IC
    volatile bool alertInterruptFlag_;

    uint8_t cellIdMap_[MAX_NUMBER_OF_CELLS]; // logical cell id -> physical cell id
    uint8_t numberOfCells_; // number of cells allowed by IC
    uint8_t connectedCells_; // actual number of cells connected
    uint16_t cellVoltages_[MAX_NUMBER_OF_CELLS]; // mV
    uint16_t cellVoltages_raw_[MAX_NUMBER_OF_CELLS]; // adc val
    uint8_t idCellMaxVoltage_;
    uint8_t idCellMinVoltage_;
    uint32_t batVoltage_; // mV
    uint16_t batVoltage_raw_; // adc val
    int32_t batCurrent_; // mA
    int16_t batCurrent_raw_; // adc val
    int16_t temperatures_[MAX_NUMBER_OF_THERMISTORS]; // °C/10
    uint8_t thermistors_;

    uint16_t nominalVoltage_; // mV, nominal voltage of single cell in battery pack
    uint16_t fullVoltage_; // mV, full voltage of single cell in battery pack
    int32_t nominalCapacity_; // mAs, nominal capacity of battery pack, max. 580 Ah possible @ 3.7V
    int32_t coulombCounter_; // mAs (= milli Coulombs) for current integration
    int32_t coulombCounter2_; // mAs (= milli Coulombs) for tracking battery cycles

    // Current limits (mA)
    int32_t maxChargeCurrent_;
    uint16_t maxChargeCurrent_delay_;
    uint32_t maxDischargeCurrent_;
    uint32_t idleCurrentThreshold_;

    // Temperature limits (°C/10)
    int16_t minCellTempCharge_;
    int16_t minCellTempDischarge_;
    int16_t maxCellTempCharge_;
    int16_t maxCellTempDischarge_;

    // Cell voltage limits (mV)
    uint16_t maxCellVoltage_;
    uint16_t minCellVoltage_;
    uint16_t balancingMinCellVoltage_mV_;
    uint8_t balancingMaxVoltageDifference_mV_;

    uint16_t adcGain_; // uV/LSB
    int16_t adcOffset_; // mV
    int16_t adcPackOffset_; // mV
    int16_t *adcCellsOffset_; // mV

    bool autoBalancingEnabled_;
    uint32_t balancingStatus_;     // holds on/off status of balancing switches
    uint16_t balancingMinIdleTime_s_;
    uint32_t idleTimestamp_;
    bool balanceCharging_;

    uint8_t charging_;
    uint32_t chargeTimestamp_;
    uint8_t fullVoltageCount_;

    uint32_t user_CHGOCD_TriggerTimestamp_;
    uint32_t user_CHGOCD_ReleaseTimestamp_;
    bool user_CHGOCD_ReleasedNow_;

    volatile uint32_t interruptTimestamp_;

    // Methods

    void updateVoltages(void);
    void updateCurrent(void);
    void updateTemperatures(void);

    void updateBalancingSwitches(void);

    uint8_t readRegister(uint8_t address);
    uint16_t readDoubleRegister(uint8_t address);
    void writeRegister(uint8_t address, uint8_t data);

};

#endif // BQ769X0_H
