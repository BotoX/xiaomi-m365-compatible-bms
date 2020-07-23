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

#include <Arduino.h>
#include <math.h>     // log for thermistor calculation
#include "main.h"
#include "bq769x0.h"

/* Software I2C */
#include <SoftWire.h>
SoftWire Wire = SoftWire();
/* Software I2C */

const char *byte2char(int x)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
{
    uint8_t data = inCrc ^ inData;

    for (uint8_t i = 0; i < 8; i++)
    {
        if ((data & 0x80) != 0)
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
    }
    return data;
}

//----------------------------------------------------------------------------

bq769x0::bq769x0(uint8_t bqType, uint8_t bqI2CAddress, bool crc)
{
    memset(this, 0, sizeof(*this));

    // set some safe default values
    setBatteryCapacity(0);
    setShuntResistorValue();
    setThermistorBetaValue();
    setBalancingThresholds();
    setIdleCurrentThreshold();

    alertInterruptFlag_ = true;   // init with true to check and clear errors at start-up

    type_ = bqType;
    if (type_ == bq76920) {
        numberOfCells_ = 5;
        setThermistors(0b001);
    } else if (type_ == bq76930) {
        numberOfCells_ = 10;
        setThermistors(0b011);
    } else {
        numberOfCells_ = 15;
        setThermistors(0b111);
    }

    // prevent errors if someone reduced MAX_NUMBER_OF_CELLS accidentally
    if (numberOfCells_ > MAX_NUMBER_OF_CELLS) {
        numberOfCells_ = MAX_NUMBER_OF_CELLS;
    }

    // initialize variables
    for (uint8_t i = 0; i < numberOfCells_; i++) {
        cellVoltages_[i] = 0;
    }

    crcEnabled_ = crc;
    I2CAddress_ = bqI2CAddress;
}

//-----------------------------------------------------------------------------

void bq769x0::debug(bool debug)
{
    debug_ = debug;
}

//-----------------------------------------------------------------------------

uint8_t bq769x0::begin(uint8_t bootPin)
{
    Wire.begin();
    pinMode(BMS_I2C_FET_PIN, OUTPUT);
    digitalWrite(BMS_I2C_FET_PIN, HIGH);

    while (true) {
        if (bootPin != 0xFF) {
            boot(bootPin);
        }

        // test communication
        writeRegister(CC_CFG, 0x19);       // should be set to 0x19 according to datasheet
        if (readRegister(CC_CFG) == 0x19)
            break;

        #if BQ769X0_DEBUG
        if(g_Debug)
            Serial.println(F("BMS communication error"));
        #endif
        delay(250);
    }

    // initial settings for bq769x0
    writeRegister(SYS_CTRL1, 0b00011000);  // switch external thermistor and ADC on
    writeRegister(SYS_CTRL2, 0b01000000);  // switch CC_EN on

    // reset balance registers just in case
    writeRegister(CELLBAL1, 0x0);
    writeRegister(CELLBAL2, 0x0);
    writeRegister(CELLBAL3, 0x0);

    // get ADC offset and gain
    adcOffset_ = readRegister(ADCOFFSET);
    adcGain_ = 365 + (((readRegister(ADCGAIN1) & 0b00001100) << 1) |
        ((readRegister(ADCGAIN2) & 0b11100000) >> 5)); // uV/LSB

    return 0;
}

//----------------------------------------------------------------------------
// Boot IC by pulling the boot pin TS1 high for some ms

void bq769x0::boot(uint8_t bootPin)
{
    pinMode(bootPin, OUTPUT);
    digitalWrite(bootPin, HIGH);
    delay(5); // wait 5 ms for device to receive boot signal (datasheet: max. 2 ms)
    pinMode(bootPin, INPUT); // don't disturb temperature measurement
    delay(10); // wait for device to boot up completely (datasheet: max. 10 ms)
}

//----------------------------------------------------------------------------
// Fast function to check whether BMS has an error
// (returns 0 if everything is OK)

uint8_t bq769x0::checkStatus()
{
    //  Serial.print("errorStatus: ");
    //  Serial.println(errorStatus);
    if (alertInterruptFlag_ || errorStatus_.regByte)
    {
        regSYS_STAT_t sys_stat;
        sys_stat.regByte = readRegister(SYS_STAT);

        // first check, if only a new CC reading is available
        if (sys_stat.bits.CC_READY == 1) {
            //Serial.println("Interrupt: CC ready");
            updateCurrent();  // automatically clears CC ready flag
        }

        // Serious error occured
        if (sys_stat.regByte & STAT_FLAGS)
        {
            if (!errorStatus_.bits.DEVICE_XREADY && sys_stat.bits.DEVICE_XREADY) { // XR error
                chargingEnabled_ = dischargingEnabled_ = false;
                chargingDisabled_ |= (1 << ERROR_XREADY);
                dischargingDisabled_ |= (1 << ERROR_XREADY);

                errorCounter_[ERROR_XREADY]++;
                errorTimestamps_[ERROR_XREADY] = millis();
                #if BQ769X0_DEBUG
                if(g_Debug)
                    Serial.println(F("bq769x0 ERROR: XREADY"));
                #endif
            }
            if (!errorStatus_.bits.OVRD_ALERT && sys_stat.bits.OVRD_ALERT) { // Alert error
                chargingEnabled_ = dischargingEnabled_ = false;
                chargingDisabled_ |= (1 << ERROR_ALERT);
                dischargingDisabled_ |= (1 << ERROR_ALERT);

                errorCounter_[ERROR_ALERT]++;
                errorTimestamps_[ERROR_ALERT] = millis();
                #if BQ769X0_DEBUG
                if(g_Debug)
                    Serial.println(F("bq769x0 ERROR: ALERT"));
                #endif
            }
            if (sys_stat.bits.UV) { // UV error
                dischargingEnabled_ = false;
                dischargingDisabled_ |= (1 << ERROR_UVP);

                errorCounter_[ERROR_UVP]++;
                errorTimestamps_[ERROR_UVP] = millis();
                #if BQ769X0_DEBUG
                if(g_Debug)
                    Serial.println(F("bq769x0 ERROR: UVP"));
                #endif
            }
            if (sys_stat.bits.OV) { // OV error
                chargingEnabled_ = false;
                chargingDisabled_ |= (1 << ERROR_OVP);

                errorCounter_[ERROR_OVP]++;
                errorTimestamps_[ERROR_OVP] = millis();
                #if BQ769X0_DEBUG
                if(g_Debug)
                    Serial.println(F("bq769x0 ERROR: OVP"));
                #endif
            }
            if (sys_stat.bits.SCD) { // SCD
                dischargingEnabled_ = false;
                dischargingDisabled_ |= (1 << ERROR_SCD);

                errorCounter_[ERROR_SCD]++;
                errorTimestamps_[ERROR_SCD] = millis();
                #if BQ769X0_DEBUG
                if(g_Debug)
                    Serial.println(F("bq769x0 ERROR: SCD"));
                #endif
            }
            if (sys_stat.bits.OCD) { // OCD
                dischargingEnabled_ = false;
                dischargingDisabled_ |= (1 << ERROR_OCD);

                errorCounter_[ERROR_OCD]++;
                errorTimestamps_[ERROR_OCD] = millis();
                #if BQ769X0_DEBUG
                if(g_Debug)
                    Serial.println(F("bq769x0 ERROR: OCD"));
                #endif
            }

            errorStatus_.regByte = sys_stat.regByte;
        }
        else {
            errorStatus_.regByte = 0;
        }
    }
    return errorStatus_.regByte;
}

//----------------------------------------------------------------------------
// tries to clear errors which have been found by checkStatus()

void bq769x0::clearErrors()
{
    if(errorStatus_.bits.DEVICE_XREADY)
    {
        // datasheet recommendation: try to clear after waiting a few seconds
        if((unsigned long)(millis() - errorTimestamps_[ERROR_XREADY]) > 3UL * 1000UL)
        {
            #if BQ769X0_DEBUG
            if(g_Debug)
                Serial.println(F("Attempting to clear XREADY error"));
            #endif
            writeRegister(SYS_STAT, STAT_DEVICE_XREADY);
            enableCharging(1 << ERROR_XREADY);
            enableDischarging(1 << ERROR_XREADY);
            errorStatus_.bits.DEVICE_XREADY = 0;
        }
    }
    if(errorStatus_.bits.OVRD_ALERT)
    {
        #if BQ769X0_DEBUG
        if(g_Debug)
            Serial.println(F("Attempting to clear ALERT error"));
        #endif
        writeRegister(SYS_STAT, STAT_OVRD_ALERT);
        enableCharging(1 << ERROR_ALERT);
        enableDischarging(1 << ERROR_ALERT);
        errorStatus_.bits.OVRD_ALERT = 0;
    }
    if(errorStatus_.bits.UV)
    {
        if(cellVoltages_[idCellMinVoltage_] > minCellVoltage_)
        {
            #if BQ769X0_DEBUG
            if(g_Debug)
                Serial.println(F("Attempting to clear UVP error"));
            #endif
            writeRegister(SYS_STAT, STAT_UV);
            enableDischarging(1 << ERROR_UVP);
            errorStatus_.bits.UV = 0;
        }
    }
    if(errorStatus_.bits.OV)
    {
        if(cellVoltages_[idCellMaxVoltage_] < maxCellVoltage_)
        {
            #if BQ769X0_DEBUG
            if(g_Debug)
                Serial.println(F("Attempting to clear OVP error"));
            #endif
            writeRegister(SYS_STAT, STAT_OV);
            enableCharging(1 << ERROR_OVP);
            errorStatus_.bits.OV = 0;
        }
    }
    if(errorStatus_.bits.SCD)
    {
        if((unsigned long)(millis() - errorTimestamps_[ERROR_SCD]) > 10UL * 1000UL)
        {
            #if BQ769X0_DEBUG
            if(g_Debug)
                Serial.println(F("Attempting to clear SCD error"));
            #endif
            writeRegister(SYS_STAT, STAT_SCD);
            enableDischarging(1 << ERROR_SCD);
            errorStatus_.bits.SCD = 0;
        }
    }
    if(errorStatus_.bits.OCD)
    {
        if((unsigned long)(millis() - errorTimestamps_[ERROR_OCD]) > 10UL * 1000UL)
        {
            #if BQ769X0_DEBUG
            if(g_Debug)
                Serial.println(F("Attempting to clear OCD error"));
            #endif
            writeRegister(SYS_STAT, STAT_OCD);
            enableDischarging(1 << ERROR_OCD);
            errorStatus_.bits.OCD = 0;
        }
    }
}

//----------------------------------------------------------------------------
// should be called at least once every 250 ms to get correct coulomb counting

uint8_t bq769x0::update()
{
    uint8_t ret = checkStatus(); // does updateCurrent()
    //updateCurrent(); // will only read new current value if alert was triggered
    updateVoltages();
    updateTemperatures();
    updateBalancingSwitches();
    if(ret)
        clearErrors();
    checkUser();
    return ret;
}

//----------------------------------------------------------------------------
// puts BMS IC into SHIP mode (i.e. switched off)

void bq769x0::shutdown()
{
    writeRegister(SYS_CTRL1, 0x0);
    writeRegister(SYS_CTRL1, 0x1);
    writeRegister(SYS_CTRL1, 0x2);
}

//----------------------------------------------------------------------------

bool bq769x0::enableCharging(uint16_t flag)
{
    chargingDisabled_ &= ~flag;

    if(!chargingEnabled_ && !chargingDisabled_)
    {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000001);  // switch CHG on
        chargingEnabled_ = true;
        #if BQ769X0_DEBUG
        if(g_Debug)
            Serial.println(F("Enabling CHG FET"));
        #endif
        return true;
    }
    else {
        return chargingEnabled_;
    }
}

//----------------------------------------------------------------------------

void bq769x0::disableCharging(uint16_t flag)
{
    chargingDisabled_ |= flag;

    if(chargingEnabled_ && chargingDisabled_)
    {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000001);  // switch CHG off
        chargingEnabled_ = false;
        #if BQ769X0_DEBUG
        if(g_Debug)
            Serial.println(F("Disabling CHG FET"));
        #endif
    }
}

//----------------------------------------------------------------------------

bool bq769x0::enableDischarging(uint16_t flag)
{
    dischargingDisabled_ &= ~flag;

    if(!dischargingEnabled_ && !dischargingDisabled_)
    {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000010);  // switch DSG on
        dischargingEnabled_ = true;
        #if BQ769X0_DEBUG
        if(g_Debug)
            Serial.println(F("Enabling DISCHG FET"));
        #endif
        return true;
    }
    else {
        return dischargingEnabled_;
    }
}

//----------------------------------------------------------------------------

void bq769x0::disableDischarging(uint16_t flag)
{
    dischargingDisabled_ |= flag;

    if(dischargingEnabled_ && dischargingDisabled_)
    {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000010);  // switch DSG off
        dischargingEnabled_ = false;
        #if BQ769X0_DEBUG
        if(g_Debug)
            Serial.println(F("Disabling DISCHG FET"));
        #endif
    }
}

//----------------------------------------------------------------------------

void bq769x0::enableAutoBalancing(void)
{
    autoBalancingEnabled_ = true;
}

void bq769x0::disableAutoBalancing(void)
{
    autoBalancingEnabled_ = false;
}

//----------------------------------------------------------------------------

void bq769x0::setBalancingThresholds(uint16_t idleTime_s, uint16_t absVoltage_mV, byte voltageDifference_mV)
{
    balancingMinIdleTime_s_ = idleTime_s;
    balancingMinCellVoltage_mV_ = absVoltage_mV;
    balancingMaxVoltageDifference_mV_ = voltageDifference_mV;
}

//----------------------------------------------------------------------------
// sets balancing registers if balancing is allowed
// (sufficient idle time + voltage)

void bq769x0::updateBalancingSwitches(void)
{
    int32_t idleSeconds = (millis() - idleTimestamp_) / 1000;
    uint8_t numberOfSections = (numberOfCells_ + 4) / 5;

    // check for millis() overflow
    if (idleSeconds < 0) {
        idleTimestamp_ = 0;
        idleSeconds = millis() / 1000;
    }

    // check if balancing allowed
    if (autoBalancingEnabled_ && errorStatus_.regByte == 0 &&
        ((balanceCharging_ && charging_ == 2) || idleSeconds >= balancingMinIdleTime_s_) &&
        cellVoltages_[idCellMaxVoltage_] > balancingMinCellVoltage_mV_ &&
        (cellVoltages_[idCellMaxVoltage_] - cellVoltages_[idCellMinVoltage_]) > balancingMaxVoltageDifference_mV_)
    {
        //Serial.println("Balancing enabled!");
        balancingStatus_ = 0;  // current status will be set in following loop

        //regCELLBAL_t cellbal;
        uint16_t balancingFlags;
        uint16_t balancingFlagsTarget;

        for (uint8_t section = 0; section < numberOfSections; section++)
        {
            // find cells which should be balanced and sort them by voltage descending
            uint8_t cellList[5];
            uint8_t cellCounter = 0;
            for (uint8_t i = 0; i < 5; i++)
            {
                if (cellVoltages_[section*5 + i] < 500)
                    continue;

                if ((cellVoltages_[section*5 + i] - cellVoltages_[idCellMinVoltage_]) > balancingMaxVoltageDifference_mV_)
                {
                    int j = cellCounter;
                    while (j > 0 && cellVoltages_[section*5 + cellList[j - 1]] < cellVoltages_[section*5 + i])
                    {
                        cellList[j] = cellList[j - 1];
                        j--;
                    }
                    cellList[j] = i;
                    cellCounter++;
                }
            }

            balancingFlags = 0;
            for (uint8_t i = 0; i < cellCounter; i++)
            {
                // try to enable balancing of current cell
                balancingFlagsTarget = balancingFlags | (1 << cellList[i]);

                // check if attempting to balance adjacent cells
                bool adjacentCellCollision =
                    ((balancingFlagsTarget << 1) & balancingFlags) ||
                    ((balancingFlags << 1) & balancingFlagsTarget);

                if (adjacentCellCollision == false) {
                    balancingFlags = balancingFlagsTarget;
                }
            }

            #if BQ769X0_DEBUG
            if(g_Debug) {
                Serial.print(F("Setting CELLBAL"));
                Serial.print(section+1);
                Serial.print(F(" register to: "));
                Serial.println(byte2char(balancingFlags));
            }
            #endif

            balancingStatus_ |= balancingFlags << section*5;

            // set balancing register for this section
            writeRegister(CELLBAL1+section, balancingFlags);

        } // section loop
    }
    else if (balancingStatus_ > 0)
    {
        // clear all CELLBAL registers
        for (uint8_t section = 0; section < numberOfSections; section++)
        {
            #if BQ769X0_DEBUG
            if(g_Debug) {
                Serial.print(F("Clearing Register CELLBAL"));
                Serial.println(section+1);
            }
            #endif

            writeRegister(CELLBAL1+section, 0x0);
        }

        balancingStatus_ = 0;
    }
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getBalancingStatus()
{
    return balancingStatus_;
}

//----------------------------------------------------------------------------

void bq769x0::setShuntResistorValue(uint32_t res_uOhm)
{
    shuntResistorValue_uOhm_ = res_uOhm;
}

//----------------------------------------------------------------------------

void bq769x0::setThermistors(uint8_t bitflag)
{
    thermistors_ = bitflag & ((1 << MAX_NUMBER_OF_THERMISTORS) - 1);
}

//----------------------------------------------------------------------------

void bq769x0::setThermistorBetaValue(uint16_t beta_K)
{
    thermistorBetaValue_ = beta_K;
}

//----------------------------------------------------------------------------

void bq769x0::setBatteryCapacity(int32_t capacity_mAh, uint16_t nomVoltage_mV, uint16_t fullVoltage_mV)
{
    nominalCapacity_ = capacity_mAh * 60 * 60;
    nominalVoltage_ = nomVoltage_mV;
    fullVoltage_ = fullVoltage_mV;
}

//----------------------------------------------------------------------------

void bq769x0::setOCV(uint16_t voltageVsSOC[NUM_OCV_POINTS])
{
    OCV_ = voltageVsSOC;
}

//----------------------------------------------------------------------------

float bq769x0::getSOC(void)
{
    return (float) coulombCounter_ / nominalCapacity_ * 100.0;
}

//----------------------------------------------------------------------------
// SOC calculation based on average cell open circuit voltage

void bq769x0::resetSOC(int percent)
{
    if (percent <= 100 && percent >= 0)
    {
        coulombCounter_ = (int32_t)(nominalCapacity_ * percent) / 100L;
    }
    else  // reset based on OCV
    {
        #if BQ769X0_DEBUG
        if(g_Debug) {
            Serial.print(F("NumCells: "));
            Serial.print(getNumberOfConnectedCells());
            Serial.print(F(", voltage: "));
            Serial.print(getBatteryVoltage());
            Serial.println(F("V"));
        }
        #endif
        uint16_t voltage = getBatteryVoltage() / getNumberOfConnectedCells();

        coulombCounter_ = 0;  // initialize with totally depleted battery (0% SOC)

        for (int i = 0; i < NUM_OCV_POINTS; i++)
        {
            if (OCV_[i] <= voltage) {
                if (i == 0) {
                    coulombCounter_ = nominalCapacity_;  // 100% full
                }
                else {
                    // interpolate between OCV[i] and OCV[i-1]
                    coulombCounter_ = (double) nominalCapacity_ / (NUM_OCV_POINTS - 1.0) *
                    (NUM_OCV_POINTS - 1.0 - i + ((float)voltage - OCV_[i])/(OCV_[i-1] - OCV_[i]));
                }
                return;
            }
        }
    }
}

//----------------------------------------------------------------------------

void bq769x0::adjADCPackOffset(int16_t offset)
{
    adcPackOffset_ = offset;
}

int16_t bq769x0::getADCPackOffset()
{
    return adcOffset_ + adcPackOffset_;
}

void bq769x0::adjADCCellsOffset(int16_t offsets[MAX_NUMBER_OF_CELLS])
{
    adcCellsOffset_ = offsets;
}

int16_t bq769x0::getADCCellOffset(uint8_t cell)
{
    if(adcCellsOffset_)
        return adcOffset_ + adcCellsOffset_[cell];
    return adcOffset_;
}

//----------------------------------------------------------------------------

void bq769x0::setTemperatureLimits(int16_t minDischarge_degC, int16_t maxDischarge_degC,
  int16_t minCharge_degC, int16_t maxCharge_degC)
{
    // Temperature limits (°C/10)
    minCellTempDischarge_ = minDischarge_degC * 10;
    maxCellTempDischarge_ = maxDischarge_degC * 10;
    minCellTempCharge_ = minCharge_degC * 10;
    maxCellTempCharge_ = maxCharge_degC * 10;
}

//----------------------------------------------------------------------------

void bq769x0::setIdleCurrentThreshold(uint32_t current_mA)
{
    idleCurrentThreshold_ = current_mA;
}

//----------------------------------------------------------------------------

void bq769x0::setBalanceCharging(bool charging)
{
    balanceCharging_ = charging;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::setShortCircuitProtection(uint32_t current_mA, uint16_t delay_us)
{
    regPROTECT1_t protect1;
    protect1.bits.RSNS = PROTECT1_RSNS;

    protect1.bits.SCD_THRESH = 0;
    uint8_t temp = (current_mA * shuntResistorValue_uOhm_) / 1000000UL;
    for (uint8_t i = sizeof(SCD_threshold_setting)-1; i > 0; i--) {
        if (temp >= SCD_threshold_setting[i]) {
            protect1.bits.SCD_THRESH = i;
            break;
        }
    }

    protect1.bits.SCD_DELAY = 0;
    for (uint8_t i = sizeof(SCD_delay_setting)-1; i > 0; i--) {
        if (delay_us >= SCD_delay_setting[i]) {
            protect1.bits.SCD_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT1, protect1.regByte);

    // returns the actual current threshold value
    return ((uint32_t)SCD_threshold_setting[protect1.bits.SCD_THRESH] * 1000000UL) /
        shuntResistorValue_uOhm_;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::setOvercurrentChargeProtection(uint32_t current_mA, uint16_t delay_ms)
{
    maxChargeCurrent_ = current_mA;
    maxChargeCurrent_delay_ = delay_ms;
    return 0;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::setOvercurrentDischargeProtection(uint32_t current_mA, uint16_t delay_ms)
{
    regPROTECT2_t protect2;

    protect2.bits.OCD_THRESH = 0;
    uint8_t temp = (current_mA * shuntResistorValue_uOhm_) / 1000000UL;
    for (uint8_t i = sizeof(OCD_threshold_setting)-1; i > 0; i--) {
        if (temp >= OCD_threshold_setting[i]) {
            protect2.bits.OCD_THRESH = i;
            break;
        }
    }

    protect2.bits.OCD_DELAY = 0;
    for (uint8_t i = sizeof(OCD_delay_setting)-1; i > 0; i--) {
        if (delay_ms >= OCD_delay_setting[i]) {
            protect2.bits.OCD_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT2, protect2.regByte);

    // returns the actual current threshold value
    return ((uint32_t)OCD_threshold_setting[protect2.bits.OCD_THRESH] * 1000000UL) /
        shuntResistorValue_uOhm_;
}


//----------------------------------------------------------------------------

uint16_t bq769x0::setCellUndervoltageProtection(uint16_t voltage_mV, uint16_t delay_s)
{
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);

    minCellVoltage_ = voltage_mV;

    uint16_t uv_trip = ((((voltage_mV - adcOffset_) * 1000UL) / adcGain_) >> 4) & 0x00FF;
    uv_trip += 1;   // always round up for lower cell voltage
    writeRegister(UV_TRIP, uv_trip);

    protect3.bits.UV_DELAY = 0;
    for (uint8_t i = sizeof(UV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= UV_delay_setting[i]) {
            protect3.bits.UV_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT3, protect3.regByte);

    // returns the actual voltage threshold value
    return ((1UL << 12UL | uv_trip << 4) * adcGain_) / 1000UL + adcOffset_;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::setCellOvervoltageProtection(uint16_t voltage_mV, uint16_t delay_s)
{
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);

    maxCellVoltage_ = voltage_mV;

    uint16_t ov_trip = ((((voltage_mV - adcOffset_) * 1000UL) / adcGain_) >> 4) & 0x00FF;
    writeRegister(OV_TRIP, ov_trip);

    protect3.bits.OV_DELAY = 0;
    for (uint8_t i = sizeof(OV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= OV_delay_setting[i]) {
            protect3.bits.OV_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT3, protect3.regByte);

    // returns the actual voltage threshold value
    return ((uint32_t)(1 << 13 | ov_trip << 4) * adcGain_) / 1000UL + adcOffset_;
}


//----------------------------------------------------------------------------

int32_t bq769x0::getBatteryCurrent(bool raw)
{
    if (raw)
        return batCurrent_raw_;
    return batCurrent_;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::getBatteryVoltage(bool raw)
{
    if (raw)
        return batVoltage_raw_;
    return batVoltage_;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getMaxCellVoltage()
{
    return cellVoltages_[idCellMaxVoltage_];
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getMinCellVoltage()
{
    return cellVoltages_[idCellMinVoltage_];
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getAvgCellVoltage()
{
    return getBatteryVoltage() / getNumberOfConnectedCells();
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getCellVoltage(uint8_t idCell, bool raw)
{
    uint8_t i = cellIdMap_[idCell];
    if (raw)
        return cellVoltages_raw_[i];
    return cellVoltages_[i];
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getCellVoltage_(uint8_t i, bool raw)
{
    if (raw)
        return cellVoltages_raw_[i];
    return cellVoltages_[i];
}

//----------------------------------------------------------------------------

uint8_t bq769x0::getNumberOfCells(void)
{
    return numberOfCells_;
}

//----------------------------------------------------------------------------

uint8_t bq769x0::getNumberOfConnectedCells(void)
{
    return connectedCells_;
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegC(uint8_t channel)
{
    if (channel >= 0 && channel <= 2) {
        return (float)temperatures_[channel] / 10.0;
    }
    else {
        return -273.15;   // Error: Return absolute minimum temperature
    }
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegF(uint8_t channel)
{
    return getTemperatureDegC(channel) * 1.8 + 32;
}

//----------------------------------------------------------------------------

int16_t bq769x0::getLowestTemperature()
{
    int16_t minTemp = INT16_MAX;
    for(uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++)
    {
        if(thermistors_ & (1 << i) && temperatures_[i] < minTemp)
            minTemp = temperatures_[i];
    }
    return minTemp;
}

int16_t bq769x0::getHighestTemperature()
{
    int16_t maxTemp = INT16_MIN;
    for(uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++)
    {
        if(thermistors_ & (1 << i) && temperatures_[i] > maxTemp)
            maxTemp = temperatures_[i];
    }
    return maxTemp;
}

//----------------------------------------------------------------------------

void bq769x0::updateTemperatures()
{
    float tmp = 0;
    int adcVal = 0;
    int vtsx = 0;
    unsigned long rts = 0;

    // calculate R_thermistor according to bq769x0 datasheet
    adcVal = readDoubleRegister(TS1_HI_BYTE);
    vtsx = adcVal * 0.382; // mV
    rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm

    // Temperature calculation using Beta equation
    // - According to bq769x0 datasheet, only 10k thermistors should be used
    // - 25°C reference temperature for Beta equation assumed
    tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue_*log(rts/10000.0)); // K
    temperatures_[0] = (tmp - 273.15) * 10.0;

    if (type_ == bq76930 || type_ == bq76940) {
        adcVal = readDoubleRegister(TS2_HI_BYTE);
        vtsx = adcVal * 0.382; // mV
        rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
        tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue_*log(rts/10000.0)); // K
        temperatures_[1] = (tmp - 273.15) * 10.0;
    }

    if (type_ == bq76940) {
        adcVal = readDoubleRegister(TS3_HI_BYTE);
        vtsx = adcVal * 0.382; // mV
        rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
        tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue_*log(rts/10000.0)); // K
        temperatures_[2] = (tmp - 273.15) * 10.0;
    }
}


//----------------------------------------------------------------------------

void bq769x0::updateCurrent()
{
    regSYS_STAT_t sys_stat;
    sys_stat.regByte = readRegister(SYS_STAT);

    // check if new current reading available
    if (sys_stat.bits.CC_READY == 1)
    {
        //Serial.println("reading CC register...");
        batCurrent_raw_ = (int16_t)readDoubleRegister(CC_HI_BYTE);
        batCurrent_ = ((int32_t)batCurrent_raw_ * 8440L) / (int32_t)shuntResistorValue_uOhm_;  // mA

        // is read every 250 ms
        coulombCounter_ += batCurrent_ / 4;

        if (coulombCounter_ > nominalCapacity_) {
            coulombCounter_ = nominalCapacity_;
        }
        if (coulombCounter_ < 0) {
            coulombCounter_ = 0;
        }

        if (batCurrent_ < 0) {
            coulombCounter2_ += -batCurrent_ / 4;
            if (coulombCounter2_ > nominalCapacity_) {
                batCycles_++;
                coulombCounter2_ = 0;
            }
        }

        if (batCurrent_ > (int32_t)idleCurrentThreshold_) {
            if (!charging_) {
                charging_ = 1;
                chargeTimestamp_ = millis();
            }
            else if (charging_ == 1 && (unsigned long)(millis() - chargeTimestamp_) > 60UL * 1000UL) {
                charging_ = 2;
                chargedTimes_++;
            }
        }
        else if (charging_ != 2 || batCurrent_ < 10)
            charging_ = 0;

        // reset idleTimestamp
        if (abs(batCurrent_) > idleCurrentThreshold_) {
            if(batCurrent_ < 0 || !(balanceCharging_ && charging_ == 2))
                idleTimestamp_ = millis();
        }

        // no error occured which caused alert
        if (!(sys_stat.regByte & 0b00111111)) {
            alertInterruptFlag_ = false;
        }

        writeRegister(SYS_STAT, 0b10000000);  // Clear CC ready flag
    }
}

//----------------------------------------------------------------------------
// reads all cell voltages to array cellVoltages[NUM_CELLS] and updates batVoltage

void bq769x0::updateVoltages()
{
    uint16_t adcVal = 0;
    uint8_t idCell = 0;

    // read cell voltages
    digitalWrite(BMS_I2C_FET_PIN, LOW);
    Wire.beginTransmission(I2CAddress_);
    Wire.write(VC1_HI_BYTE);
    Wire.endTransmission();

    idCellMaxVoltage_ = 0;
    idCellMinVoltage_ = 0;
    for (int i = 0; i < numberOfCells_; i++)
    {
        if (crcEnabled_ == true) {
            Wire.requestFrom(I2CAddress_, 4U);
            uint8_t crc;
            uint8_t data = Wire.read();
            adcVal = (data & 0b00111111) << 8;

            // CRC of first bytes includes slave address (including R/W bit) and data
            crc = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 1);
            crc = _crc8_ccitt_update(crc, data);
            if (crc != Wire.read()) {
                digitalWrite(BMS_I2C_FET_PIN, HIGH);
                return; // don't save corrupted value
            }

            data = Wire.read();
            adcVal |= data;

            // CRC of subsequent bytes contain only data
            crc = _crc8_ccitt_update(0, data);
            if (crc != Wire.read()) {
                digitalWrite(BMS_I2C_FET_PIN, HIGH);
                return; // don't save corrupted value
            }
        }
        else {
            Wire.requestFrom(I2CAddress_, 2U);
            adcVal = (Wire.read() & 0b00111111) << 8 | Wire.read();
        }

        cellVoltages_raw_[i] = adcVal;
        cellVoltages_[i] = ((uint32_t)adcVal * adcGain_) / 1000 + getADCCellOffset(i);

        if (cellVoltages_[i] < 500) {
            continue;
        }

        cellIdMap_[idCell] = i;

        if (cellVoltages_[i] > cellVoltages_[idCellMaxVoltage_]) {
            idCellMaxVoltage_ = i;
        }
        if (cellVoltages_[i] < cellVoltages_[idCellMinVoltage_]) {
            idCellMinVoltage_ = i;
        }

        idCell++;
    }
    digitalWrite(BMS_I2C_FET_PIN, HIGH);
    connectedCells_ = idCell;

    // read battery pack voltage
    batVoltage_raw_ = readDoubleRegister(BAT_HI_BYTE);
    batVoltage_ = ((uint32_t)4.0 * adcGain_ * batVoltage_raw_) / 1000.0 + connectedCells_ * getADCPackOffset();

    if(batVoltage_ >= connectedCells_ * fullVoltage_) {
        if(fullVoltageCount_ == 240) { // 60s * 4(250ms)
            resetSOC(100);
        }
        if(fullVoltageCount_ < 255)
            fullVoltageCount_++;
    } else {
        fullVoltageCount_ = 0;
    }
}

//----------------------------------------------------------------------------

void bq769x0::writeRegister(uint8_t address, uint8_t data)
{
    digitalWrite(BMS_I2C_FET_PIN, LOW);
    Wire.beginTransmission(I2CAddress_);

    Wire.write(address);
    Wire.write(data);

    if (crcEnabled_ == true) {
        // CRC is calculated over the slave address (including R/W bit), register address, and data.
        uint8_t crc;
        crc = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 0);
        crc = _crc8_ccitt_update(crc, address);
        crc = _crc8_ccitt_update(crc, data);

        Wire.write(crc);
    }

    Wire.endTransmission();
    digitalWrite(BMS_I2C_FET_PIN, HIGH);
}

//----------------------------------------------------------------------------

uint8_t bq769x0::readRegister(uint8_t address)
{
    digitalWrite(BMS_I2C_FET_PIN, LOW);
    Wire.beginTransmission(I2CAddress_);
    Wire.write(address);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        digitalWrite(BMS_I2C_FET_PIN, HIGH);
        return 0;
    }

    uint8_t data;
    if (crcEnabled_ == true) {
        uint8_t wantcrc;
        uint8_t gotcrc;
        do {
            Wire.requestFrom(I2CAddress_, 2U);
            data = Wire.read();
            gotcrc = Wire.read();

            // CRC is calculated over the slave address (including R/W bit) and data.
            wantcrc = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 1);
            wantcrc = _crc8_ccitt_update(wantcrc, data);
        } while (gotcrc != wantcrc);
    }
    else {
        Wire.requestFrom(I2CAddress_, 1U);
        data = Wire.read();
    }
    digitalWrite(BMS_I2C_FET_PIN, HIGH);

    return data;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::readDoubleRegister(uint8_t address)
{
    digitalWrite(BMS_I2C_FET_PIN, LOW);
    Wire.beginTransmission(I2CAddress_);
    Wire.write(address);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        digitalWrite(BMS_I2C_FET_PIN, HIGH);
        return 0;
    }

    uint16_t result;
    if (crcEnabled_ == true) {
        while(true)
        {
            Wire.requestFrom(I2CAddress_, 4U);
            uint8_t crc;
            uint8_t data = Wire.read();
            result = (uint16_t)data << 8;

            // CRC of first bytes includes slave address (including R/W bit) and data
            crc = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 1);
            crc = _crc8_ccitt_update(crc, data);
            if (crc != Wire.read()) {
                continue;
            }

            data = Wire.read();
            result |= data;

            // CRC of subsequent bytes contain only data
            crc = _crc8_ccitt_update(0, data);
            if (crc != Wire.read()) {
                continue;
            }
            break;
        }
    }
    else {
        Wire.requestFrom(I2CAddress_, 2U);
        result = ((uint16_t)Wire.read() << 8) | Wire.read();
    }
    digitalWrite(BMS_I2C_FET_PIN, HIGH);

    return result;
}

//----------------------------------------------------------------------------
// The bq769x0 drives the ALERT pin high if the SYS_STAT register contains
// a new value (either new CC reading or an error)

void bq769x0::setAlertInterruptFlag()
{
    interruptTimestamp_ = millis();
    alertInterruptFlag_ = true;
}

//----------------------------------------------------------------------------
// Check custom error conditions like over/under temperature, over charge current

void bq769x0::checkUser()
{
    // charge temperature limits
    if(getLowestTemperature() < minCellTempCharge_ || getHighestTemperature() > maxCellTempCharge_)
    {
        if(!(chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP)))
        {
            disableCharging(1 << ERROR_USER_CHG_TEMP);
            errorCounter_[ERROR_USER_CHG_TEMP]++;
            errorTimestamps_[ERROR_USER_CHG_TEMP] = millis();
        }
    }
    else if(chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP))
    {
        enableCharging(1 << ERROR_USER_CHG_TEMP);
    }

    // discharge temperature limits
    if(getLowestTemperature() < minCellTempDischarge_ || getHighestTemperature() > maxCellTempDischarge_)
    {
        if(!(dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP)))
        {
            disableDischarging(1 << ERROR_USER_DISCHG_TEMP);
            errorCounter_[ERROR_USER_DISCHG_TEMP]++;
            errorTimestamps_[ERROR_USER_DISCHG_TEMP] = millis();
        }
    }
    else if(dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP))
    {
        enableDischarging(1 << ERROR_USER_DISCHG_TEMP);
    }

    // charge current limit
    // charge current can also come through discharge FET that we can't turn off (regen on P-)
    // that's why this looks a bit funky
    if(batCurrent_ > maxChargeCurrent_)
    {
        user_CHGOCD_ReleaseTimestamp_ = 0;

        if(chargingEnabled_ && !(chargingDisabled_ & (1 << ERROR_USER_CHG_OCD)))
        {
            if(!user_CHGOCD_TriggerTimestamp_)
                user_CHGOCD_TriggerTimestamp_ = millis();

            if((millis() - user_CHGOCD_TriggerTimestamp_) > maxChargeCurrent_delay_ || user_CHGOCD_ReleasedNow_)
            {
                disableCharging(1 << ERROR_USER_CHG_OCD);
                errorCounter_[ERROR_USER_CHG_OCD]++;
                errorTimestamps_[ERROR_USER_CHG_OCD] = millis();
            }
        }
    }
    else
    {
        user_CHGOCD_TriggerTimestamp_ = 0;
        user_CHGOCD_ReleasedNow_ = false;

        if(chargingDisabled_ & (1 << ERROR_USER_CHG_OCD))
        {
            if(!user_CHGOCD_ReleaseTimestamp_)
                user_CHGOCD_ReleaseTimestamp_ = millis();

            if((unsigned long)(millis() - user_CHGOCD_ReleaseTimestamp_) > 10UL * 1000UL)
            {
                enableCharging(1 << ERROR_USER_CHG_OCD);
                user_CHGOCD_ReleaseTimestamp_ = 0;
                user_CHGOCD_ReleasedNow_ = true;
            }
        }
    }
}

#if BQ769X0_DEBUG

//----------------------------------------------------------------------------
// for debug purposes

void bq769x0::printRegisters()
{
    Serial.print(F("0x00 SYS_STAT:  "));
    Serial.println(byte2char(readRegister(SYS_STAT)));

    Serial.print(F("0x01 CELLBAL1:  "));
    Serial.println(byte2char(readRegister(CELLBAL1)));

    Serial.print(F("0x04 SYS_CTRL1: "));
    Serial.println(byte2char(readRegister(SYS_CTRL1)));

    Serial.print(F("0x05 SYS_CTRL2: "));
    Serial.println(byte2char(readRegister(SYS_CTRL2)));

    Serial.print(F("0x06 PROTECT1:  "));
    Serial.println(byte2char(readRegister(PROTECT1)));

    Serial.print(F("0x07 PROTECT2:  "));
    Serial.println(byte2char(readRegister(PROTECT2)));

    Serial.print(F("0x08 PROTECT3   "));
    Serial.println(byte2char(readRegister(PROTECT3)));

    Serial.print(F("0x09 OV_TRIP:   "));
    Serial.println(byte2char(readRegister(OV_TRIP)));

    Serial.print(F("0x0A UV_TRIP:   "));
    Serial.println(byte2char(readRegister(UV_TRIP)));

    Serial.print(F("0x0B CC_CFG:    "));
    Serial.println(byte2char(readRegister(CC_CFG)));

    Serial.print(F("0x32 CC_HI_LO:  "));
    Serial.println(readDoubleRegister(CC_HI_BYTE));

    Serial.print(F("0x2A BAT_HI_LO: "));
    Serial.println(readDoubleRegister(BAT_HI_BYTE));

    Serial.print(F("ADCGAIN:        "));
    Serial.println(adcGain_);
    Serial.print(F("ADCOFFSET:      "));
    Serial.println(adcOffset_);

    Serial.print(F("CHG DIS:        "));
    Serial.println(chargingDisabled_);
    Serial.print(F("DISCHG DIS:     "));
    Serial.println(dischargingDisabled_);
}

#endif
