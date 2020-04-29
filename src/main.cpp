#include <Arduino.h>
#include <limits.h>
#include <EEPROM.h>
#include <PinChangeInterrupt.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include "bq769x0.h"
#include "main.h"

M365BMS g_M365BMS;
BMSSettings g_Settings;

bool g_Debug = true;
// I2CAddress = 0x08, crcEnabled = true
bq769x0 g_BMS(bq76940, 0x08, true);
volatile bool g_interruptFlag = false;
unsigned long g_lastActivity = 0;
unsigned long g_lastUpdate = 0;
volatile bool g_uartRxInterrupted = false;
volatile bool g_wakeupFlag = false;

volatile bool g_K1Flag = false;
bool g_dischargeEnabled = true;

unsigned long g_oldMillis = 0;
int g_millisOverflows = 0;

extern volatile unsigned long timer0_millis;
volatile unsigned int g_timer2Overflows = 0;

void alertISR()
{
    g_BMS.setAlertInterruptFlag();
    g_interruptFlag = true;
    g_wakeupFlag = true;
}

#ifdef K1SWITCH
void K1ISR()
{
    g_K1Flag = true;
    g_wakeupFlag = true;
}
#endif

void uartRxISR()
{
    g_uartRxInterrupted = true;
    g_wakeupFlag = true;
}

ISR(TIMER2_OVF_vect)
{
    // only used to keep track of time while sleeping to adjust millis()
    g_timer2Overflows++;
}

void setup()
{
    MCUSR = 0;
    wdt_disable();

    Serial.begin(76800);
    Serial.println(F("BOOTED!"));

    power_adc_disable();
    power_spi_disable();
    power_timer1_disable();
    power_twi_disable();
    delay(1000);

    loadSettings();

    g_BMS.begin(BMS_BOOT_PIN);
    g_BMS.setThermistors(0b110);

    applySettings();

    // Bluetooth power pin
    pinMode(BMS_VDD_EN_PIN, OUTPUT);

    // attach ALERT interrupt
    pinMode(BMS_ALERT_PIN, INPUT);
    attachPCINT(digitalPinToPCINT(BMS_ALERT_PIN), alertISR, RISING);

#ifdef K1SWITCH
    // attach K1 / power switch interrupt
    pinMode(BMS_K1_PIN, INPUT);
    attachPCINT(digitalPinToPCINT(BMS_K1_PIN), K1ISR, CHANGE);
#endif

    // attach UART RX pin interrupt to wake from deep sleep
    attachPCINT(digitalPinToPCINT(0), uartRxISR, CHANGE);
    disablePCINT(digitalPinToPCINT(0));

    interrupts();

    delay(1000);
    g_BMS.update();
    g_BMS.resetSOC(100);

#ifdef K1SWITCH
    g_dischargeEnabled = !digitalRead(BMS_K1_PIN);
#endif

    g_BMS.enableCharging();
    if(g_dischargeEnabled)
        g_BMS.enableDischarging();

    g_Debug = false;

    wdt_enable(WDTO_1S);
}


void loadSettings()
{
    if(EEPROM.read(0) != g_Settings.header[0] || EEPROM.read(1) != g_Settings.header[1])
    {
        for(uint16_t i = 0; i < EEPROM.length(); i++) {
            EEPROM.write(i, 0);
        }
        EEPROM.put(0, g_Settings);
    }
    else
    {
        EEPROM.get(0, g_Settings);
    }
}

void saveSettings()
{
    EEPROM.put(0, g_Settings);
}

void applySettings()
{
    g_BMS.setBatteryCapacity(g_Settings.capacity, g_Settings.nominal_voltage, g_Settings.full_voltage);

    g_BMS.setShuntResistorValue(g_Settings.shuntResistor_uOhm);
    g_BMS.setThermistorBetaValue(g_Settings.thermistor_BetaK);

    g_BMS.setTemperatureLimits(g_Settings.temp_minDischargeC,
                             g_Settings.temp_maxDischargeC,
                             g_Settings.temp_minChargeC,
                             g_Settings.temp_maxChargeC);
    g_BMS.setShortCircuitProtection(g_Settings.SCD_current, g_Settings.SCD_delay);
    g_BMS.setOvercurrentChargeProtection(g_Settings.OCD_current, g_Settings.OCD_delay);
    g_BMS.setOvercurrentDischargeProtection(g_Settings.ODP_current, g_Settings.ODP_delay);
    g_BMS.setCellUndervoltageProtection(g_Settings.UVP_voltage, g_Settings.UVP_delay);
    g_BMS.setCellOvervoltageProtection(g_Settings.OVP_voltage, g_Settings.OVP_delay);

    g_BMS.setBalancingThresholds(g_Settings.balance_minIdleTime,
                               g_Settings.balance_minVoltage,
                               g_Settings.balance_maxVoltageDiff);
    g_BMS.setIdleCurrentThreshold(g_Settings.idle_currentThres);
    if(g_Settings.balance_enabled)
        g_BMS.enableAutoBalancing();
    else
        g_BMS.disableAutoBalancing();

    g_BMS.setBalanceCharging(true);

    g_BMS.adjADCPackOffset(g_Settings.adcPackOffset);
    g_BMS.adjADCCellsOffset(g_Settings.adcCellsOffset);

    strncpy(g_M365BMS.serial, g_Settings.serial, sizeof(g_M365BMS.serial));
    g_M365BMS.design_capacity = g_M365BMS.real_capacity = g_Settings.capacity;
    g_M365BMS.nominal_voltage = g_Settings.nominal_voltage;
    g_M365BMS.date = g_Settings.date;
    g_M365BMS.num_cycles = g_Settings.num_cycles;
    g_M365BMS.num_charged = g_Settings.num_charged;
}


void onNinebotMessage(NinebotMessage &msg)
{
    // Enable TX
    UCSR0B |= (1 << TXEN0);

    if(msg.addr != M365BMS_RADDR)
        return;

    if(msg.mode == 0x01 || msg.mode == 0xF1)
    {
        if(msg.length != 3)
            return;

        uint16_t ofs = (uint16_t)msg.offset * 2; // word aligned
        uint8_t sz = msg.data[0];

        if(sz > sizeof(NinebotMessage::data))
            return;

        msg.addr = M365BMS_WADDR;
        msg.length = 2 + sz;

        if(msg.mode == 0x01)
        {
            if((ofs + sz) > sizeof(g_M365BMS))
                return;

            memcpy(&msg.data, &((uint8_t *)&g_M365BMS)[ofs], sz);
        }
        else if(msg.mode == 0xF1)
        {
            if((ofs + sz) > sizeof(g_Settings))
                return;

            memcpy(&msg.data, &((uint8_t *)&g_Settings)[ofs], sz);
        }

        ninebotSend(msg);
    }
    else if(msg.mode == 0x03 || msg.mode == 0xF3)
    {
        uint16_t ofs = (uint16_t)msg.offset * 2; // word aligned
        uint8_t sz = msg.length - 2;

        if(msg.mode == 0x03)
        {
            if((ofs + sz) > sizeof(g_M365BMS))
                return;

            memcpy(&((uint8_t *)&g_M365BMS)[ofs], &msg.data, sz);
        }
        else if(msg.mode == 0xF3)
        {
            if((ofs + sz) > sizeof(g_Settings))
                return;

            memcpy(&((uint8_t *)&g_Settings)[ofs], &msg.data, sz);
        }
    }
    else if(msg.mode == 0xFA)
    {
        switch(msg.offset)
        {
            case 1: {
                applySettings();
            } break;
            case 2: {
                EEPROM.get(0, g_Settings);
            } break;
            case 3: {
                EEPROM.put(0, g_Settings);
            } break;
#if BQ769X0_DEBUG
            case 4: {
                g_Debug = msg.data[0];
            } break;
            case 5: {
                debug_print();
            } break;
#endif
            case 6: {
                g_BMS.disableDischarging();
                g_BMS.disableCharging();
            } break;
            case 7: {
                g_BMS.enableDischarging();
                g_BMS.enableCharging();
            } break;

            case 8: {
                digitalWrite(BMS_VDD_EN_PIN, LOW);
            } break;
            case 9: {
                digitalWrite(BMS_VDD_EN_PIN, HIGH);
            } break;
#if BQ769X0_DEBUG
            case 10: {
                // test watchdog
                for (;;) { (void)0; }
            } break;
#endif
            case 11: {
                // restart to bootloader
                typedef void (*do_reboot_t)(void);
                const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1);
                wdt_disable();
                cli(); TCCR0A = TCCR1A = TCCR2A = 0; // make sure interrupts are off and timers are reset.
                MCUSR = 0;
                do_reboot();
            }
        }
    }
}

void ninebotSend(NinebotMessage &msg)
{
    msg.checksum = (uint16_t)msg.length + msg.addr + msg.mode + msg.offset;

    Serial.write(msg.header[0]);
    Serial.write(msg.header[1]);
    Serial.write(msg.length);
    Serial.write(msg.addr);
    Serial.write(msg.mode);
    Serial.write(msg.offset);
    for(uint8_t i = 0; i < msg.length - 2; i++)
    {
        Serial.write(msg.data[i]);
        msg.checksum += msg.data[i];
    }

    msg.checksum ^= 0xFFFF;
    Serial.write(msg.checksum & 0xFF);
    Serial.write((msg.checksum >> 8) & 0xFF);
}

void ninebotRecv()
{
    static NinebotMessage msg;
    static uint8_t recvd = 0;
    static unsigned long begin = 0;
    static uint16_t checksum;

    while(Serial.available())
    {
        g_lastActivity = millis();

        if(millis() >= begin + 100)
        { // 100ms timeout
            recvd = 0;
        }

        uint8_t byte = Serial.read();
        recvd++;

        switch(recvd)
        {
            case 1:
            {
                if(byte != 0x55)
                { // header1 mismatch
                    recvd = 0;
                    break;
                }

                msg.header[0] = byte;
                begin = millis();
            } break;

            case 2:
            {
                if(byte != 0xAA)
                { // header2 mismatch
                    recvd = 0;
                    break;
                }

                msg.header[1] = byte;
            } break;

            case 3: // length
            {
                if(byte < 2)
                { // too small
                    recvd = 0;
                    break;
                }

                msg.length = byte;
                checksum = byte;
            } break;

            case 4: // addr
            {
                if(byte != M365BMS_RADDR)
                { // we're not the receiver of this message
                    recvd = 0;
                    break;
                }

                msg.addr = byte;
                checksum += byte;
            } break;

            case 5: // mode
            {
                msg.mode = byte;
                checksum += byte;
            } break;

            case 6: // offset
            {
                msg.offset = byte;
                checksum += byte;
            } break;

            default:
            {
                if(recvd - 7 < msg.length - 2)
                { // data
                    msg.data[recvd - 7] = byte;
                    checksum += byte;
                }
                else if(recvd - 7 - msg.length + 2 == 0)
                { // checksum LSB
                    msg.checksum = byte;
                }
                else
                { // checksum MSB and transmission finished
                    msg.checksum |= (uint16_t)byte << 8;
                    checksum ^= 0xFFFF;

                    if(checksum != msg.checksum)
                    { // invalid checksum
                        recvd = 0;
                        break;
                    }

                    onNinebotMessage(msg);
                    recvd = 0;
                }
            } break;
        }
    }
}


void loop()
{
    unsigned long now = millis();
#ifdef K1SWITCH
    if(g_K1Flag)
    {
        g_K1Flag = false;
        g_dischargeEnabled = !digitalRead(BMS_K1_PIN);

        if(g_dischargeEnabled)
            g_BMS.enableDischarging();
        else
            g_BMS.disableDischarging();
    }
#endif

    if(g_interruptFlag || (unsigned long)(now - g_lastUpdate) >= 500)
    {
        if(g_interruptFlag)
            g_interruptFlag = false;

        uint8_t error = g_BMS.update(); // should be called at least every 250 ms
        g_lastUpdate = now;

        // update M365BMS struct
        {
            // charging state
            if(g_BMS.getBatteryCurrent() > (int16_t)g_Settings.idle_currentThres)
                g_M365BMS.status |= (1 << 6); // charging
            else if(g_BMS.getBatteryCurrent() < (int16_t)g_Settings.idle_currentThres / 2)
                g_M365BMS.status &= ~(1 << 6);

            if(error & STAT_OV) {
                g_M365BMS.status |= (1 << 9); // overvoltage
                error &= ~STAT_OV;
            }
            else
                g_M365BMS.status &= ~(1 << 9);

            uint16_t batVoltage = g_BMS.getBatteryVoltage() / 10;
            if(batVoltage > g_M365BMS.max_voltage)
                g_M365BMS.max_voltage = batVoltage;

            int16_t batCurrent = g_BMS.getBatteryCurrent() / 10;
            if(batCurrent > 0 && (uint16_t)batCurrent > g_M365BMS.max_charge_current)
                g_M365BMS.max_charge_current = batCurrent;
            else if(batCurrent < 0 && (uint16_t)-batCurrent > g_M365BMS.max_discharge_current)
                g_M365BMS.max_discharge_current = -batCurrent;

            g_M365BMS.capacity_left = g_M365BMS.design_capacity * g_BMS.getSOC() / 100.0;
            g_M365BMS.percent_left = g_BMS.getSOC();
            g_M365BMS.current = -batCurrent;
            g_M365BMS.voltage = batVoltage;
            g_M365BMS.temperature[0] = g_BMS.getTemperatureDegC(1) + 20.0;
            g_M365BMS.temperature[1] = g_BMS.getTemperatureDegC(2) + 20.0;

            if(g_BMS.getHighestTemperature() > (g_Settings.temp_maxDischargeC - 3) * 10)
                g_M365BMS.status |= (1 << 10); // overheat
            else
                g_M365BMS.status &= ~(1 << 10);

            if(g_BMS.batCycles_) {
                g_M365BMS.num_cycles += g_BMS.batCycles_;
                g_BMS.batCycles_ = 0;
                g_Settings.num_cycles = g_M365BMS.num_cycles;
                EEPROM.put(0, g_Settings);
            }

            if(g_BMS.chargedTimes_) {
                g_M365BMS.num_charged += g_BMS.chargedTimes_;
                g_Settings.num_charged = g_M365BMS.num_charged;
                g_BMS.chargedTimes_ = 0;
            }

            uint8_t numCells = g_BMS.getNumberOfConnectedCells();
            for(uint8_t i = 0; i < numCells; i++)
                g_M365BMS.cell_voltages[i] = g_BMS.getCellVoltage(i);

            // cell voltage difference too big
            uint16_t bigDelta = g_BMS.getMaxCellVoltage() - g_BMS.getMinCellVoltage();
            if(bigDelta > 100)
                error = 1;

            if(error)
                g_M365BMS.status &= ~1;
            else
                g_M365BMS.status |= 1;
        }

        if(g_oldMillis > now)
            g_millisOverflows++;
        g_oldMillis = now;
    }

    ninebotRecv();

    if((unsigned long)(now - g_lastActivity) >= 5000 && !g_Debug)
    {
        // Disable TX
        UCSR0B &= ~(1 << TXEN0);

        // go into deep sleep, will wake up every 250ms by BQ769x0 ALERT or from USART1 RX (first byte will be lost)
        noInterrupts();
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);

        // Timer/Counter2 8-byte OVF 8MHz /1024 = 32.64ms
        TCCR2A = 0;
        TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);
        TCNT2 = 0;
        TIMSK2 = (1<<TOIE2);

        UCSR0B &= ~(1 << RXEN0); // Disable RX
        enablePCINT(digitalPinToPCINT(0));

        wdt_reset();
        g_wakeupFlag = false;

        sleep_enable();
        interrupts();
        do // go to sleep if it's just timer2 that woke us up (unless we were idle for longer than 500ms)
        {
            sleep_cpu();
        } while(!g_wakeupFlag && g_timer2Overflows < 16);
        sleep_disable();

        // Disable Timer/Counter2 and add elapsed time to Arduinos 'timer0_millis'
        TCCR2B = 0;
        TIMSK2 = 0;
        float elapsed_time = g_timer2Overflows * 32.64 + TCNT2 * 32.64 / 255.0;
        timer0_millis += (unsigned long)elapsed_time;
        g_timer2Overflows = 0;

        if(g_uartRxInterrupted)
            g_lastActivity = millis();
        g_uartRxInterrupted = false;

        disablePCINT(digitalPinToPCINT(0));
        UCSR0B |= (1 << RXEN0); // Enable RX

        interrupts();
    }

    wdt_reset();
}

#if BQ769X0_DEBUG
void debug_print()
{
    g_BMS.printRegisters();
    Serial.println(F(""));

    unsigned long uptime = g_millisOverflows * (UINT_MAX / 1000UL);
    uptime += millis() / 1000;
    Serial.print(F("uptime: "));
    Serial.println(uptime);

    Serial.print(F("Battery voltage: "));
    Serial.print(g_BMS.getBatteryVoltage());
    Serial.print(F(" ("));
    Serial.print(g_BMS.getBatteryVoltage(true));
    Serial.println(F(")"));

    Serial.print(F("Battery current: "));
    Serial.print(g_BMS.getBatteryCurrent());
    Serial.print(F(" ("));
    Serial.print(g_BMS.getBatteryCurrent(true));
    Serial.println(F(")"));

    Serial.print(F("SOC: "));
    Serial.println(g_BMS.getSOC());

    Serial.print(F("Temperature: "));
    Serial.print(g_BMS.getTemperatureDegC(1));
    Serial.print(F(" "));
    Serial.println(g_BMS.getTemperatureDegC(2));

    Serial.print(F("Balancing status: "));
    Serial.println(g_BMS.getBalancingStatus());

    Serial.print(F("Cell voltages ("));
    Serial.print(g_BMS.getNumberOfConnectedCells());
    Serial.print(F(" / "));
    int numCells = g_BMS.getNumberOfCells();
    Serial.print(numCells);
    Serial.println(F("):"));
    for(int i = 0; i < numCells; i++) {
        Serial.print(g_BMS.getCellVoltage_(i));
        Serial.print(F(" ("));
        Serial.print(g_BMS.getCellVoltage_(i, true));
        Serial.print(F(")"));
        if(i != numCells - 1)
            Serial.print(F(", "));
    }
    Serial.println(F(""));

    Serial.print(F("Cell V: Min: "));
    Serial.print(g_BMS.getMinCellVoltage());
    Serial.print(F(" | Avg: "));
    Serial.print(g_BMS.getAvgCellVoltage());
    Serial.print(F(" | Max: "));
    Serial.print(g_BMS.getMaxCellVoltage());
    Serial.print(F(" | Delta: "));
    Serial.println(g_BMS.getMaxCellVoltage() - g_BMS.getMinCellVoltage());

    Serial.print(F("maxVoltage: "));
    Serial.println(g_M365BMS.max_voltage);
    Serial.print(F("maxDischargeCurrent: "));
    Serial.println(g_M365BMS.max_discharge_current);
    Serial.print(F("maxChargeCurrent: "));
    Serial.println(g_M365BMS.max_charge_current);

    Serial.print(F("XREADY errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_XREADY]);
    Serial.print(F("ALERT errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_ALERT]);
    Serial.print(F("UVP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_UVP]);
    Serial.print(F("OVP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_OVP]);
    Serial.print(F("SCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_SCD]);
    Serial.print(F("OCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_OCD]);
    Serial.println();
    Serial.print(F("DISCHG TEMP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_DISCHG_TEMP]);
    Serial.print(F("CHG TEMP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_CHG_TEMP]);
    Serial.print(F("CHG OCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_CHG_OCD]);
}
#endif
