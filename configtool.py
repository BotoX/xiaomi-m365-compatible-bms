#!/usr/bin/python
import serial
import time
import threading
import sys
from queue import Queue
from binascii import hexlify
import cstruct

g_Running = True
COMPORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
ser = serial.Serial(COMPORT, 76800)

cstruct.typedef("uint8", "uint8_t")
cstruct.typedef("int8", "int8_t")
cstruct.typedef("uint16", "uint16_t")
cstruct.typedef("int16", "int16_t")
cstruct.typedef("uint32", "uint32_t")
cstruct.typedef("int32", "int32_t")

class M365BMS(cstruct.CStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __struct__ = """
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
    """

class BMSSettings(cstruct.CStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __struct__ = """
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
        uint32_t ODP_current = 40000; // mA
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
    """


g_Settings = BMSSettings()
g_M365BMS = M365BMS()

g_Queue = Queue(maxsize=1)

class RecvThread(threading.Thread):
    def run(self):
        while g_Running:
            msg = {'header': bytes(), 'data': bytes()}
            recvd = 0
            chk = 0

            while g_Running:
                b = ser.read()
                bi = int.from_bytes(b, 'little')
                recvd += 1

                if recvd == 1:
                    if bi != 0x55:
                        sys.stdout.write(b.decode('ascii', 'replace'))
                        break
                    msg['header'] += b

                elif recvd == 2:
                    if bi != 0xAA:
                        break
                    msg['header'] += b

                elif recvd == 3:
                    msg['len'] = bi
                    chk = bi

                elif recvd == 4:
                    msg['addr'] = b
                    chk += bi

                elif recvd == 5:
                    if bi == 0x65 or bi == 0x64:
                        break
                    msg['mode'] = b
                    chk += bi

                elif recvd == 6:
                    msg['ofs'] = bi
                    chk += bi

                else:
                    if recvd - 7 < msg['len'] - 2:
                        msg['data'] += b
                        chk += bi
                    elif recvd - 7 - msg['len'] + 2 == 0:
                        msg['chk'] = bi
                    else:
                        msg['chk'] |= bi << 8
                        chk ^= 0xFFFF

                        if chk != msg['chk']:
                            print('!!! checksum: {:02X} != {:02X}'.format(msg['chk'], chk))
                            break

                        g_Queue.put(msg, False)

                        break

def m365_send(length, addr, mode, offset, data):
    for i in range(5):
        ser.write(0xFF) # Wake
        time.sleep(0.01)

    arg = [length, addr, mode, offset]
    arg.extend(data)
    crc = sum(arg) ^ 0xFFFF

    send = [0x55, 0xAA]
    send.extend(arg)

    send.append(crc & 0xFF)
    send.append((crc >> 8) & 0xFF)
    send = bytes(send)
    print(hexlify(send))
    ser.write(send)

def m365_recv():
    d = g_Queue.get(True, 1)
    g_Queue.task_done()
    return d


#################
# User Commands #
#################

# BMS Settings -> g_Settings
def getSettings():
    m365_send(3, 0x22, 0xF1, 0, [len(g_Settings)])
    d = m365_recv()
    g_Settings.unpack(d['data'])

# g_Settings -> BMS Settings
def putSettings():
    d = g_Settings.pack()
    m365_send(2 + len(g_Settings), 0x22, 0xF3, 0, list(d))

# apply (new) BMS settings
def applySettings():
    m365_send(3, 0x22, 0xFA, 1, [0])

# save BMS settings to EEPROM
def saveSettings():
    m365_send(3, 0x22, 0xFA, 3, [0])


# BMS M365 -> g_M365
def getM365BMS():
    m365_send(3, 0x22, 0x01, 0, [len(g_M365BMS)])
    d = m365_recv()
    g_M365BMS.unpack(d['data'])

# g_M365 -> BMS M365
def putM365BMS():
    d = g_M365BMS.pack()
    m365_send(2 + len(g_M365BMS), 0x22, 0x03, 0, list(d))


# BMS g_Debug = <enable>
def debug(enable):
    on = 1 if enable else 0
    m365_send(3, 0x22, 0xFA, 4, [on])

# call debug_print() on BMS
def debug_print():
    m365_send(3, 0x22, 0xFA, 5, [0])


# DISCHG & CHG FET OFF
def disable():
    m365_send(3, 0x22, 0xFA, 6, [0])

# DISCHG & CHG FET ON
def enable():
    m365_send(3, 0x22, 0xFA, 7, [0])

# Bluetooth power OFF
def blue_off():
    m365_send(3, 0x22, 0xFA, 8, [0])

# Bluetooth power ON
def blue_on():
    m365_send(3, 0x22, 0xFA, 9, [0])

# Hangs the BMS software and relies on watchdog to rest it
def watchdog_test():
    m365_send(3, 0x22, 0xFA, 10, [0])

# Reboots to bootloader
def bootloader():
    m365_send(3, 0x22, 0xFA, 11, [0])

# YOU HAVE TO INSERT THE EMPTY (0) SLOTS YOURSELF !!!
def calc_cellofs(measured):
    getM365BMS()
    bms = g_M365BMS.cell_voltages[:len(measured)]
    ofs = []
    for i, a in enumerate(measured):
        a *= 1000
        b = bms[i]
        ofs.append(round(a - b))
    return ofs


recvT = RecvThread()
recvT.start()
