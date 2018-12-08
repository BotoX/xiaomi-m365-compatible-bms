# Xiaomi M365 compatible BMS

#### Warning: This project is meant for people with electronics and microcontroller knowledge!

This repository contains alternative firmware for the following BMS hardware: [SP15SV0001-LLT](https://www.lithiumbatterypcb.com/product/13s-48v-li-ion-battery-pcb-board-54-6v-lithium-bms-with-60a-discharge-current-for-electric-motorcycle-and-e-scooter-protection-2-2-3-2-2-2-2-2/).  
It's programmed with the [Arduino](https://www.arduino.cc/) platform and is built with [PlatformIO](https://platformio.org/).  
It runs on an ATMega328p MCU and controls a TI BQ769x0 battery monitoring IC over I²C.

This is a fully fledged replacement BMS for the Xiaomi M365 that implements their proprietary BMS protocol and supports all of the features you'd expect, like:

 * Battery SOC (State Of Charge: mAh, %) using Coulomb Counting
 * Pack Voltage, Cell Voltages, Current, Temperature
 * Discharge and Charge cycles

All the battery information can be viewed in the native apps or 3rd party apps like [m365 Tools](https://play.google.com/store/apps/details?id=app.peretti.m365tools).

Here's some pr0n of my 12S4P NCR18650B battery: [Gallery](https://cloud.botox.bz/apps/gallery/s/94drnBJfjacBDnr).


## The caveat
The BMS uses an ATMega328p MCU without an external crystal and thus has to use the internal 8 MHz resonator.  
However at 8 MHz the hardware UART will not function at 115200 baud, which is the baudrate the M365 controller expects.  
The frequency of the internal resonator can be calibrated (manually with an oscilloscope or logic analyzer) using OSCCAL to still make this work.  

But instead we simply patch the M365 firmware to use 76800 baud which works just fine with the internal 8 MHz resonator and no calibration trouble.  
Patch your firmware at [m365beta.botox.bz](https://m365beta.botox.bz/) and use the "Change ESC<->BMS baud rate to 76800" option.  
Thanks to Oleg for the idea and help.


## Hardware
### Requirements
* The BMS itself: [SP15SV0001-LLT](https://www.lithiumbatterypcb.com/product/13s-48v-li-ion-battery-pcb-board-54-6v-lithium-bms-with-60a-discharge-current-for-electric-motorcycle-and-e-scooter-protection-2-2-3-2-2-2-2-2/)
  * 10S - 13S. 30A version is recommended, >30A versions have both sides of the BMS PCB populated with MOSFETs and will not fit in the limited space of the M365.
* An ISP programmer with a 6pin ISP cable/adapter, example: [Aliexpress](https://www.aliexpress.com/item/10-Pin-Convert-to-Standard-6-Pin-Adapter-Board-USBASP-USBISP-AVR-Programmer-USB/2055099231.html)
* Serial UART adapter, sold by the BMS shop or on [Aliexpress](https://www.aliexpress.com/item/1PCS-CP2102-USB-2-0-to-TTL-UART-Module-6Pin-Serial-Converter-STC-Replace-FT232/32717057832.html)


### Current Shunt resistors
By default the BMS comes with ten 4mOhm shunt resistors in parallel. This results in a shunt resistance of 0.4mOhm, this is too small for accurate coulomb counting. In addition the voltage coming from the shunt resistors is also cut in half.

A simple fix for this is to simply remove six of the ten 4mOhm resistors, this leave four 4mOhm resistors. Thus the shunt resistance will be 1mOhm.  
And to fix the shunt voltage from getting cut in half two more small resistors are removed from the top layer, as in the pictures: [Bottom](https://cloud.botox.bz/s/J6oZWqJDikzpTw8/preview) and [Top](https://cloud.botox.bz/s/2ipzTsJNWQ222TH/preview).


## Software
### Configuration
Depending on your battery you might want to configure some of the settings here: [src/main.h](src/main.h#L61)

* `capacity`: The *actual* total capacity in mAh of your battery pack, don't use the manufacturer stated capacity if you want to have accurate SOC but rather look at sites like [lygte-info.dk](https://lygte-info.dk/), etc.
  * Example: for 4 * NCR18650B I used a value of 12400mAh.
* `nominal_voltage`: The nominal voltage in mV of your cells, this will be 3.6V for almost all cells.
* `full_voltage`: The voltage in mV that you will charge your cells to. I only charge mine to 4.1V for cycle life.
* `ODP_current`: Over current protection value in mA, if your scooter is shutting off on your crazy settings then make this higher.
* `UVP_voltage`: The BMS will shut off when any cells voltage goes below this.
  * Remember: During load the cell voltage will drop a lot.
* `OVP_voltage`: The BMS will shut off when any cells voltage goes above this.
  * **IMPORTANT**: During high speed braking with a full battery the voltage per cell can go up by more than 100mV for a short time!
  * If you set this too low then you risk destroying your controller, the BMS shuts off and the system voltage will shoot up!
  * I recommend making this 100mV higher than your `full_voltage`.

### Compiling
This project uses [PlatformIO](https://platformio.org/), please check out their [Documentation](https://docs.platformio.org/en/latest/) to get started.

### ISP programming
Connect your ISP programmer to the BMS, you can (and should) keep the battery disconnected while programming.  
Here's the pinout of the ISP header: [Image](https://cloud.botox.bz/s/qGa7rS6Ktt4pG24/preview)

You can use PlatformIO to program the BMS.  
You will probably have to adjust the `upload_port` (COM port) in the [platformio.ini](platformio.ini) file.  
Please do check out the [Documentation](https://docs.platformio.org/en/latest/platforms/atmelavr.html) from PlatformIO.

You can also use any other program to flash the .hex file generated by PlatformIO to your BMS.

#### Important
** Make sure NOT to change the fuses on your BMS! **

If you program wrong fuses you can brick your BMS.  
For example: The BMS doesn't have an external oscillator, if you program fuses that try to use the external oscillator your BMS will be bricked until you connect an external oscillator to it!

The default fuses are: Low = 0xE2, High = 0xDA, Extended = 0xFD, Lockbits = 0xFE


## Troubleshooting
Have you patched your M365 firmware for 76800 baud?

Make sure the temperature sensors are plugged in and all wires are connected properly,  C- too.

Reset the BMS by shorting GND with RST on the ISP header.

Run the `configtool.py` in an interactive python shell (IDLE on windows) and configure your COM port in there correctly first.  
You'll probably have to install these two dependencies: [cstruct](https://pypi.org/project/cstruct/) and [pyserial](https://pypi.org/project/pyserial/).  
Check the source code for commands you can use, though you'll probably only need `debug_print()`.


## M365
After you've checked that your BMS works (voltage between + and C-) and communicates UART (it print's `BOOTED!` when it boots) you can connect it with your M365.

The original BMS was connected to the top 3pin header on the M365 ESC: [Image](https://cloud.botox.bz/s/QLzWYc9C253QECi/preview)  
You'll have to connect the **R** pin to the BMSs **TX** pin and the **T** pin to the BMSs **RX** pin.  
The **L** pin is + for the red light on the back fender, you'll have to make your own cable for that.


## Final words
### Credits
A big part of the BQ769x0 code is taken from here: [LibreSolar/bq769x0_mbed_lib](https://github.com/LibreSolar/bq769x0_mbed_lib).

### Support
If you've spent at least an hour with your issue you can ask about it nicely in my [M365 Telegram group](https://t.me/XiaomiM365Hacking).

### Disclaimer
If you brake anything it's your own fault.  
**Works for me™.** is the only guarantee I can give you.

I am in no way affiliated with the company that makes the BMS. I just bought it, reversed some stuff and made this firmware.
