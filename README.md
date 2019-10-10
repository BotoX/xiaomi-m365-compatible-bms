# Xiaomi M365 compatible BMS

#### Warning: This project is meant for people with electronics and microcontroller knowledge!

This repository contains alternative firmware for the following BMS hardware: [SP15SV0001-LLT](https://www.lithiumbatterypcb.com/product/13s-48v-li-ion-battery-pcb-board-54-6v-lithium-bms-with-60a-discharge-current-for-electric-motorcycle-and-e-scooter-protection-2-2-3-2-2-2-2-2/).  
It's programmed with the [Arduino](https://www.arduino.cc/) platform and is built with [PlatformIO](https://platformio.org/).  
It runs on an ATMega328p MCU and controls a TI BQ769x0 ([TI Datasheet](http://www.ti.com/lit/ds/symlink/bq76940.pdf)) battery monitoring IC over I²C.

This is a fully fledged replacement BMS for the Xiaomi M365 that implements their proprietary BMS protocol and supports all of the features you'd expect, like:

 * Battery SOC (State Of Charge: mAh, %) using Coulomb Counting
 * Pack Voltage, Cell Voltages, Current, Temperature
 * Discharge and Charge cycles

All the battery information can be viewed in the native apps or 3rd party apps like [m365 Tools](https://play.google.com/store/apps/details?id=app.peretti.m365tools).

Here's some pr0n of my 12S4P NCR18650B battery: [Gallery](https://cloud.botox.bz/apps/gallery/s/94drnBJfjacBDnr).  
Materials I used:
- [Malectrics DIY Arduino Battery Spot Welder](https://malectrics.eu/product/diy-arduino-battery-spot-welder-prebuilt-kit-v3/)
- [8mm*0.2mm pure nickel strip](https://www.aliexpress.com/item/Pure-Nickel-Strip-for-Scientific-research-18650-battery-welding-Size-0-2-8mm/32739610924.html) (0.15mm is easier to weld)
- [18650 positive insulation pad](https://www.aliexpress.com/item/100Pcs-18650-lithium-battery-positive-electrode-hollow-flat-head-insulation-pad-meson-18500-positive-surface-pad/32850696072.html) **important**
- [125mm/80mm Battery PVC shrink wrap](https://www.aliexpress.com/item/125mm-Diameter-80mm-PVC-Heat-Shrink-Tubing-for-Battery-Wrap-Free-Shipping/32790874560.html)
- [50mm kapton tape](https://www.aliexpress.com/item/5-6-8-10-12-15-20-25-30-40-50MM-x-30M-Tape-Sticky-High/32889388030.html)
- I used automotive windshield glue that I had lying around, anything that sticks well to PVC should work.
- 18650 Cells: 
  - I used NCR18650B because I got them cheap.
  - Good price/performance: [Samsung INR18650-29E](https://eu.nkon.nl/samsung-inr18650-29e.html)
- Charger: Mean Well HLG-240H-48A or ELG-240-48A
  - I paid 45€ for it at [getgoods.com](https://www.getgoods.com/products/442045/Mean-Well-HLG-240H-48A-LED-driver-LED-transformer-Constant-voltage-Constant-current-240-W-5-A-48-Vdc-PFC-circuit-Surge.html) but pricing seems to vary a lot by country.
  - Both Mean Well HLG-240H-48A and ELG-240-48A allow you to set the max current and voltage.
    - I charge with 4A (1A per cell) and to 4.1V per cell for better cycle life.
  - Look for another CC/CV power supply if this one is too expensive.
    - Keep in mind that the chager output voltage has to be adjustable to 49.2V (50.4V if you plan on running 4.2V per cell).
  


## The caveat
The BMS uses an ATMega328p MCU without an external crystal and thus has to use the internal 8 MHz resonator.  
However at 8 MHz the hardware UART will not function at 115200 baud, which is the baudrate the M365 controller expects.  
The frequency of the internal resonator can be calibrated (manually with an oscilloscope or logic analyzer) using OSCCAL to still make this work.  

But instead we simply patch the M365 firmware to use 76800 baud which works just fine with the internal 8 MHz resonator and no calibration trouble.  
Patch your firmware at [m365beta.botox.bz](https://m365beta.botox.bz/) and use the "Change ESC<->BMS baud rate to 76800" option.  
Thanks to Oleg for the idea and help.


## Hardware
### Requirements
* The BMS itself: [SP15SV0001-LLT](https://www.lithiumbatterypcb.com/product/13s-48v-li-ion-battery-pcb-board-54-6v-lithium-bms-with-60a-discharge-current-for-electric-motorcycle-and-e-scooter-protection-2-2-3-2-2-2-2-2/) or on [Aliexpress](https://www.aliexpress.com/item/12S-44-4V-smart-Lithium-li-ion-battery-protection-board-BMS-system-60A-20A-Bluetooth-phone/32976215661.html)
  * 10S - 12S. 30A version is recommended, >30A versions have both sides of the BMS PCB populated with MOSFETs and will not fit in the limited space of the M365.
* An ISP programmer with a 6pin ISP cable/adapter, example: [Aliexpress](https://www.aliexpress.com/item/10-Pin-Convert-to-Standard-6-Pin-Adapter-Board-USBASP-USBISP-AVR-Programmer-USB/2055099231.html)
* Serial UART adapter, sold by the BMS shop or on [Aliexpress](https://www.aliexpress.com/item/1PCS-CP2102-USB-2-0-to-TTL-UART-Module-6Pin-Serial-Converter-STC-Replace-FT232/32717057832.html)


### Current Shunt resistors
By default the BMS comes with ten 4mOhm shunt resistors in parallel. This results in a shunt resistance of 0.4mOhm, this is too small for accurate coulomb counting. In addition the voltage coming from the shunt resistors is also cut in half.

A simple fix for this is to simply remove some of the 4mOhm resistors so that only four resistors are left. Thus the shunt resistance will be 1mOhm.  
And to fix the shunt voltage from getting cut in half two more small resistors are removed from the top layer, as in the pictures: [Bottom](https://cloud.botox.bz/s/J6oZWqJDikzpTw8/preview) and [Top](https://cloud.botox.bz/s/2ipzTsJNWQ222TH/preview).

#### Current Shunt Resistors Calibration
One way to calibrate the BMS to know the exact current:

While measuring the charging current with a calibrated multimeter you can query the RAW current value via `configtool.py` using `debug_print()` command.
Now we make some calculations: ```R[uOhm]  = RAW Value * 8440 / A[mA]```.

 An example:
  * measured 4,1169A charging current
  * raw value of 445
  * Results in ```R =  445 * 8440 / 4116,9 = 911,69```
  * used in Settings: ```g_Settings.shuntResistor_uOhm = 912```. 

_Please refer to [Software/Configuration](#Configuration) on how to get, change, put, apply and save Settings for the BMS._

### Wiring
#### IMPORTANT: The M365 ESC - connects to P- of the BMS!
**Otherwise the M365 will be damaged when you brake with a full battery due to overvoltage!**

**C- has to be used to charge the battery, otherwise the BMS offers no protection against faulty chargers/overvoltage!**

**Do not cut the big - trace on the M365 ESC or it will damage the BMS because of different GND potentials on UART!**

**Do not connect GND from the BMS to the M365 anywhere! P- is GND for the M365! The only extra wires going from the BMS to the M365 are RX and TX!**

## Software
### Configuration
Depending on your battery you might want to configure some of the settings here: [src/main.h](src/main.h#L24)

* `capacity`: The *actual* total capacity in mAh of your battery pack, don't use the manufacturer stated capacity if you want to have accurate SOC but rather look at sites like [lygte-info.dk](https://lygte-info.dk/), etc.
  * Example: for 4 * NCR18650B I used a value of 12400mAh.
* `nominal_voltage`: The nominal voltage in mV of your cells, this will be 3.6V for almost all cells.
* `full_voltage`: The voltage in mV that you will charge your cells to. I only charge mine to 4.1V for cycle life.
* `ODP_current`: Over current protection value in mA, if your scooter is shutting off on your crazy settings then make this higher.
* `UVP_voltage`: The BMS will shut off P- when any cells voltage goes below this.
  * Remember: During load the cell voltage will drop a lot.
* `OVP_voltage`: The BMS will shut off C- when any cells voltage goes above this.
  * So do not connect your M365 ESC to C- or it'll die when you brake with a full battery, use P- for the ESC and C- to charge!

If you've already flashed and run the firmware on your BMS you can only make changes using `configtool.py` (since the settings are written to EEPROM).  
Use `getSettings()` to populate g_Settings, then make changes like so: `g_Settings.capacity = 7800`.  
If you're done with making changes use: `putSettings()` to push them to the BMS, use `applySettings()` to apply them and `saveSettings()` to write them to EEPROM.

### Compiling
This project uses [PlatformIO](https://platformio.org/), please check out their [Documentation](https://docs.platformio.org/en/latest/) to get started.

### Programming bootloader via ISP
Connect your ISP programmer to the BMS, you can (and should) keep the battery disconnected while programming.  
Here's the pinout of the ISP header: [Image](https://cloud.botox.bz/s/qGa7rS6Ktt4pG24/preview)  
On the new V1.5 PCB it's a little trickier: [Image1](https://cloud.botox.bz/s/eYmBCM4Z44P84tj/preview) [Image2](https://cloud.botox.bz/s/7BkSS7NKk878B4d/preview)

Choose the correct bootloader file (the MHZ value in the Filename **must** match your hardware, default = 8MHz)

Choose the correct fuses for your hardware
* for internal 8MHz Clock use ```-U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m``` (Hardware version <= 1.5)
* for external 8MHz Clock use ```-U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m``` (Hardware version >= 1.6)
* for anything else think twice what values you are using as you can brick your atmega chip with wrong values, use of tools like [AVR Fuse Calculator](http://www.engbedded.com/fusecalc) is recommended.

Finally flash the bootloader with [AVRDUDE](https://
download.savannah.gnu.org/releases/avrdude/avrdude-6.3-mingw32.zip) using the following command: `avrdude -patmega328p -cstk500v2 -P/dev/ttyUSB0 -U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m -U lock:w:0x3F:m -U flash:w:optiboot_atmega328_8mhz_57600bps.hex`  
* Adjust the `-P/dev/ttyUSB0` part to the correct COM port on your PC. (Omit for `usbasp`)
* Adjust the `-cstk500v2` part to your programmer (`-cusbasp` for the [Aliexpress](https://www.aliexpress.com/item/10-Pin-Convert-to-Standard-6-Pin-Adapter-Board-USBASP-USBISP-AVR-Programmer-USB/2055099231.html) example one.)
* Adjust the `-U` parts with your fuse values

### Uploading/Updating firmware
You can upload the firmware in platformio, there's a little arrow somewhere.  
You have to short the RESET pin to GROUND right before you hit the upload button!  
For updating firmware you don't have to short the RESET pin but can run `bootloader.py /dev/ttyUSB0` right before you hit upload.

### Optional: Building optiboot yourself
Clone the [Optiboot repository](https://github.com/Optiboot/optiboot/) and `git apply` this patch: [optiboot.diff](optiboot.diff)
* 8MHz build: `make atmega328 AVR_FREQ=8000000L PRODUCTION=1 BAUD_RATE=57600 LED_START_FLASHES=0`
* 16MHz build: `make atmega328 AVR_FREQ=16000000L PRODUCTION=1 BAUD_RATE=115200 LED_START_FLASHES=0`


## Troubleshooting
Have you patched your M365 firmware for 76800 baud?

Make sure the temperature sensors are plugged in and all wires are connected properly.
B- needs to be connected to the battery -.

Reset the BMS by shorting GND with RST on the ISP header.

Does your Voltmeter only show a too low voltage like 10 or 20 Volt? -> Reset the BMS by shorting GND with RST on the ISP header. The AVR Chip crashed during plugging the balancing connector.

Run `configtool.py /dev/ttyUSB0` in an interactive python shell (IDLE on Windows, `python -i` on Linux) with the correct COM port.  
You'll probably have to install these two dependencies: [cstruct](https://pypi.org/project/cstruct/) and [pyserial](https://pypi.org/project/pyserial/).  
Check the source code for commands you can use, though you'll probably only need `debug_print()`.
You can also use any serial terminal and send this string in HEX `55aa0322fa0500dbfe` instead of `debug_print()`.  
Since the controller will enter sleep mode after 5 seconds of inactivity you'll have to send this twice quickly when using a serial terminal.


## M365
After you've checked that your BMS works (voltage between + and P-) and communicates via UART (it print's `BOOTED!` when it boots) you can connect it with your M365.

The original BMS was connected to the top 3pin header on the M365 ESC: [Image](https://cloud.botox.bz/s/QLzWYc9C253QECi/preview)  
You'll have to connect the ESC **R** pin to the BMS **TX** pin and the **T** pin to the **RX** pin.  
The ESC **L** pin is + for the red/brake light on the back fender, you'll have to make your own cable for that.


## Final words
### Credits
A big part of the BQ769x0 code is taken from here: [LibreSolar/bq769x0_mbed_lib](https://github.com/LibreSolar/bq769x0_mbed_lib).

### Support
If you've spent at least an hour with your issue you can ask about it nicely in my [M365 Telegram group](https://t.me/XiaomiM365Hacking).

### Disclaimer
If you break anything it's your own fault.  
**Works for me™.** is the only guarantee I can give you.

I am in no way affiliated with the company that makes the BMS. I just bought it, reversed some stuff and made this firmware.
 
