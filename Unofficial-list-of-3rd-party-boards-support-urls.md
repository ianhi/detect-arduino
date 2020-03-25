Have you added support for a non Arduino board to the Arduino IDE? Please let everyone know by publishing your package_**YOUR-NAME**_**PACKAGE-NAME**_index.json URL.

**If you are using Arduino IDE 1.6.6 then you may need to open Boards Manager twice before the entry for a newly added Boards Manager URL will appear** [(issue #3795)](https://github.com/arduino/Arduino/issues/3795)**.**

### List of 3rd party boards support URLs

* **Adafruit**: https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
  * Adafruit AVR Boards (Flora, Metro, Trinket, Pro Trinket, & Gemma)
  * Adafruit SAMD Boards (Feather M0)
  * [TeeOnArdu](https://github.com/adafruit/TeeOnArdu/tree/1.6.x) USB MIDI support for Leonardo & Micro
  * arcore USB MIDI support for Leonardo & Micro

* **Adelino**: http://adelino.cc/package_adelino_index.json
  * [Adelino](http://adelino.cc) is a _dual-microcontroller_ development board based on ATmega32U4 and ESP8266 (using ESP-12F WiFi module), inspired by Arduino Leonardo and Arduino Yún

* **Akafugu**: https://raw.githubusercontent.com/akafugu/akafugu_core/master/package_akafugu_index.json
  * Akafugu Breadboard Adapter (Internal 8MHz Clock)
  * Akafugu Breadboard Adapter (External 16MHz Clock)
  * Akafuino L
  * Akafugu Nixie Clock

* **Alorium**: https://raw.githubusercontent.com/AloriumTechnology/Arduino_Boards/master/package_aloriumtech_index.json
  * [XLR8](http://www.aloriumtech.com/): FPGA accelerated AVR compatible Boards

* **Ameba**: https://github.com/Ameba8195/Arduino/raw/master/release/package_realtek.com_ameba_index.json
  * Ameba (RTL8195AM, RTL8711AM, RTL8711AF)

* **Arduino MKR1000 - Beta**: http://downloads.arduino.cc/packages/package_mkr1000_index.json
  * Hackster.io - MKR1000 Build
  * This appears to be seriously outdated. MKR1000 support has long since been added to the standard Arduino SAMD Boards package available in Boards Manager without adding a 3rd party URL. Possibly this version was written for the beta test version of the MKR1000, which had a different pinout than the final version of the board?

* **Arachnid Labs**: https://raw.githubusercontent.com/arachnidlabs/arachnidlabs-boards/master/package_arachnidlabs.com_boards_index.json
  * Tsunami (http://www.arachnidlabs.com/tsunami/) Signal generator based on Arduino Leonardo (AD9838)

* **Ardhat**: https://ardhat.github.io/ardhat-board-support/arduino/package_ardhat_index.json
  * [Ardhat forums](http://forums.ardhat.com)

* **Arduboy**: https://arduboy.github.io/board-support/package_arduboy_index.json
  * [Arduboy](https://www.arduboy.com/) miniature game system and developer kit

* **Arducam**: http://www.arducam.com/downloads/ESP8266_UNO/package_ArduCAM_index.json
  * Arducam ESP8266 UNO 
  * Arducam ESP32S UNO (http://www.arducam.com/downloads/ESP32_UNO/package_ArduCAM_ESP32S_UNO_index.json)

* **Ariadne Bootloader**: https://per1234.github.io/Ariadne-Bootloader/package_codebendercc_ariadne-bootloader_index.json
  * Arduino Ethernet/Ethernet Shield/W5100 bootloader

* **Arrow**: https://raw.githubusercontent.com/ioteamit/smarteverything-core/master/package_arrow_index.json
  * SmartEverything Fox
  * SmartEverything Lion
  * SmartEverything Dragonfly
  * SmartEverything Tiger

* **ATFlash**: https://mesom.de/atflash/package_atflash_index.json
  * Engage the following controllers with 8 MHz (and 9600 Baud) standalone on breadboard (without _anything_ else!):
  * ATtiny84A
  * ATtiny85
  * ATmega328P
  * All can be programmed via an UNO and the integrated Tiny Safe Boot (TSB) bootloader or the integrated ISP programming
  * More on [the ATFlash website](https://mesom.de/atflash/index_english.html)

* **Atmega & Attiny cores**: http://www.leonardomiliani.com/repository/package_leonardomiliani.com_index.json
  * ATmega644/644P & ATmega1284P core (w/bootloader)
  * ATmega168P/328P core (for standalone MCUs)
  * ATtiny24/44/84, ATtiny25/45/85, and ATtiny2313/4313 core (I2C & SoftSerial)

* **ATmegaxxM1-C1**: https://thomasonw.github.io/ATmegaxxM1-C1/package_thomasonw_ATmegaxxM1-C1_index.json
  * ATmega32M1
  * ATmega64M1
  * Others to come.

* **ATtiny**: https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json
  * ATtiny45 / ATtiny85 and ATtiny44 / ATtiny84 support

* **ATtinyCore**: http://drazzy.com/package_drazzy.com_index.json
  * ATtiny25/45/85, 24/44/85, 261/461/861, 87/167, 48/88, 2313/4313
  * ATtiny441/841, 1634, 828 (optiboot support on 841/1634/828)

* **AutomationDirect.com**: https://raw.githubusercontent.com/facts-engineering/facts-engineering.github.io/master/package_productivity-P1AM-boardmanagermodule_index.json 
  * [P1AM-100](https://facts-engineering.github.io/) The ProductivityOpen P1AM-100 is an automation platform compatible with Productivity1000 Series I/O modules, P1AM Series shields, and Arduino MKR format shields.

* **avdweb**: https://raw.githubusercontent.com/avandalen/SAM15x15/master/package_avdweb_nl_index.json
  * [SAM 15x15](http://www.avdweb.nl/arduino/samd21/sam-15x15.html): Arduino Zero compatible board

* **avr_boot**: https://zevero.github.io/avr_boot/package_zevero_avr_boot_index.json
  * SD card bootloader for ATmega processors:
    * [List of supported microcontrollers](https://github.com/zevero/avr_boot/tree/gh-pages#supported-microcontrollers)
    * [List of supported boards](https://github.com/zevero/avr_boot/tree/gh-pages#supported-boards)

* **Azure IoT Hub (MXChip)**: https://raw.githubusercontent.com/VSChina/azureiotdevkit_tools/master/package_azureboard_index.json
  * AZ3166 v1.0
  * [Azure IoT Hub](https://microsoft.github.io/azure-iot-developer-kit/)

* **Barebones ATmega Chips (no bootloader)**: https://raw.githubusercontent.com/carlosefr/atmega/master/package_carlosefr_atmega_index.json
  * ATmega8/168/168p/328/328P (internal and external clocks)

* **Breadboard Arduino**: https://raw.githubusercontent.com/oshlab/Breadboard-Arduino/master/avr/boardsmanager/package_oshlab_breadboard_index.json
  * [GitHub - Breadboard-Arduino](https://github.com/oshlab/Breadboard-Arduino)
  * ATmega328P (8 MHz internal)

* **Canique**: https://resources.canique.com/ide/package_canique_index.json
  * [Canique MK1](https://www.canique.com/mk1)

* **chipKIT**: https://github.com/chipKIT32/chipKIT-core/raw/master/package_chipkit_index.json
  * Microchip PIC32 based boards
  * All supported chipKIT boards including UNO32, MAX32, uC32, Fubarino SD, Fubarino Mini, WF32, WiFire, etc.
  * Main chipKIT.net site http://chipkit.net/
  * chipKIT forums http://chipkit.net/forum

* **clkdiv8**: http://clkdiv8.com/download/package_clkdiv8_index.json
  * Sparrow V4 Board: [http://clkdiv8.com](https://clkdiv8.com/wiki/doku.php)

* **Cosa**: https://raw.githubusercontent.com/mikaelpatel/Cosa/master/package_cosa_index.json
  * AdaFruit ATmega32U4
  * Anarduino MiniWireless
  * Arduino Diecimila
  * Arduino Duemilanove
  * Arduino Leonardo
  * Arduino Mega 1280
  * Arduino Mega 2560
  * Arduino Micro
  * Arduino Nano
  * Arduino Pro Micro
  * Arduino Pro Mini
  * Arduino Uno
  * Breadboards: ATtinyX4, ATtinyX5, ATtinyX61, ATmega328, ATmega1284
  * ITEAD Studio IBoard
  * LilyPad Arduino
  * LilyPad Arduino USB
  * Moteino
  * Moteino Mega
  * Pinoccio Scout
  * Microduino-Core
  * Microduino-Core32u4
  * Microduino-Core+
  * PJRC Teensy 2.0
  * PJRC Teensy++ 2.0
  * Wicked Device WildFire V3

* **Cytron Technologies**: https://raw.githubusercontent.com/CytronTechnologies/Cytron-Arduino-URL/master/package_cytron_index.json
  * Cuteduino
  * CT UNO

* **DFRobot**: https://raw.githubusercontent.com/DFRobot/DFRobotDuinoBoard/master/package_dfrobot_index.json
  * Bluno M0 MainBoard(Nuc123LD4AN0)  * NUC123LD4AN0/P/PA/A
  * DFRduino M0 MainBoard(Nuc123LD4AN0) * NUC123LD4AN0/P/PA/A
  * Bluno M3 MainBoard(STM32) STM32/P/PA/A

* **DFRobotIot**: https://raw.githubusercontent.com/DFRobot/DFRobotDuinoBoard/master/package_dfrobot_iot_mainboard.json
  * DFRobot 8266 Iot Mainboard (ESP8266)

* **DFRobot ESP32 Iot**: https://git.oschina.net/dfrobot/FireBeetle-ESP32/raw/master/package_esp32_index.json
  * DFRobot ESP32 Iot Mainboard (Firebeetle ESP32 SKU: DFR0478)

* **Digistump (Official)**: http://digistump.com/package_digistump_index.json
  * Digispark (16.5 MHz)
  * Digispark Pro (16 MHz)
  * Digispark Pro (16 MHz) (32 byte buffer)
  * Digispark Pro (16 MHz) (64 byte buffer)
  * Digispark (16 MHz - No USB)
  * Digispark (8 MHz - No USB)
  * Digispark (1 MHz - No USB)
  * Digistump DigiX
  * Note: Includes driver installer for Windows installs

* **Dwengo**: http://www.dwengo.org/sites/default/files/package_dwengo.org_dwenguino_index.json
  * Dwenguino (http://www.dwengo.org/tutorials/dwenguino/dwenguino-board)

* **Electronica Elemon**: https://raw.githubusercontent.com/MauricioJancic/Elemon/master/package_Elemon_index.json
  * EESA-IOT 5.0 v1.0 more info in http://www.elemon.com.ar/NovedadesDet.aspx?Id=56

* **Elektor**
  * Elektor.Labs [support](http://www.elektor-labs.com), [main site](http://www.elektor.com)
  * Elektor Uno R4 (ATmega328PB, [project page](https://www.elektormagazine.com/labs/elektorino-uno-r4-150790)): https://github.com/ElektorLabs/Arduino/releases/download/v1.0.1/package_elektor_uno_r4_1_8_x_index.json
  * Platino Universal AVR Board (supports 28-pin and 40-pin devices), [project page](https://www.elektormagazine.com/labs/platino-versatile-board-for-avr-microcontrollers-100892-150555)) 
  * AVR Playground (dev board with many peripherals + mikroe Click slot, [project page](https://www.elektormagazine.com/labs/avr-playground-129009-2))
  * eRIC Nitro (with on-board LPRS radio module), [project page](https://www.elektormagazine.com/labs/eric-nitro-150308)):
  https://github.com/ElektorLabs/Arduino/releases/download/v1.0.1/package_elektor_index.json

* **Engimusing**: https://engimusing.github.io/arduinoIDE/package_engimusing_modules_index.json
  * Engimusing has a variety of small boards which are centered around the Silicon Labs Gecko microprocessors.
  * More information can be found at https://www.engimusing.com.
  * EFM32G232
  * EFM32TG110
  * EFM32TG222
  * EFM32WG840
  * EFM32ZG108
  * EFM32ZG222
  * EFM32ZGUSB

* **ESP8266 Community**: https://arduino.esp8266.com/stable/package_esp8266com_index.json
  * Generic ESP8266 modules
  * Olimex MOD-WIFI-ESP8266
  * NodeMCU 0.9 (ESP-12)
  * NodeMCU 1.0 (ESP-12E)
  * Adafruit HUZZAH ESP8266 (ESP-12)
  * SparkFun Thing
  * SweetPea ESP-210
  * WeMos D1
  * WeMos D1 mini

* **Espressif ESP32**: https://dl.espressif.com/dl/package_esp32_index.json
  * Many variants of ESP32 boards

* **FemtoCow**: https://raw.githubusercontent.com/FemtoCow/ATTinyCore/master/Downloads/package_femtocow_attiny_index.json
  * ATtiny84, ATtiny85, ATtiny861, ATtiny167, ATtiny2313, ATtiny88

* **ftDuino**: https://raw.githubusercontent.com/harbaum/ftduino/master/package_ftduino_index.json
  * [ftDuino fischertechnik compatible controller](http://ftduino.de)

* **Goldilocks 1284p**: https://raw.githubusercontent.com/feilipu/feilipu.github.io/master/package_goldilocks_index.json
  * Goldilocks 20MHz
  * Goldilocks 22.1184MHz
  * Goldilocks Analogue (24.576MHz, integrated MCP4822 DAC & Headphone Amp, Mic Amp for ADC, SPI SRAM & EEPROM)

* **HidnSeek**: http://hidnseek.github.io/hidnseek/package_hidnseek_boot_index.json
  * Main HidnSeek site: https://hidnseek.fr
  * An ATmega328P for IoT in low power mode with USB Battery charger, GPS, Accelerometer, Temperature, Pressure, SIGFOX RF link (w/USB boot loader)

* **In-Circuit**: http://library.radino.cc/Arduino_1_8/package_radino_radino32_index.json
  * radino32 DW1000
  * radino32 SX1272
  * radino32 CC1101
  * radino32 WiFi
  * radino32 nRF8001
  * radino CC1101
  * radino WiFi
  * radino nRF8001
  * radino RF69
  * radino RF233

* **Infineon Technologies**: https://github.com/Infineon/Assets/releases/download/current/package_infineon_index.json
  * GitHub Project: https://github.com/Infineon/XMC-for-Arduino
  * Supported Boards:
    * [XMC 2Go](https://www.infineon.com/cms/en/product/evaluation-boards/KIT_XMC_2GO_XMC1100_V1/productType.html?productType=db3a304443537c4e01436ccecb5d154f)
    * [XMC1100 Boot Kit](https://www.infineon.com/cms/en/product/evaluation-boards/KIT_XMC11_BOOT_001/productType.html?productType=db3a30443b360d0e013b8f5163c46f62#ispnTab1)
    * [XMC4700 Relax Kit](https://www.infineon.com/cms/en/product/evaluation-boards/KIT_XMC47_RELAX_LITE_V1/productType.html?productType=5546d46250cc1fdf0150f6a2788e6e89)

* **IntoRobot**: https://github.com/IntoRobot/IntoRobotPackages-ArduinoIDE/releases/download/1.0.0/package_intorobot_index.json
  * IntoRobot-Atom (STM32F103)
  * IntoRobot-Neutron (STM32F411)
  * IntoRobot-Nut (ESP8266)
  * IntoRobot-Fig (ESP32)

* **IOTEAM**: https://raw.githubusercontent.com/ioteamit/ioteam-arduino-core/master/package_ioteam_index.json
  * Dustino

* **Iteaduino Lite**: https://raw.githubusercontent.com/udif/ITEADSW_Iteaduino-Lite-HSP/master/package/package_iteaduino_lite_index.json
  * [Iteaduino Lite](https://www.itead.cc/wiki/Iteaduino_Lite) is a cheap Arduino clone (no longer sold), which is based on an ATmega88 chip clone called LGT8F88A.
  * This package is fork of the original Itead GitHub repository which was intended to be manually installed over an Arduino 1.0.x or 1.5.x and did not contain a JSON based plugin release.

* **Kristian Sloth Lauszus**: https://raw.githubusercontent.com/Lauszus/Sanguino/master/package_lauszus_sanguino_index.json
  * Sanguino

* **Konekt Dash/DashPro (Official)**: http://downloads.konekt.io/arduino/package_konekt_index.json
  * Cortex M4-based global cellular dev kits, support for all board variants, support for USB and over-the-air programming straight from within Arduino IDE
  * Main Konekt site: https://konekt.io
  * Konekt forums: https://community.konekt.io
  * Konekt GitHub: https://github.com/konektlabs
  * Tutorial: https://content.konekt.io/tutorials/hardware/konekt-dash/getting-started/

* **Laika**: https://raw.githubusercontent.com/eightdog/laika_arduino/master/IDE_Board_Manager/package_project_laika.com_index.json
  * Laika Explorer:
    * ATmega88PA

* **Lattuino**: http://fpgalibre.sf.net/Lattuino/package_lattuino_index.json
  * Lattuino is an Arduino implementation using an iCE40 FPGA
  * Is compatible with Arduino UNO, but with less memory
  * Web page: http://fpgalibre.sourceforge.net/Lattuino_en/index.html
  * FPGA board used: http://fpgalibre.sourceforge.net/Kefir_en/index.html
  * Lattuino IP and API: https://github.com/INTI-CMNB/Lattuino_IP_Core
  * Support IP cores: https://github.com/FPGALibre/fpgacores

* **LinkIt ONE (Seeed Studio)**: http://download.labs.mediatek.com/package_mtk_linkit_index.json
  * MediaTek v1.0

* **LinkIt Smart 7688 Duo (Seeed Studio)**: http://download.labs.mediatek.com/package_mtk_linkit_smart_7688_index.json
  * LinkIt Smart 7688 Duo(Atmega32U4)(3.3V)(8MHz)

* **LinkIt 7697**: http://download.labs.mediatek.com/package_mtk_linkit_7697_index.json
  * LinkIt 7697

* **Macchina**: https://macchina.cc/package_macchina_index.json
  * [Macchina M2](https://www.macchina.cc/)

* **MattairTech LLC**: https://www.mattairtech.com/software/arduino/package_MattairTech_index.json
  * MT-D21E (ATSAMD21ExxA)
  * MT-D11 (ATSAMD11D14AM)
  * MT-DB-U1 (AT90USB162)
  * MT-DB-U2 (ATmega32U2)
  * MT-DB-U4 (ATmeaga32U4)
  * MT-DB-U6 (AT90USB646/AT90USB1286)

* **Maxim Integrated**: https://raw.githubusercontent.com/MaximIntegratedMicros/arduino-collateral/master/package_maxim_index.json
  * GitHub Project: https://github.com/MaximIntegratedMicros/arduino-max326xx
  * Supported Boards:
    - [MAX32620FTHR](https://www.maximintegrated.com/en/products/microcontrollers/MAX32620FTHR.html)
    - [MAX32625MBED](https://www.maximintegrated.com/en/products/microcontrollers/MAX32625MBED.html)
    - [MAX32630FTHR](https://www.maximintegrated.com/en/products/microcontrollers/MAX32630FTHR.html)

* **MegaCore**: https://mcudude.github.io/MegaCore/package_MCUdude_MegaCore_index.json
  * ATmega2561/V
  * ATmega2560/V
  * ATmega1281/V
  * ATmega1280/V
  * ATmega640/V
  * ATmega128/L/A
  * ATmega64/L/A

* **MicroCore**: https://mcudude.github.io/MicroCore/package_MCUdude_MicroCore_index.json
  * ATtiny13

* **MightyCore**: https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json
  * ATmega1284/P
  * ATmega644/P/PA/A
  * ATmega324P/PA/A
  * ATmega164P/PA/A
  * ATmega32
  * ATmega16
  * ATmega8535

* **MiniCore**: https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json
  * ATmega328/P/PA/A
  * ATmega168/P/PA/A
  * ATmega88/P/PA/A
  * ATmega48/P/PA/A
  * ATmega8

* **Moteino (Official)**: https://lowpowerlab.github.io/MoteinoCore/package_LowPowerLab_index.json  
  * Moteino (16 MHz)
  * MoteinoMEGA (16 MHz)
  * [All about Moteino](http://lowpowerlab.com/moteino)

* **Navspark**: http://navspark.mybigcommerce.com/content/package_navspark_index.json
  * Navspark-GL: Arduino Compatible Development Board with GPS/GLONASS
  * NavSpark: Arduino Compatible Development Board with GPS

* **NeKuNeKo**: https://nekuneko.github.io/arduino-board-index/package_nekuneko_index.json
  * SoK Zero Dawn (SAMD21J18A)
  * SoK M4 Advance (SAMD51J20A)

* **NicoHood HoodLoader2**: https://raw.githubusercontent.com/NicoHood/HoodLoader2/master/package_NicoHood_HoodLoader2_index.json
  * ATmega16u2
  * ATmega32u2
  * ATmega8u2
  * AT90USB162
  * Original ATmega16u2 DFU Firmware
  * Arduino Uno HID-Bridge
  * Arduino Mega 2560 HID-Bridge

* **Nordic Semiconductor nRF5 based boards**: https://sandeepmistry.github.io/arduino-nRF5/package_nRF5_boards_index.json
  * GitHub Project: https://github.com/sandeepmistry/arduino-nRF5
  * Plain nRF52 MCU Supported Boards:
    * Nordic Semiconductor nRF52 DK
      * Shenzhen Taida Century Technology nRF52 low cost development board
      * RedBear Blend 2
      * RedBear Nano 2
    * nRF51 Supported Boards:
      * Plain nRF51 MCU
      * BBC micro:bit
      * Bluz DK
      * Nordic Semiconductor nRF51822 Development Kit + nRF51422 Development Kit
      * PCA10000
      * PCA10001, PCA10002, PCA10003, PCA10004 via nRF6310(nRFgo)
      * Nordic Semiconductor NRF51 Dongle
      * OSHChip
      * RedBearLab BLE Nano
      * RedBearLab nRF51822
      * Waveshare BLE400
      * ng-beacon
      * Seeedstudio Tinyble

* **OLIMEX**:
  * **OLIMEX (AVR BOARDS)**: https://raw.githubusercontent.com/OLIMEX/Arduino_configurations/master/AVR/package_olimex_avr_index.json
    * Examples for OLIMEX AVR Arduino-like boards (OLIMEXINO-328, OLIMEXINO-32u4, OLIMEXINO-NANO, etc)
    * Examples for OLIMEX AVR Digispark-like boards (OLIMEXINO-85, OLIMEXINO-85S, OLIMEXINO-85BC, FOSDEM-85, etc)
    * Libraries and examples for various OLIMEX shields and UEXT extension boards (MOD-LCD1x9, SHIELD-LCD16x2, MOD-IO2, etc)
  * **OLIMEX (PIC BOARDS)**: https://raw.githubusercontent.com/OLIMEX/Arduino_configurations/master/PIC/package_olimex_pic_index.json
    * Examples for PIC32-PINGUINO, PIC32-PINGUINO-OTG
    * Library and examples for OLIMEX UEXT extension board (MOD-GPS)
    * Note: Since the tools (compiler, uploader, drivers) are from chipKIT's package you have to add their *.json file (https://github.com/chipKIT32/chipKIT-core/raw/master/package_chipkit_index.json) in the list of "Additional Boards Managers URLs" (installation of chipKIT's package is not required).
  * **OLIMEX (STM BOARDS)**: https://raw.githubusercontent.com/OLIMEX/Arduino_configurations/master/STM/package_olimex_stm_index.json
    * Examples for OLIMEX STM32-E407

* **OMC**: https://raw.githubusercontent.com/ThamesValleyReprapUserGroup/Beta-TVRRUG-Mendel90/master/Added-Documents/OMC/package_omc_index.json
  * [OMC](http://www.tvrrug.org.uk/): Open Motion Controller

* **ROBOTIS**: https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

* **Open Panzer**: https://openpanzerproject.github.io/OpenPanzerBoards/package_openpanzer_index.json
  * [TCB (Tank Control Board)](https://github.com/OpenPanzerProject/TCB) with ATmega2560 processor

* **OpenTracker**: https://raw.githubusercontent.com/geolink/opentracker-arduino-board/master/package_opentracker_index.json
  * GeoLink [OpenTracker](https://geolink.io/opentracker.php)

* **panStamp**: http://panstamp.org/arduino/package_panstamp_index.json
  * [panStamp AVR](http://www.panstamp.com/product/panstamp-avr/) w/ Atmega328p
  * [panStamp NRG](http://www.panstamp.com/product/197/) w/ CC430F5137 (TI MSP430 core + CC1101)

* **Quirkbot**: https://raw.githubusercontent.com/Quirkbot/QuirkbotArduinoHardware/master/package_quirkbot.com_index.json
  * [Quirkbot](http://quirkbot.com) support

* **RFduino (Discontinued)**: http://rfduino.com/package_rfduino_index.json
  * RFduino

* **RedBear**: https://redbearlab.github.io/arduino/package_redbear_index.json
  * RedBear Duo (Cortex-M3, WiFi + BLE)
  * [RedBear discussion forum](http://discuss.redbear.cc/)

* **RedBearLab**: https://redbearlab.github.io/arduino/package_redbearlab_index.json
  * RedBearLab AVR+BLE ([ATmega32U4](http://www.atmel.com/devices/atmega32u4.aspx) + [nRF8001](https://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF8001)) Boards ([Blend](http://redbearlab.com/blend) and [Blend Micro](http://redbearlab.com/blendmicro)).

* **Riddle&Code Ltd.**: https://raw.githubusercontent.com/RiddleAndCode/RnCAtmega256RFR2/master/Board_Manager/package_rnc_index.json
  * ATmega256RFR2 Xplained Pro Evaluation Kit (Complete package including bootloader)

* **RIG by REKA**: http://rig.reka.com.my/package_rig_index.json
  * [RIG Cell Lite](https://reka.com.my/metier/artifact/rig/) Cellular Based IoT Board

* **RobotCing**: https://raw.githubusercontent.com/RobotCing/Cing/master/Software/Packages/package_RobotCing_index.json
  * ATtiny85
  * ATtiny84
  * ATmega8
  * ATmega328
  * ATmega32U4

* **Seeeduino(Seeed Studio)**: https://raw.githubusercontent.com/Seeed-Studio/Seeeduino-Boards/master/package_seeeduino_index.json
  * Seeeduino V3.0(ATmega328P)
  * Seeeduino Mega 2560
  * Seeeduino V4(ATmega328P)
  * Seeeduino Lotus
  * Seeeduino Lite

* **Simba**:
  * **Simba AVR**: https://raw.githubusercontent.com/eerimoq/simba-releases/master/arduino/avr/package_simba_avr_index.json
      * Arduino Mega
      * Arduino Nano
      * Arduino Pro Micro
      * Arduino Uno
  * **Simba ESP**: https://raw.githubusercontent.com/eerimoq/simba-releases/master/arduino/esp/package_simba_esp_index.json
      * ESP-01
      * ESP-12E
  * **Simba SAM**: https://raw.githubusercontent.com/eerimoq/simba-releases/master/arduino/sam/package_simba_sam_index.json
      * Arduino Due

* **Sipeed**:
  * **kendryte k210 (MAIX)**: http://dl.sipeed.com/MAIX/Maixduino/package_Maixduino_k210_index.json
    * MAIX One Dock
    * MAIX Bit
    * MAIX Go
  * **K210 (MAIX) CDN**: http://dl.sipeed.com/MAIX/Maixduino/package_Maixduino_k210_dl_cdn_index.json
    * CDN mirror of K210 (MAIX)

* **SODAQ**:
  * **SODAQ AVR**: http://downloads.sodaq.net/package_sodaq_index.json
    * SODAQ Mbili
    * SODAQ Tatu
  * **SODAQ SAMD**: http://downloads.sodaq.net/package_sodaq_samd_index.json
    * SODAQ Autonomo
    * SODAQ ONE
    * SODAQ ExpLoRer

* **Sony**: https://github.com/sonydevworld/spresense-arduino-compatible/releases/download/generic/package_spresense_index.json
  * Spresense

* **SparkFun**: https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json
  * AVR Boards (ATmega128RFA1 Development Board, Digital Sandbox, Fio v3, MaKey MaKey, Mega Pro (Mini) 3.3V, ProMicro 3.3V & 5V, Serial 7-Segment Display)

* **STM32 core (official)**: https://raw.githubusercontent.com/stm32duino/BoardManagerFiles/master/STM32/package_stm_index.json
  * Provide support of the following STM32 series:
    * STM32F0, STM32F1, STM32F2, STM32F3, STM32F4, STM32F7
    * STM32G0, STM32G4
    * STM32H7
    * STM32L0, STM32L1, STM32L4
    * STM32MP1 (cortex-M)
    * STM32WB
  * List of current supported boards:
    * https://github.com/stm32duino/Arduino_Core_STM32#boards-available

* **STM8**: https://github.com/tenbaht/sduino/raw/master/package_sduino_stm8_index.json
  * An Arduino-like programming API that can be used from within the Arduino IDE or for Makefile-controlled builds.
  * Using the SDCC and written in plain C instead of C++.
  * STM8F103 breakout board
  * ESP14 Wifi board
  * STM8S105Discovery board

* **STM32F1xx/STM32F4xx/STM32F3xx Series**: http://dan.drown.org/stm32duino/package_STM32duino_index.json
  * STM32 Discovery F407
  * STM32Stamp F405
  * Netduino2 F405
  * STM32F3Discovery 
  * Maple Mini
  * Maple (Rev 3)
  * Maple (RET 6)
  * Microduino Core STM32 to FLASH
  * STM Nucleo F103RB (STLink)
  * Generic STM32F103C series
  * Generic STM32F103R series
  * Generic STM32F103T series
  * Generic STM32F103V series
  * Generic STM32F103Z series         
  * Generic GD32F103C series

* **Talk²**: http://talk2arduino.wisen.com.au/master/package_talk2.wisen.com_index.json
  * [Talk² Whisper Node](https://talk2.wisen.com.au/product-talk2-whisper-node-avr/): Ultra-low power Arduino + RFM69 running on single AA battery

* **TKJ Electronics**: https://raw.githubusercontent.com/TKJElectronics/Balanduino/master/package_tkj_balanduino_index.json
  * Balanduino

* **TL7788 Kit**: https://rawgit.com/hunianhang/nufront_arduino_json/master/package_tl7788_index.json
  * TL7788 Kit v1.0.3

* **UDOO**: https://udooboard.github.io/arduino-board-package/package_udoo_index.json
  * UDOO Quad/Dual
  * UDOO Neo

* **Windows 10 IoT Core**: https://github.com/ms-iot/iot-utilities/raw/master/IotCoreAppDeployment/ArduinoIde/package_iotcore_ide-1.6.6_index.json
  * [Windows 10 IoT Core](http://www.windowsondevices.com/)
    * Raspberry Pi 2 & 3
    * Minnowboard MAX

* **wirino**: https://per1234.github.io/wirino/package_per1234_wirino_index.json
  * GitHub repository: https://github.com/per1234/wirino
  * Boards:
    * Wiring S
    * Wiring S with Play Shield
    * Wiring V1.0
    * Wiring Mini V1.0
    * Wiring V1.1 ATmega1281
    * Wiring V1.1 ATmega2561
  * Dependencies:
    * MightyCore: https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json
    * MegaCore: https://mcudude.github.io/MegaCore/package_MCUdude_MegaCore_index.json

* **XMegaForArduino project**: https://github.com/XMegaForArduino/IDE/raw/master/package_XMegaForArduino_index.json
  * Github project:  http://github.com/XMegaForArduino
  * Support for several XMEGA processors (some may require compiler patches), more to come
  * Basic 'pins_arduino.h' files for 'standard' layouts, described in the file
  * Intended to be the basis for your own custom XMEGA-based board
  * Supports dedicated pins for hardware serial flow control
  * Implements a low-current 'wait' state for delay and wait for interrupt
  * bootloaders for 'arduino' and 'wiring' protocol.
  * USB support 'under development' (for ATxmega128A1U, others)
  * Provides new libraries for SPI, SD, 2-wire so you can start using these right away
  * Includes patch files for compilers and tools (source patch), and instructions on how to modify earlier IDE versions
  * Leverages XMEGA's internal 32 MHz clock in the startup code (no external crystal or resonator required)
  * Advanced interrupt handling support (basically ANY pin can be an interrupt).
  * Advanced pin settings include pull up, pull down, on both input and output
  * Project is still in a 'beta' state, but is fully functional for nearly all practical uses, and as 'Arduino compatible' as could be made possible.

* **Zoubworld core**: http://zoubworld.com/~zoubworld_Arduino/files/Release/package_Zoubworld_index.json
  * based on ATSAMC21/ATSAMC20, ATSAMD21, ATSAML22, ATSAML21, ATSAME54 32 bit MCU core (ARM core CM0+ and CM4)
  * Zoubworld Core (http://zoubworld.com/~zoubworld_Arduino and https://github.com/zoubata/ArduinoCore-samd)
    * Pilo: extension I/O board for Raspberry Pi
    * Captor: captor board for robot (Ultrasonic Sensor - HC-SR04, Sharp GP2Y*, motor coder, Photoelectric Sensors Infrared)
    * Line: line detector for robot

* **EBot Arduino core**: https://raw.githubusercontent.com/sanu-krishnan/ebot-arduino-core/master/package_ebots.cc_index.json
  * based on Atmega1284p

* **AmbaSat-1 Satellite**: https://ambasat.com/boards/package_ambasat-1.com_index.json
  * [AmbaSat-1 Space Satellite](https://ambasat.com/): The Low Earth Orbit Space Satellite Development Kit