# wolfTPM for Espressif

Initial minimum memory requirements: 35KB Stack. See `sdkconfig.defaults`.

Current memory assigned: 50960

## TODO

Currently the `components\wolftpm\include\options.h` is not found by the respective component `CMakeLists.txt`,
so it is copied to `C:\workspace\wolfTPM-gojimmypi\wolftpm` as an interim solution.

The `wrap_test.h` is currently _copied_ to the example and not used. Consider pointing to original repo source.

The `native_test.h` is currently _copied_ to the example and not used. Consider pointing to original repo source.

```
./configure --enable-infineon --enable-i2c --with-wolfcrypt=[path]
```

## Pin assignments

### I2C Pin Assignments

Set I2C in Example Configuration `SCL GPIO NUM` and `SDA GPIO NUM`. Although other pins may be used, the recommended values for the ESP32 are:

- `GPIO 21` = `I2C SDA`
- `GPIO 22` = `I2C SCL`

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA            | SCL            |
| ---------------- | -------------- | -------------- |
| Wire Color       | White          | Gray           |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| ESP32 alt        | GPIO 18        | GPIO 19        |
| ESP32            | GPIO 21        | GPIO 22        |
| TPM2 Device      | SDA            | SCL            |

For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Example Configuration` in `menuconfig`.

**Note:** There's no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

See [Optiga TPM SLB 9673 Raspberry Pi I2C Hat](https://www.infineon.com/dgdl/Infineon-OPTIGA_TPM_SLB_9673_RPi-DataSheet-v01_02-EN.pdf?fileId=8ac78c8c8779172a0187ed7465fa19e8)

### SPI Pin Assignments

There are typically 4 SPI peripherals available on the ESP32: `SPI0` (used as a buffer for accessing external memory), `SPI1`, `SPI2` (HSPI), and `SPI3` (VSPI).
Controllers SPI1~SPI3 share two DMA channels. See [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#spi).

See [SPI Master Driver docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#spi-master-driver):

> SPI1 is not a GP-SPI. SPI Master driver also supports SPI1 but with quite a few limitations, see [Notes on Using the SPI Master Driver on SPI1 Bus](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#spi-master-on-spi1-bus).

|                | MOSI        | MISO        | CLK        | CS            | Vcc 3.3v  | GND    |
| -------------- | ----------- | ----------- | ---------- | ------------- |---------- | ------ |
| Wire color     | Gray        | White       | Blue       | Brown         | Red       | Black  |
| ESP32-S3 name  | FSPID       | FSPIQ       | FSPICLK    | FSPICS0       | 3v3       | GND    |
| ESP32-S3       | MOSI Pin 11 | MISO Pin 13 | CLK Pin 12 | CS Pin 10     |           |        |
| ESP32 name     | VSPI MOSI   | VSPI MISO   | VPI CLK    | VSPI CS       |           |        |
| ESP32          | MOSI Pin 23 | MISO Pin 19 | CLK Pin 18 | CS Pin 5      |           |        |
| SPI RPi HAT    | Header 19   | Header 21   | Header 23  | Header 26     | 3v3 Pin 1 | Pin 25 |
| Optiga Chip    | MOSI Pin 21 | MISO Pin 24 | CLK Pin 19 | CE1 (CS/TEST) |           |        |

NOTE: The `CE0` pin on the Optiga RPi hat is noted with the respective `R6` resistor "n.p." (this typically means "not populated").
With no connection between `CE0` and pin 20 of IC2 (`CS#`/`TEST#`) it is recommended to use `CE1` for `CS` (chip select).

NOTE: The SPI GPIO Pin 13 is also used by the JTAG programmer / debugger `TCK` / `CLK` / `SCL`.

See [Optiga TPM SLB 967s Raspberry Pi SPI Hat](https://www.infineon.com/dgdl/Infineon-OPTIGA%20TPM%20SLB%209672%20FW15-DataSheet-v01_02-EN.pdf?fileId=8ac78c8c850f4bee01852eeaeb200bc8)



## Troubleshooting

If problems are encountered with the I2C module:

- Check power requirements. ESP32 dev boards typically do not have enough on-board power for additional peripherals.
- Ensure the TPM module is reset at boot time (briefly bring TPM Module RST low).
- Beware that printing to the UART during an I2C transaction may affect timing and cause errors.
- Ensure the TPM module has been reset after flash updated.
- Check wiring. `SCL` to `SCL`, `SDA` to `SDA`. Probably best to ensure GND is connected. Vcc is 3.3v only.
- Ensure the proper pins are connected on the ESP32. SCL default is `GPIO 19`;  SDA default is `GPIO 18`.
- Test with only a single I2C device before testing concurrent with other I2C boards.
- When using multiple I2C boards, check for appropriate pullups. See data sheet.
- Reset TPM device again. Press button on TPM SLB9673 eval board or set TPM pin 17 as appropriate.
