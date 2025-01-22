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

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA            | SCL            |
| ---------------- | -------------- | -------------- |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| TPM2 Device      | SDA            | SCL            |

For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Example Configuration` in `menuconfig`.

**Note:** There's no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

See [Optiga TPM SLB 9673 Raspberry Pi I2C Hat](https://www.infineon.com/dgdl/Infineon-OPTIGA_TPM_SLB_9673_RPi-DataSheet-v01_02-EN.pdf?fileId=8ac78c8c8779172a0187ed7465fa19e8)

### SPI Pin Assignments

There are typically 4 SPI peripherals available on the ESP32: `SPI0` (used as a buffer for accessing external memory), `SPI1`, `SPI2` (HSPI), and `SPI3` (VSPI).
Controllers SPI1~SPI3 share two DMA channels. See [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#spi).

See [SPI Master Driver docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#spi-master-driver):

> SPI1 is not a GP-SPI. SPI Master driver also supports SPI1 but with quite a few limitations, see [Notes on Using the SPI Master Driver on SPI1 Bus](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#spi-master-on-spi1-bus).

|                | MOSI        | MISO        | CLK        | CS        | Vcc 3.3v  | GND    |
| -------------- | ----------- | ----------- | ---------- | --------- |---------- | ------ |
| ESP I2C Master | MOSI Pin 11 | MICO Pin 13 | CLK Pin 12 | CS Pin 10 |
| SPI RPi HAT    | MOSI Pin 19 | MISO Pin 21 | CLK Pin 23 |           | 3v3 Pin 1 | Pin 25 |

See [Optiga TPM SLB 967s Raspberry Pi SPI Hat](https://www.infineon.com/dgdl/Infineon-OPTIGA%20TPM%20SLB%209672%20FW15-DataSheet-v01_02-EN.pdf?fileId=8ac78c8c850f4bee01852eeaeb200bc8)



## Troubleshooting

If problems are encountered with the I2C module:

- Ensure the TPM module is reset at boot time (briefly bring TPM Module RST low).
- Beware that printing to the UART during an I2C transaction may affect timing and cause errors.
- Ensure the TPM module has been reset after flash updated.
- Check wiring. `SCL` to `SCL`, `SDA` to `SDA`. Probably best to ensure GND is connected. Vcc is 3.3v only.
- Ensure the proper pins are connected on the ESP32. SCL default is `GPIO 19`;  SDA default is `GPIO 18`.
- Test with only a single I2C device before testing concurrent with other I2C boards.
- When using multiple I2C boards, check for appropriate pullups. See data sheet.
- Reset TPM device again. Press button on TPM SLB9673 eval board or set TPM pin 17 as appropriate.
