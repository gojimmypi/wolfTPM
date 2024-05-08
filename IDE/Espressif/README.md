# wolfTPM for Espressif

Initial minimum memory requirements: 35KB Stack. See `sdkconfig.defaults`.

## TODO

Currently the `components\wolftpm\include\options.h` is not found by the respective component `CMakeLists.txt`, 
so it is copied to `C:\workspace\wolfTPM-gojimmypi\wolftpm` as an interim solution.

The `wrap_test.h` is currently _copied_ to the example. Consider pointing to original repo source.

```
./configure --enable-infineon --enable-i2c --with-wolfcrypt=[path]
```

## Troubleshooting

If problems are encountered with the I2C module:

- Beware that printing to the UART during an I2C transaction may affect timing and cause errors.
- Ensure the TPM module has been reset after flash updated.
- Check wiring. `SCL` to `SCL`, `SDA` to `SDA`. Probably best to ensure GND is connected. Vcc is 3.3v only.
- Ensure the proper pins are connected on the ESP32. SCL default is `GPIO 19`;  SDA default is `GPIO 18`.
- Test with only a single I2C device before testing concurrent with other I2C boards.
- When using multiple I2C boards, check for appropriate pullups. See data sheet.
- Reset TPM device again. Press button on TPM SLB9673 eval board or set TPM pin 17 as appropriate.
- 