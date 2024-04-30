# wolfTPM for Espressif

Initial minimum memory requirements: 35KB Stack. See `sdkconfig.defaults`.

## TODO

Currently the `components\wolftpm\include\options.h` is not found by the respective component `CMakeLists.txt`, 
so it is copied to `C:\workspace\wolfTPM-gojimmypi\wolftpm` as an interim solution.

The `wrap_test.h` is currently _copied_ to the example. Consider pointing to original repo source.

```
./configure --enable-infineon --enable-i2c --with-wolfcrypt=[path]
```
