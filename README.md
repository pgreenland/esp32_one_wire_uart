# ESP-32 1-Wire Host Implementation using UART

This repository contains an implementation of the 1-Wire host protocol, using a hardware UART within the ESP-32 family of devices.

Unlike bit-bashed designs, making use of a hardware UART ensures accurate timing is maintained. While reducing CPU load on the host system, allowing other work to progress while bytes are transferred by the UART hardware in the background.

For a usage example see the following companion repository, containing a driver for the Analog / Dallas DS18B20 temperature sensor:

https://github.com/pgreenland/esp32_ds18b20

Or example application which makes use of the repository above and this one to communicate with one or more temperature sensors:

https://github.com/pgreenland/esp32_ds18b20_example

For more information see my associated blog post: https://www.quantulum.co.uk/blog/1-wire-with-uart-on-esp-32
