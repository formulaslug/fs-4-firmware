Port of OneWireSTM Library to Mbed OS
=====================================

This library is a port of OneWireSTM library from [rocerclarkmelbourne/Arduino_STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32).

## Why?

I wanted to use DS18B20 on mbed-os, but existing libraries did not work for me. I used to work with OneWireSTM using STM32F103 and Arduino framework, so I ported that library to Mbed OS.

## How did it go?

I used this library with Mbed OS 6.10 running on STM32 NUCLEO-F446RE development board and it is working perfectly.

## How to use?

Import this library using mbed-cli then compile the program.

```sh
mbed import https://github.com/alwint3r/onewire-mbed-os
```

See this [usage example with DS18B20](https://github.com/alwint3r/ds18b20-example-onewire-mbed-os).
