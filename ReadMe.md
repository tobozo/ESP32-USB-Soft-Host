
# ESP32 USB Soft Host library for Arduino IDE

This is mainly a wrapper around the excellent work of [sdima1357](https://github.com/sdima1357) with [esp32_usb_soft_host](https://github.com/sdima1357/esp32_usb_soft_host)

Some parts of the code (e.g. the timer group isr calls) have been regressed to fit esp-idf 3.3 needs.


esp32 USB-LS is a pure software implementation of USB host thru general IO pins.

It supports up to 4 HID devices simultaneously.

Works on ESP32-Wroom an Wrover, untested yet on S2, can't compile on C3.
