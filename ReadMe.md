# ESP32 USB Soft Host library for Arduino IDE


What is it?
-----------

This is mainly a wrapper around the excellent work of [Dmitry Samsonov (@sdima1357)](https://github.com/sdima1357)
with [esp32_usb_soft_host](https://github.com/sdima1357/esp32_usb_soft_host)

Some parts of the code (e.g. the timer group isr calls) have been regressed to fit esp-idf 3.3 needs.

USB Low Speed is slow
---------------------

ESP32 USB-LS Soft host (:warning: LS=low speed) is a pure software implementation of USB host thru general IO pins.


Features
--------

The library supports up to 4 HID *low speed* devices simultaneously.
A callback can be attached both for detection and events.


Hardware support
----------------

This library has been tested on ESP32-Wroom/Wrover and works fine as long as the pins aren't
shared (beware of the psram pins!).
It remains untested (but useless) on ESP32-S2, and can't compile (yet?) on ESP32-C3.


Known working HID devices:
--------------------------

- USB Keyboards: any, but consider powering externally it has backlights and/or other gadgets
- USB Mice:
  - LogiLink ID0062 (idVendor=0x1a2c, idProduct=0x0042): works every time
  - Logitech B100 (idVendor=0x046d, idProduct=0xc077): partial detection, may need several resets before events are fired



Proof of concept:
-----------------

[![](extras/ESP32-USB-host.png)]
[![](extras/m5stack-pins_16_17.jpeg)]

Credits:
--------
- [Dmitry Samsonov ](https://github.com/sdima1357)
- [Grigory Prigodin](https://github.com/zbx-sadman)
- [@ivo1981](https://github.com/ivo1981)
