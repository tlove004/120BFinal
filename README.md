# CS120B Final Project

## Automatic Coffee Roaster

By Tyson Loveless!

## Overview:

This project aimed to overcome the hands-on process of roasting coffee in an air popcorn popper through temperature monitoring and PID control of the fan and heating elements. The resulting coffee roaster works by reading temperature inputs in two locations (one representing the heating element temperature and the other the temperature of the mass of beans), setting a temperature goal based on pre-defined roast profiles, and updating a PID output in order to modulate the heating element.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/CRpMMAHqQ1g/0.jpg)](https://www.youtube.com/watch?v=CRpMMAHqQ1g)

See it in action!

## User Guide:

In order to operate the machine, power must be supplied to the breadboard and the roaster must be plugged in to 120 VAC. A switch on the front must be flipped to turn the machine on, which automatically activates the fan to 100% power. This ensures that the heating element will never be on by itself, potentially causing a fire. Once the breadboard and roaster are powered and on, the user interface is displayed on the LCD screen. The button allows different pre-programmed roast profiles to be selected, while the switch starts the roast. Once the switch is flipped, no more user interaction is required; however, if a user wants to stop a roast short, the switch can be flipped off, at which point the machine will enter the cooling stage early.

### A roast progresses as follows:

- First Ramp: The first temperature in the profile is reached within 2 minutes of activation
- First Shelf: The temperature rises 10-20 degrees over the next 2 minutes, ensuring momentum is not lost.
- Second Ramp: The next temperature spike in the profile is reached over the next 4 minutes.
- Second Shelf: Similar to first shelf, but over a 4 minute span.
- Cooling: Once the second shelf set point is reached (or a user terminates the roast early), the machine enters a stage in which the heating element is completely off, cooling at least until the bean mass is less than 100 degrees fahrenheit.

## Technology/Components Used:

- 2 K-type thermocouples
- 1 Dual channel MAX31855 breakout interfaced over SPI (for reading temperature from thermocouples)
- 1 SN74HC128 3-8 line inverting demultiplexer - for decoding SPI child selection
- AVR182 spec zero-cross detection - used to warn of AC current (could be used for phase angle control of fan)
- Proportional-integral-derivative (PID) software controller using customized open source pid library.
- PWM of 25A DC to AC zero cross solid state relay (UXCELL SSR-25 DA)
- Nokia 5110 LCD screen for user interface (displays roast progression and current temperatures)

## Known Issues:
## License:
