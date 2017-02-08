# Jointmaster
Jointmaster is used to create finger joints/ mortise tenon connections in wood. This repository contains source code for the STM32F407VG discovery board used in the Jointmaster application.  For more information see http://jointmaster.eu/.

The source code enables you to adress up to 4 step-motors, 8 stop-switches and 4 encoders. Either via USB or Bluetooth.
It is intended to be used in conjunction with a STM32F407VG discovery board + a Jointmaster/ Toothmaster compliant PCB. For more information [click here](http://jointmaster.eu/2017/01/toothmaster-solution-introduction/).
This project sourcecode communicates flawlessly with Toothmaster open source that can be found [here](https://github.com/patricksevat/Toothmaster) 

![Toothmaster overview](http://jointmaster.eu/wp-content/uploads/2017/01/Toothmaster-solution-1100x599.jpg)

If you prefer a Windows based USB solution for creating your Mortise-Tenon joints please consider [Jointmaster Pro](http://jointmaster.eu/jointmaster-pro/)

If you prefer the most simple and reliable finger jointing solution for both Windows and Android without STM32 board, consider [Jointmaster audio] (http://jointmaster.eu/jointmaster-audio/)

**This Github repository is intended for developers wishing to improve or modify Toothmaster/ use Toothmaster with STM32 in a different application. Of course you can contact us if you need help.**
**If you're looking to install the app, please visit the [Google Play store](https://play.google.com/store/search?q=Toothmaster&c=apps&hl=en)**

We made use (and modified, thank you Tilen !) the standard peripheral libraries from Tilen Majerle that can be found here in case you need other (https://stm32f4-discovery.net/2014/05/all-stm32f429-libraries-at-one-place/)

## Reliability + Availability
The source code did go through an enourmous testing phase. We had it running for one week using [Jointmaster Pro](http://jointmaster.eu/jointmaster-pro/). For this reason the STM32 side is extremely stable.

## Safety
The step motors stop if a stopswitch is hit or if too much encoder steps are missed. 
We do not guarantee anything if you use this software because the STM32 is not fit for safety critical applications. For example there is always a non-quantifiable chance that the step motor continues running when a stopswitch is hit. 

## Requirements
- You need a programming and debugging environment. For instance the one we use for [follow these steps](http://bartteunissen.com/blog/programming-and-debugging-a-stm32f0-discovery-with-eclipse/)
- You need the STM32 standard peripheral library that can be found [here](http://www.st.com/en/embedded-software/stsw-stm32065.html) Do not migrate to the stm32cube.

## Installation
We use file defines.h to decide how to compile. 
You will have to tailor the standard application library to fit the STM32F407VG discovery. The license for the SMT32 standard library is personal. We cannot provide it here.
You can hire us to set up a working environment if necessary. 

## Contributing

Pull requests are most welcome.

Contributions on these sections are particularly welcome:
- Beta spline processing capability. The idea is to work with beta splines to command the step motor's (instead of small line segments). When processing so-called G-code the step motors start and stop over very short distances. We would like to have a pre-processor on the Toothmaster side that decides whether a spline offers advantage over a lot of small line segments. Mostly you draw some 3D object that can be matched using beta-spline. Next this 3D object is translated into millions of small line segments by a Gcode processor. The small line segments are causing (using the current source code) too much communication and step motor starting and stopping.
    
## PCB communication

Jointmaster/ Toothmaster communicates using serial communication over Bluetooth.
The PCB can be ordered [here](http://jointmaster.eu/product/jointmaster-usb-solution-low-budget-kit/) If you order you also get a variety of pre-compiled programs for the STM-32.    
The PCB flashed with software has its own commands: [here](https://github.com/patricksevat/Toothmaster/blob/master/PCB-communication.md)

## Contact

Feel free to open an issue if anything is unclear.

You can also contact [fill in this contact form](http://jointmaster.eu/contact/) for any questions regarding the required hardware or the PCB software.

For more general information see http://jointmaster.eu/. This website also contains useful blogs and videos on using Jointmaster/ Toothmaster. 

## License

All software is published under GNU GPL v3 license. This means that you are allow to modify, share and use my source and other stuff in personal or commercial use. If you modify source code, it has to stay under GNU GPL v3 license too. I reserve a right to shut down this website at any time.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
 
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
