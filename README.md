> [!CAUTION]
> **DEPRECATED**: This repository is archived and no longer maintained. It has been superseded by a new, Firmata-based implementation.
> 
> Use [**Hermes-Five - IoT & Robotics platform**](https://github.com/dclause/hermes-five) for developer-focused, hardware-agnostic board control API, or [**Hermes-Studio**](https://github.com/dclause/hermes-studio) for an all-in-one, UI-driven solution.

# HermesClient

HermesClient implements the Hermes communication protocol between an Hermes backend implementation and a compatible
board.
Communication is done through one of the available protocols (Serial, Bluetooth, Ethernet, WIFI, etc...). Once done, the
device becomes remotely controllable from backend code or from a compatible UI.

## Installation

There is two ways to use Hermes client: either as a library, or as a standalone program.

### Standalone program

#### Using ArduinoIDE 2.x

* Open `arduino/arduino.ino` file in Arduino IDE 2.x
* Upload to the board as usual

**_NOTE: arduino.ino is a symlink to src/main.cpp. If for some reason your IDE or OS does not recognize it as such, 
replace the file with an actual copy of `src/main.cpp`._**

#### Using PlatformIO

* Install platformIO in your favorite IDE. It should be available as an extension. For instance: 
  * On VSCode, use https://platformio.org/install/ide?install=vscode
  * On CLion, use https://www.jetbrains.com/help/clion/platformio.html
* Open this directory as a platformIO project
* Eventually modify platformio.ini file as necessary
* * Upload `main.cpp` to the board

### Installation as a library

#### Using ArduinoIDE 2.x

* Open Arduino IDE 2.x
    * _option 1:_ Move the whole directory in your Arduino _libraries_ folder.
    * _option 2:_ Create a symlink to this folder in your Arduino _libraries_ folder.
    * _option 3:_ ZIP this whole directory and use [Arduino IDE](https://www.arduino.cc/en/software) to import
      it: `Sketch` > `Include a Library` > `ADD .ZIP Library`
* Open the `Samples` > `HermesClient` > `Simple.ino` file in [Arduino IDE](https://www.arduino.cc/en/software)
* Upload to the board as usual

**_NOTE: Don't worry about Arduino IDE
not capable of showing related files in inner folder, it will be compiled and uploaded appropriately._**

## Compatibility

The code is written to be compatible with all Arduino boards. **It has only been tested on _Arduino NANO_, _Arduino UNO_
and _Arduino
MEGA_**.

## Usage

The code implements the HERMES protocol and is meant to be used in conjunction with _HermesBackend_. Any other
third-party system is not tested and supported.

**It does not do anything on its own. Control is done by [backend master code](https://github.com/dclause/HermesBackend--rust).**

## Developers

The project is handled via CLion IDE but any IDE of your own convenience is good.

- Code style and formatting is handled by the `.clang-tidy` and `.clang-format` files.
- CLion specific formatting overrides are to be imported from the `CLionSettings.xml` file.
- Compilation and debug via IDE needs the [PlatformIO support](https://www.jetbrains.com/help/clion/platformio.html)
  plugin
- Feel free to [submit PR](https://github.com/dclause/HermesClient--arduino) if needed.

### /!\ Information about code structure

The code is written within .h files, which is uncommon for C++ code. The reason is the Arduino IDE is not
capable to handle inclusion of cpp outside the main folder and/or residing in specific lib structure. I wanted to
keep compatibility with the Arduino IDE to handle code compiling and upload made easy - but also wanted to organize my
code in folders - hence the decision.
