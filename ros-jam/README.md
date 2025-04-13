# Arduino Command Handler

## Description
This project allows you to send commands to an MC (Microcontroller) over serial communication. The Arduino processes these commands as JSON messages and executes corresponding callback functions. The system is designed to be flexible, enabling custom command handlers to be added at runtime. Commands are registered on the MC side, and the appropriate function is executed when a command is received.

## Features
- Handle custom commands sent via serial communication
- JSON-based interface using `ArduinoJson` library
- Dynamically register commands and their associated callback functions
- Easily extendable with new commands and callbacks

## Installation

1. **Install VSCode and PlatformIO**:
   - Make sure you have **[VSCode](https://code.visualstudio.com/)** installed on your computer.
   - Install the **[PlatformIO IDE extension](https://platformio.org/platformio-ide)** for VSCode.

2. **Clone the Repository**:
   - Clone this project to your local machine using Git or download it as a ZIP file from GitHub.

3. **Open the Project in VSCode**:
   - Open the folder containing the cloned project in **VSCode**.



## Usage

### Arduino Setup

Users need to:

1. **Define their custom functions** (e.g., `motorStart()`, `sensorRead()`).
2. **Override `initCustomSetup()`** to add custom hardware setup, calling the base version of the function using `::initCustomSetup()` first (Note: write `::initCustomSetup()` at the top of your `initCustomSetup()` simply for the `Serial.begin(9600)` line that I included in my `setup()`. If you want to use a different baud rate, just write in `Serial.begin(...)` at the top of your `initCustomSetup()` and do NOT write `::initCustomSetup()` at the top.




### Setting Up Each Function

1. **Register Custom Setup**: In the `setup()` function function of the `main.cpp` code, `initCustomSetup()` is called for flexibility and to allow members to define their own setups. 

Note: You must define your own `initCustomSetup()` function that would simply have a similar structure:

```cpp
// Users will define their custom setup function like this:

void initCustomSetup() {
  // Call the base version of initCustomSetup to retain default setup
  ::initCustomSetup();  // This ensures the default setup code is executed first
  
  // Add custom pin setups or sensor initializations
  pinMode(5, OUTPUT);   // Custom pin setup for an LED or motor
  pinMode(A0, INPUT);   // Custom analog input pin setup for a sensor

  // Any additional hardware or sensor setup code can go here
  Serial.println("Custom setup complete");
}

```


2. **Register Custom Commands**: In the `setup()` function of the main.cpp code, `initCustomCommands()` is called to register commands using `registerCommand()`. This allows you to link each command to its respective function.

   Example:
   ```cpp
   void initCustomCommands() {
     registerCommand("ledOn", ledOn); // Register command to turn LED on
     registerCommand("motorStart", motorStart); // Register command to start motor
   }