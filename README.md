# XDF-001

## Introduction

### What is XDF-001?

The **XDF-001**, or **Experimental Drone Fighter Model 001**, is a flight platform based on the P-51 Mustang airframe. Its experimental objective is to achieve a multi-role aircraft capable of both aerobatic flight and payload transport. Additionally, the XDF-001 platform seeks to demonstrate the viability of implementing simple autonomous capabilities in RC drones using vector operations, alongside data visualization via the **XDF-001 Control** application.

### Motivation

The motivation for this project stems from an interest in military aviation and aeronautics, as well as addressing the economic barriers within the RC aircraft community. Developing an aircraft within the ESP32 ecosystem helps overcome financial constraints, enabling the transition beyond typical 8-inch drones to larger-scale aircraft at equally competitive prices.

Another key motivation is the code's iterative potential; it can be adapted and modified for a myriad of missions and other types of aircraft and vehicles, allowing for extensive iteration and versatility.

### Objectives

The primary objective is to develop a fixed-wing platform based on the ESP32 ecosystem, capable of both radio control and simple autonomous maneuvers such as waypoint navigation or orbiting.

The secondary objectives are:

- Development of a telemetry visualization and control application for the flight platform.
- Study of various aerodynamic configurations to maximize the stability-to-top-speed ratio.
- Study of ESP32 behavior in handling real-time vector operations and autonomous capabilities.
- Development of a payload bay within the platform for payload transport.

### Architecture

As previously mentioned, the project is based on the ESP32 ecosystem and programmed using the Arduino IDE. However, communication between the ground ESP32 and the PC utilizes a binary system, enabling both command transmission and telemetry reception via USB serial. For a comprehensive study of the firmware architecture, please refer to the **[Documentation](Documentation/)** folder: check the diagrams section for data flows or the PDF section for function documentation.

## Project Parts

Below is a brief description of the repository's main folders and their contents:

### [CAD](CAD/)

Contains all CAD modeling files, including the root file modeled in FreeCAD and `.stp` files for the aircraft and PCB, as well as 3D models for the modules used.

### [Firmware](Firmware/)

Contains all ESP32 code, divided into air and ground sections. This includes communication functions, vector calculations, etc.

### [PCB](PCB/)

Contains all electronics-related files, including schematics, `.pcb` files, and manufacturing Gerbers.

### [XDF-001 Control](XDF-001_Control/)

Dedicated strictly to the visualization and control application. Contains all backend and frontend code.

### [References](References/)

Contains all structural references and concepts adopted from various creators for the project's construction.

### [Documentation](Documentation/)

Contains flowcharts and a detailed PDF explaining the application code, autonomous functions, and vector-based calculations.

### [Construction Manual](Construction%20Manual/)

PDF file detailing the aircraft construction method and key considerations for assembly.

## Changelog

### V0.0.0

Base project files have been exported, including code prototypes for both the ground and air sections. This release also includes the prototype for the control and telemetry application, as well as the PCB rendering, schematics, and CAD files.

## Roadmap

### V0.1.0

Completion of integration with Excel and SQL databases for the control and telemetry application.
Theoretical completion of the ground and air section code, enabling remote flight capabilities without telemetry and video visualization.

### V0.2.0

Achievement of initial flight tests, featuring full integration between the ground section and the control and telemetry application.
Initial implementation of autonomous functions, such as navigating to a designated point at a predefined altitude.

## FAQ

### What is the cost of the project?

The cost of the project varies depending on the dimensions of the flight platform and the components selected. For instance, opting to exclude the camera or telemetry functions can result in a cost of approximately $30. Conversely, selecting premium components can increase the cost to the order of $200 or more.

### Can I adapt this project?

Yes, under the license provided, I grant permission for the project to be modified to suit individual needs. In fact, adaptability is one of the core objectives of this project.

### What types of peripherals can I use to control the drone?

Any peripheral is acceptable provided it can be read by the ESP32 microcontroller. In this instance, a console controller was used, but joysticks, keyboards, mice, or any HID device are also compatible.

### How is the visualization and telemetry application used?

#### User Mode (Run Executable)

To use the application as a final user:

**Option A: Installer (Recommended)**

1. Navigate to `XDF-001_Control/out/make/squirrel.windows/x64/`.
2. Run `xdf-001_control-1.0.0 Setup.exe`.
3. The app will install and open automatically.

**Option B: Portable (Direct Run)**

1. Navigate to `XDF-001_Control/out/xdf-001_control-win32-x64/`.
2. Run `xdf-001_control.exe` directly.
3. No installation required. (You can also share the `out/make/zip` file).

#### Developer Mode (Run from Source)

If you want to modify the code or debug:

1. **Prerequisites**: Ensure [Node.js](https://nodejs.org/) (LTS) is installed.
2. Open a terminal in `XDF-001_Control`.
3. Install dependencies: `npm install`.
4. Start the app: `npm start`.

#### Build Mode (Create Executable)

To generate the `.exe` file yourself:

1. Follow the **Developer Mode** steps to set up the environment.
2. Run the build command: `npm run make`.
3. The output files will be created in the `out/` folder.
