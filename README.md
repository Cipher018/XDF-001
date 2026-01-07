# XDF-001

---

## Introduction

### Objectives

The primary objective of this project is to develop an experimental drone based on the ESP32 ecosystem architecture. This involves utilizing a radio frequency communication protocol and integrating an open-source application for real-time telemetry and video visualization, with capabilities to interface with Excel or SQL databases. Furthermore, the project aims to test the implementation of autonomous functions, ranging from simple maneuvers like flight path planning and point orbiting to more complex actions such as payload release, utilizing vector and geometric operations.

### Motivation

The core motivation for this project arises from a passion for military aviation and a commitment to addressing the economic barriers prevalent in modern aeromodelling. Developing a drone within the ESP32 ecosystem facilitates the iteration of this architecture across diverse projects. Although this specific iteration incurs a higher cost due to the platform's dimensions, subsequent iterations focused on micro-drones or smaller scale vehicles would be significantly more cost-effective.

Another key motivation is the versatility of the drone for various applications, including surveillance, package delivery, and aerobatic exhibitions, leveraging its aerodynamic and high-performance design.

### Architecture

The architecture of this project is founded on the ESP32 ecosystem, employing a radio frequency communication protocol and an open-source software application that enables real-time visualization of telemetry and video. It includes the capability to link with Excel or SQL databases. This facilitates a myriad of applications, from video recording and recreational flying to atmospheric data collection and air quality monitoring. The ESP32's connectivity allows it to interface with various devices, such as another ESP32 measuring air quality variables, and transmit this data to the ground station.

In its simplest form, this project consists of a ground segment, which receives telemetry and transmits commands to the drone, and an air segment, which reads the telemetry and executes the commands sent from the ground.

---

## Project Parts

### [CAD](CAD/)

Contains all 3D models and design files for the mechanical structure.

### [Construction Manual](Construction%20Manual/)

Step-by-step guides and instructions for assembling the drone hardware.

### [Firmware](Firmware/)

Source code for the ESP32 microcontrollers, including flight control logic.

### [PCB](PCB/)

Schematics, board layouts, and manufacturing files for the custom electronics.

### [References](References/)

Datasheets, technical papers, and external resources used in the project.

### [XDF-001_Control](XDF-001_Control/)

Control software and telemetry dashboard interface.

---

## Changelog

### V0.0

Base project files have been exported, including code prototypes for both the ground and air sections. This release also includes the prototype for the control and telemetry application, as well as the PCB rendering, schematics, and CAD files.

---

## Roadmap

### V0

Base project files have been exported, including code prototypes for both the ground and air sections. This release also includes the prototype for the control and telemetry application, as well as the PCB rendering, schematics, and CAD files.

### V0.1

Completion of integration with Excel and SQL databases for the control and telemetry application.
Theoretical completion of the ground and air section code, enabling remote flight capabilities without telemetry and video visualization.

### V0.2

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
