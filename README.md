# Drone Technology and Application SS2025

Unmanned Aerial Vehicles (UAVs), commonly known as drones, are increasingly used in research, industry, and education due to their versatility in tasks such as aerial mapping, inspection, and autonomous navigation. In the academic setting, building and programming drones offers students an opportunity to bridge theoretical knowledge with hands-on engineering practice.

As part of our course "Drone Technology and Application", we undertook the challenge of assembling and programming a quadcopter within a laboratory environment. While the base drone platform and simulation environment were prepared by the lab staff, our work focused on assembling and integrating the hardware, configuring the flight controller, and developing the software for a precision landing task using ArUco marker detection. To achieve this, we use ROS2 (Robot Operating System 2) as the software framework, enabling modular control, integration of computer vision, and communication with the flight controller.

The work is carried out in the EOLab drone environment, a controlled laboratory space maintained by the teaching staff, which provides the infrastructure and safety conditions for conducting flight experiments.

This report serves both as a documentation of our project and as a manual for replicating the build. It is organized into three main sections:
	1.	Drone Assembly 
	2.	Drone Configuration
	3.	Drone Programming

## Section 1 – Drone Assembly

### 1.1 Components List
The following table lists all components used to assemble our quadcopter. While some parts were taken from the F450 Flame Wheel kit, several others were custom-built with the help of the teaching staff or provided by them to replace unused kit parts and adapt the drone for our project. 

| Component                         | Model / Type                      | Quantity |
|-----------------------------------|-----------------------------------|----------|
| Flight Controller                 | Pixhawk 2.4.8                     | 1        |
| Companion Computer                | –                                 | 1        |
| Frame Arms                        | 2× 450FAC (red), 2× 450FAW (white)| 4        |
| Motors                            | 2× CW, 2× CCW                     | 4        |
| Propellers                        | 2× CW, 2× CCW                     | 4        |
| ESC (Electronic Speed Controllers)| –                                 | 4        |
| Top Board                         | –                                 | 1        |
| Power Distribution Board (PDB)    | –                                 | 1        |
| Battery                           | LiPo, 4S                          | 1        |       
| Buzzer                            | Active Buzzer Module              | 1        |
| Radio Telemetry                   | Telem 02 (RX)                     | 1        |
| GPS Module                        | –                                 | 1        |
| Landing Legs                      | –                                 | 4        |

### 1.2 Assembly Process
Before starting, prepare all components on your workspace and identify each part.  
Read the instructions carefully before performing each step. 

#### Step 1 – Prepare and Solder the Power Distribution Board (PDB)
- Begin by soldering the **ESC power cables** directly to the **bottom board**. For our bottom board, we build a **power distribution board**  to simplify battery connections.   
  - **Red wire → + (positive terminal)**  
  - **Black wire → – (negative terminal)**  
-  Double-check polarity before soldering.  
-  Be mindful: soldering tools become very hot — make sure you are comfortable with the process before starting.  

### Step 2 – Mount the ESCs
- Secure each ESC along the inside of the frame arms using plastic clamps or zip ties.  
- Place them midway along the arm to keep wiring organized.

### Step 3 – Assemble the Frames
- Separate the two **white arms (450FAW)** from the two **red arms (450FAC)**.  
- Decide which color will mark the **front** of your drone (in our case: white = front, red = back).  
- Attach the legs of the arms to the **PDB (bottom board)** using screws.

### Step 4 – Attach the Motors
- Fix each motor to the free end of a frame arm.  
- Connect each motor to its corresponding ESC.  
- Pay attention to motor orientation: two motors must rotate **clockwise (CW)** and two **counter-clockwise (CCW)**.  
  - Position them diagonally (e.g., front-left and back-right = CW, front-right and back-left = CCW).  
- Do not mount the propellers yet.  
- ![Figure Y: Correct motor orientation diagram](images/motor-orientation.jpg)

### Step 5 – Attach the Landing Legs
- Secure the four landing legs to the bottom board using screws.  
- Ensure they are firmly tightened and positioned evenly for stable landings.  
- Double-check that no wires are trapped between the legs and the frame.  

### Step 6 – Install the Companion Computer
- Place the companion computer between the boards.
- This module handles higher-level processing (e.g., vision tasks).  

### Step 7 – Install the Top Board
- Place the top board onto the frame, above the arms and PDB (bottom board).  
- Secure it using screws to firmly close the frame structure.  
- Make sure all cables from the ESCs and power distribution board are routed properly before tightening, so they are not pinched or damaged.   

### Step 8 – Mount the Flight Controller
- Fix the **Pixhawk flight controller** at the center-top of the frame using double-sided tape.  
- Ensure the orientation arrow points forward.  

### Step 9 – Connect the ESCs to Pixhawk
- Connect ESC signal wires to **MAIN OUT 1–4** on the Pixhawk.  
- Ensure correct mapping of **signal, ground, and power** pins.  
- ![Figure Z: Example wiring diagram](path/to/image.png)

### Step 9 – Connect Peripherals
- **GPS module** → GPS port, I²C
- **Telemetry radio** → TELEM1   
- **Buzzer** → BUZZ port    
- Position devices so that no cables obstruct propellers.  
- *(We 3D-printed a GPS mount for improved placement.)*

### Step 10 – Final Checks
- Verify all screws are secure.  
- Ensure no wires touch the motors or propellers.  
- Confirm polarity one last time before connecting the battery.  
- Keep propellers off until configuration and motor tests are complete.

The assembly of our drone is still in progress; therefore, some additional steps and components will be added to these section. Detailed information about the PDB,and the battery will also be provided soon.

## Section 2 – Drone Configuration

Selecting the right flight controller can be a challenging task, as it must match the drone’s requirements and mission goals. For our build, we selected the **Pixhawk 2.4.8**, a reliable and well-documented option widely used in educational projects.

### 2.1 Pixhawk Overview
The **Pixhawk** is an open-hardware flight controller that serves as the **“brain” of the drone**. It interprets pilot commands and sensor data to regulate the motors, ensuring stable and responsive flight. The board integrates a variety of sensors that measure movement, orientation, and environmental data. Using this information, it continuously adjusts motor speeds to execute flight maneuvers. 

### Key Features
- Support for **8 RC channels** and **4 serial ports**  
- Multiple user interfaces for **programming, log review, and configuration**  
- **Smartphone and tablet apps** available for convenient setup  
- Automatic detection and configuration of connected peripherals  

### Technical Specifications
- **SKU:** 29453  
- **Supply Voltage:** 7 V  
- **Processor:** 32-bit ARM Cortex-M4 core  
- **Bus Interfaces:** UART, I²C, SPI, CAN  
- **Firmware:** Mission Planner  
- **Sensors:** Gyroscope, Accelerometer, Barometer, Magnetometer  
- **Storage:** microSD card slot for flight data logging  


### 2.2 Relationship of Pixhawk & PX4
The Pixhawk, running the **PX4 autopilot stack**, provides several advantages:
- A Unix/Linux-like programming environment  
- Advanced autopilot functions for flexible mission planning  
- Support for **sophisticated mission scripting and flight behaviors**  
- A custom PX4 driver layer ensuring **precise timing across processes**  

This combination makes Pixhawk 2.4.8 a robust platform for both **learning** and **practical drone applications**, enabling us to configure the drone for reliable flight and prepare it for the **precision landing assignment**. 





## Section 2 – Drone Configuration

### 2.1 Pixhawk Overview
- General description of Pixhawk  
- Relationship between Pixhawk and PX4  
- Key features and advantages  

### 2.2 Connecting the Flight Controller
- Wiring and power connections  
- Port functions (TELEM, GPS, CAN, I²C, etc.)  
- Safety considerations  

### 2.3 Firmware Setup
- Install **QGroundControl** and open it.  
- Go to the **Firmware** section and follow the on-screen instructions.  
- Connect your computer to the **Pixhawk** using a USB cable.  
- Update the firmware by selecting **PX4 Pro v1.16.0 – Stable Release**.  

#### Example Screenshots
1. ![QGroundControl Firmware Tab](images/01_Firmware Setup.jpg)  
   *Figure: Firmware section in QGroundControl.*  

2. ![Pixhawk USB Connection](images/02_Firmware Setup.jpg)  
   *Figure: Connecting Pixhawk to the computer via USB.*  

3. ![Firmware Selection](images/qgc-firmware-selection.jpg)  
   *Figure: Choosing PX4 Pro v1.16.0 (Stable Release) during setup.*    

### 2.4 Sensor Calibration
- Accelerometer calibration  
- Gyroscope calibration  
- Magnetometer (compass) calibration  
- Radio/RC calibration  
- ESC calibration  

### 2.5 Configuring Peripheral Devices
- GPS module  
- Telemetry radio  
- Buzzer  
- Additional sensors (e.g., rangefinder, camera)  

### 2.6 Parameter Settings
- Key parameters for stable flight  
- Example of non-standard configurations (if used)  
- Saving and restoring parameter files  

### 2.7 Final Checks
- Verifying all connections  
- Ensuring firmware and parameters are updated  
- Running initial test flights without propellers  

more info ...
