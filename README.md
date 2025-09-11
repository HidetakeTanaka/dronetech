# Drone Technology and Application SS2025

As part of our coursework in drone and robotics engineering, we are building a quadcopter to learn and apply theoretical knowledge in a practical environment. The project involves three main stages: assembling the quadcopter hardware, configuring the flight systems, and programming it to perform a precision landing task on an ArUco marker. All experiments are conducted in the EOLab drone environment, which is maintained and prepared by the lab staff.

## Section 1 – Drone Assembly

### 1.1 Components List
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
| Buzzer                            | Active Buzzer Module              | 1        |
| Radio Telemetry                   | Telem 02 (RX)                     | 1        |
| GPS Module                        | –                                 | 1        |
| Landing Legs                      | –                                 | 4        |

### 1.2 Assembly Process

### Step 1 – Assemble the Frame
- Separate the two **white arms (450FAW)** from the two **red arms (450FAC)**.  
- Decide which color will mark the **front** of your drone (commonly red = front, white = back).  
- Attach the arms to the top board using screws and tighten firmly.  
- ⚠️ Orientation helps with flight control and visual recognition in the air.

### Step 2 – Install the ESC Power Leads
- Solder the ESC power cables onto the top board (or power distribution board).  
  - **Red wire → + (positive terminal)**  
  - **Black wire → – (negative terminal)**  
- ⚠️ Double-check polarity before soldering.  
- ![Figure X: Example of ESC soldering](path/to/image.png)

### Step 3 – Mount the ESCs
- Secure each ESC along the inside of the frame arms using plastic clamps or zip ties.  
- Place them midway along the arm to keep wiring organized.  

### Step 4 – Attach the Motors
- Fix each motor to the free end of a frame arm.  
- ⚠️ Do not mount the propellers yet.  
- ![Figure Y: Correct motor orientation diagram](path/to/image.png)

### Step 5 – Install the Bottom Board
- Attach the bottom board to complete the frame structure.  
- In our build, we integrated a **power distribution board (PDB)** between the plates to simplify battery connections.

### Step 6 – Install the Companion Computer *(optional)*
- Place the companion computer between the boards.  
- This module handles higher-level processing (e.g., vision tasks).  

### Step 7 – Mount the Flight Controller
- Fix the **Pixhawk flight controller** at the center-top of the frame using double-sided tape or vibration-damping pads.  
- Ensure the orientation arrow points forward.  

### Step 8 – Connect the ESCs to Pixhawk
- Connect ESC signal wires to **MAIN OUT 1–4** on the Pixhawk.  
- Ensure correct mapping of **signal, ground, and power** pins.  
- ![Figure Z: Example wiring diagram](path/to/image.png)

### Step 9 – Connect Peripherals
- **GPS module** → GPS port  
- **Telemetry radio** → TELEM1/TELEM2  
- **Buzzer** → BUZZ port  
- **Additional sensors** (e.g., camera, rangefinder) → I2C or UART ports  
- ⚠️ Position devices so that no cables obstruct propellers.  
- *(We 3D-printed a GPS mount for improved placement.)*

### Step 10 – Final Checks
- Verify all screws are secure.  
- Ensure no wires touch the motors or propellers.  
- Confirm polarity one last time before connecting the battery.  
- ⚠️ Keep propellers off until configuration and motor tests are complete.

## Section 2 – Drone Configuration

### Choosing the Flight Controller
Selecting the right flight controller can be a challenging task, as it must match the drone’s requirements and mission goals. For our build, we selected the **Pixhawk 2.4.8**, a reliable and well-documented option widely used in research and educational projects.

### Pixhawk Overview
The **Pixhawk** is an open-hardware flight controller that serves as the **“brain” of the drone**. It interprets pilot commands and sensor data to regulate the motors, ensuring stable and responsive flight. The board integrates a variety of sensors that measure movement, orientation, and environmental data. Using this information, it continuously adjusts motor speeds to execute flight maneuvers.

For our project, we are using the **Pixhawk 2.4.8** model. Key features include:
- Support for **8 RC channels** and **4 serial ports**  
- Multiple user interfaces for **programming, log review, and configuration**  
- **Smartphone and tablet apps** available for convenient setup  
- Automatic detection and configuration of connected peripherals  

### Benefits of Pixhawk + PX4
The Pixhawk, running the **PX4 autopilot stack**, provides several advantages:
- A Unix/Linux-like programming environment  
- Advanced autopilot functions for flexible mission planning  
- Support for **sophisticated mission scripting and flight behaviors**  
- A custom PX4 driver layer ensuring **precise timing across processes**  

This combination makes Pixhawk 2.4.8 a robust platform for both **learning** and **practical drone applications**, enabling us to configure the drone for reliable flight and prepare it for the **precision landing assignment**.

more info ...
