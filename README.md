# Drone Technology SS2025
# Section 1 – Drone Assembly

## 1.1 Components List


## 1.2 Assembly Process

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
...

more info ...
