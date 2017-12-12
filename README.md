My Lil Homey - BlueNRG2 Modules
===============================

Interfaces
------------------------
This Repo contains 2 My Lil Homey projects:
Lil_BLE_Homey - The Magic Vent application code
Lil_Blind_Homey - The Magic Blind application code 

The following files were created by the Lil Homey Team for interfacing with various peripherals on the BlueNRG-1 or 2, specifically The Magic Blinds:  

Lil_Blind_Homey/src/Lil_MotionDetector.c - Interface for initializing interrupts triggered by the motion detector  
Lil_Blind_Homey/src/Lil_Motor.c - Interface for controlling the motor and calibrating it's opening/closing range by reading from the optical encoder  
Lil_Blind_Homey/src/chat.c - (Modified from demo application) Connects to Magic Vent as Slave. Can be configured to connect directly to Central Hub (Raspberry Pi)  
Lil_Blind_Homey/src/gatt_db.c - Sets up GATT services and characteristics for the Blinds  

The Magic Vent's application code contains the same interfaces as The Magic Blinds (listed above) except with a simpler motor interface and a more complex BLE connection state machine (to act as a relay node):  

Lil_BLE_Homey/src/chat.c - Handles connection state machine for acting as Master & Slave simultaneously. Handles BLE event callbacks for things like notifications and remote device writes to local gatt characteristics  
Lil_BLE_Homey/src/gatt_db.c - Sets up GATT services and characteristics for the Vents as well as the necessary characteristics to relay the Blind's services   


To start working with this project:
-----------------------------------
Install Git Bash for Windows 10 before proceeding.

Follow these steps exactly to set up repo in Windows 10:
1. Open Git Bash and navigate to the Documents folder
2. Type `git clone [URL] MyLilHomey` where [URL] = This repo's address
3. Copy the "Library" folder from the BlueNRG SW Package into "MyLilHomey". This folder is in .gitignore and will not be committed.
4. Open the TrueStudio project

