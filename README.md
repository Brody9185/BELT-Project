![9185 Banner](Docs/Assests/9185 Red CM.png "9185 Banner")

Hello, we are a group of Programmers from a VRC High School Organization, and we go by the 9185 number.
This prodject is an attempt to organize the likes of the LemLib, and Ez Templates into one cohersive file.
We also plan to work of other generally untilities for VRC use and will both include them here, but also as their own file meant to implamented into any PROS(And maybe even Vex Code Pro) Prodject.

Advice from someone else working with this implamentation:
# EZ Template + LemLib

## necesseities:

1. when calibrating the chassis in `void initialize();`, be sure to initialize your EZ-Template chassis 
object first, then your LemLib chassis object. the boolean in `LEMchassis.calibrate(false)' makes it so that only EZ-Template
calibrates the IMU to avoid conflicts

2. EZ-Template pid must be turned off before calling any LemLib functions. Call `EZchassis.pid_drive_toggle(false)` before using any LemLib
movement functions, and the same function but to true before using any EZ-Template movement functions.

3. LemLib and EZ-Template movement functions will run asynchronously unless specified. `EZchassis.pid_wait();` and `LEMchassis.waitUntilDone();`
ensure that the previously called movement function will be completed before reading the next line of code.
