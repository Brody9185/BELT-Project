![9185 Banner](Docs/Assests/9185_Red_CM.png "9185 Banner")

# BELT Project
Hello, we are a group of Programmers from a VRC High School Organization, and we go by the 9185 number.
This prodject is an attempt to organize the likes of the LemLib, and Ez Templates into one cohersive file.
We also plan to work of other generally untilities for VRC use and will both include them here, but also as their own file meant to implamented into any PROS(And maybe even Vex Code Pro) Project.

This is a VRC project meant to allow the use of both EZ Template, and LEMlib functions, aswell as to provide other general untility to teams.


## Badges
![Static Badge](https://img.shields.io/badge/Version-2.2-rgb(204%2C0%2C0))

![Static Badge](https://img.shields.io/badge/PROS-Functional-green)

![Static Badge](https://img.shields.io/badge/Vex_Code_Pro-Unavailable-maroon)

![Static Badge](https://img.shields.io/badge/Example_Project-WIP-yellow)

![Static Badge](https://img.shields.io/badge/Template-WIP-maroon)



## Used and Developed By

This project is used and developed by:

- 9185C Carnage
- 9185T Titan
- 9185V Venom


# Installation

## Example Project
[V 2.2](https://github.com/Brody9185/BELT-Project/blob/main/Releases/BELT%202.2.zip)

## Template
"WIP"
# Uses

- LEMLib + EZ PID Autonomous Movments
- PID Tuner
# Examples

## LEMLib + EZ PID Autonomous Movments
```cpp
void testAuton(){
  updatePID();
  EZchassis.drive_brake_set(MOTOR_BRAKE_HOLD); 
  EZchassis.pid_drive_toggle(false);

  LEMchassis.turnToHeading(100,10);
  allPIDWait();

  LEMchassis.moveToPoint(20, 15, 3000, {true, 60}, true);
  allPIDWait();

  LEMchassis.turnToHeading(0,10);
  allPIDWait();

  EZchassis.pid_drive_toggle(true);
  EZchassis.pid_turn_set(-45, 110, true);
  allPIDWait();

  EZchassis.pid_drive_set(25, 110, true);
  allPIDWait();

  EZchassis.pid_drive_toggle(false);
}
```
By Declaring that the EZchassis is disabled you can use LEMLib functions freely, and then by enabling EZchassis later you can use its movments while LEMLib keeps track of the X,Y of the robot.

## PID Tuner
We have worked to allow the use of EZ Templates PID Tuner to be used with LEM PID.

Just ensure you run the updatePID(); function at the beginning of each Autonomous.
```cpp
void lin_PID(){
  updatePID();
  EZchassis.pid_drive_toggle(false);
}
```

# Documentation

[Table of Contents](https://github.com/7865-Owlumination/BELT-Project/blob/main/Docs/TOC.md)

# Acknowledgements

## ![Static Badge](https://img.shields.io/badge/Robotics_is_EZ-https%3A%2F%2Fgithub.com%2FEZ--Robotics-rgb(255%2C147%2C213))
 - ![Static Badge](https://img.shields.io/badge/Discord-Jess-rgb(255%2C112%2C200))
## ![Static Badge](https://img.shields.io/badge/LEMLib-https%3A%2F%2Fgithub.com%2FLemLib-lime)
 - ![Static Badge](https://img.shields.io/badge/Discord-Lembron_James-rgb(0%2C148%2C0))

## Others
- ![Static Badge](https://img.shields.io/badge/Discord-Praful_%7C_5839B-%23feec95)
- ![Static Badge](https://img.shields.io/badge/Discord-kapri-rgb(234%2C144%2C238))


