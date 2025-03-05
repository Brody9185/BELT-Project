#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/tracking_wheel.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <numbers>
namespace BeltFunctions{

    //Global Vars
/** 
        * @param LineCounter A Variable used to ensure that Data Values printed move up to the highest set line position.
        */
        inline int LineCounter = 4;

    namespace util {
        #define ToggleVar
        inline void ToggleBool(bool &ToggleBoolv){
        if(ToggleBoolv) {
        ToggleBoolv = false;
        } else if(!ToggleBoolv){
            ToggleBoolv = true;
        }
        }
        inline double VertTrackDiameterForTheta(double WheelDiameter, double WheelAngle){
        double v;
        v = cos(WheelAngle) * numbers::pi * WheelDiameter;
        return v;
        }
        inline double HorTrackDiameterForTheta(double WheelDiameter, double WheelAngle){
            double h;
            h = sin(WheelAngle) * numbers::pi * WheelDiameter;
            return h;
            }
    };
    
    inline ez::tracking_wheel LeftV(Ver1Tracking_PORT, BeltFunctions::util::VertTrackDiameterForTheta(VerTrackingWheelDiameter, 45), Ver1Offset);
    inline ez::tracking_wheel LeftH(Ver1Tracking_PORT, BeltFunctions::util::HorTrackDiameterForTheta(VerTrackingWheelDiameter, 45), Hor2Offset);
    inline ez::tracking_wheel RightV(Hor1Tracking_PORT, BeltFunctions::util::VertTrackDiameterForTheta(HorTrackingWheelDiameter, 45), Ver2Offset);
    inline ez::tracking_wheel RightH(Hor1Tracking_PORT, BeltFunctions::util::HorTrackDiameterForTheta(HorTrackingWheelDiameter, 45), Hor1Offset);
    class AngledTrackingWheels {
        public:
        int LeftPositive;
        int RightPositive;
        double OldWheelPos;
        double NewWheelPos;
        double WheelPosError;
        int integrationTime;
        int ErrorRange;
        double AccDif;
        int AllowedAccDif = 200;
        inline void setWheelEncoders(pros::Rotation LeftE, pros::Rotation RightE){
        OldWheelPos = LeftE.get_angle();
        pros::delay(integrationTime);
        NewWheelPos = LeftE.get_angle();
        WheelPosError = NewWheelPos - OldWheelPos;
        if(WheelPosError > ErrorRange){
        LeftPositive = 1;
        } else if(WheelPosError < -ErrorRange){
        LeftPositive = 2;
        } else if(WheelPosError < ErrorRange &&  WheelPosError > -ErrorRange){
        LeftPositive = 0;
        }
        OldWheelPos = RightE.get_angle();
        pros::delay(integrationTime);
        NewWheelPos = RightE.get_angle();
        if(NewWheelPos - OldWheelPos > ErrorRange){
        RightPositive = 1;
        } else if(NewWheelPos - OldWheelPos < -ErrorRange){
        RightPositive = 2;
        }
        AccDif = abs(RightE.get_velocity() - LeftE.get_velocity());


        if(LeftPositive == 2 && RightPositive == 2 && AccDif < AllowedAccDif || LeftPositive == 1 && RightPositive == 1 && AccDif < AllowedAccDif){
        EZchassis.odom_tracker_right_set(&RightV);
        EZchassis.odom_tracker_left_set(&LeftV);
        EZchassis.odom_tracker_front_set(nullptr);
        EZchassis.odom_tracker_back_set(nullptr);
        }else if(LeftPositive == 1 && RightPositive == 2 && AccDif < AllowedAccDif || LeftPositive == 2 && RightPositive == 1 && AccDif < AllowedAccDif){
        EZchassis.odom_tracker_right_set(nullptr);
        EZchassis.odom_tracker_left_set(nullptr);
        EZchassis.odom_tracker_front_set(&RightH);
        EZchassis.odom_tracker_back_set(&LeftH);
        }else if(LeftPositive == 1 && RightPositive == 0 && AccDif < AllowedAccDif|| LeftPositive == 2 && RightPositive == 0 && AccDif < AllowedAccDif){
        EZchassis.odom_tracker_right_set(nullptr);
        EZchassis.odom_tracker_left_set(&LeftV);
        EZchassis.odom_tracker_front_set(nullptr);
        EZchassis.odom_tracker_back_set(&LeftH);
        }else if(LeftPositive == 0 && RightPositive == 1 && AccDif < AllowedAccDif || LeftPositive == 0 && RightPositive == 2 && AccDif < AllowedAccDif){
        EZchassis.odom_tracker_right_set(&RightV);
        EZchassis.odom_tracker_left_set(nullptr);
        EZchassis.odom_tracker_front_set(&RightH);
        EZchassis.odom_tracker_back_set(nullptr);
        }else if(LeftPositive > 0 && RightPositive > 0 && AccDif > AllowedAccDif){
            EZchassis.odom_tracker_right_set(&RightV);
            EZchassis.odom_tracker_left_set(nullptr);
            EZchassis.odom_tracker_front_set(nullptr);
            EZchassis.odom_tracker_back_set(&LeftH);
        }
        }
    };


class driverProfile {

    public:

        bool Active = false;
        bool Shift = false;
        /**
        @param ToggleButton1 Var to set the 1st toggle button;
        */
        pros::controller_digital_e_t ToggleButton1;
        /**
        @param ToggleButton2 Var to set the 2nd toggle button;
        */
        pros::controller_digital_e_t ToggleButton2;
        /**
        @param ToggleButton3 Var to set the 3rd toggle button;
        */
        pros::controller_digital_e_t ToggleButton3;
        /**
        @param ToggleButton4 Var to set the 4th toggle button;
        */
        pros::controller_digital_e_t ToggleButton4;
        /**
        @param SToggleButton1 Var to set the 1st Shift toggle button;
        */
        pros::controller_digital_e_t SToggleButton1;
        /**
        @param SToggleButton2 Var to set the 2nd Shift toggle button;
        */
        pros::controller_digital_e_t SToggleButton2;
        /**
        @param SToggleButton3 Var to set the 3rd Shift toggle button;
        */
        pros::controller_digital_e_t SToggleButton3;
        /**
        @param SToggleButton4 Var to set the 4th Shift toggle button;
        */
        pros::controller_digital_e_t SToggleButton4;
        bool Toggle1 = false;
        bool Toggle2 = false;
        bool Toggle3 = false;
        bool Toggle4 = false;
        bool SToggle1 = false;
        bool SToggle2 = false;
        bool SToggle3 = false;
        bool SToggle4 = false;
        //Toggle
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button2 Button Input for the second Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button3 Button Input for the third Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button4 Button Input for the fourth Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setToggleButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2,pros::controller_digital_e_t Button3, pros::controller_digital_e_t Button4){
            ToggleButton1 = Button1;
            ToggleButton2 = Button2;
            ToggleButton3 = Button3;
            ToggleButton4 = Button4;
        }
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button2 Button Input for the second Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button3 Button Input for the third Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button4 Button Input for the fourth Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setSToggleButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2,pros::controller_digital_e_t Button3, pros::controller_digital_e_t Button4){
            SToggleButton1 = Button1;
            SToggleButton2 = Button2;
            SToggleButton3 = Button3;
            SToggleButton4 = Button4;
        }
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button2 Button Input for the second Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button3 Button Input for the third Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setToggleButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2,pros::controller_digital_e_t Button3){
            ToggleButton1 = Button1;
            ToggleButton2 = Button2;
            ToggleButton3 = Button3;
        }
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button2 Button Input for the second Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button3 Button Input for the third Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setSToggleButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2,pros::controller_digital_e_t Button3){
            SToggleButton1 = Button1;
            SToggleButton2 = Button2;
            SToggleButton3 = Button3;
        }
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button2 Button Input for the second Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setToggleButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2){
            ToggleButton1 = Button1;
            ToggleButton2 = Button2;
        }
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        * @param Button2 Button Input for the second Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setSToggleButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2){
            SToggleButton1 = Button1;
            SToggleButton2 = Button2;
        }
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setToggleButtons(pros::controller_digital_e_t Button1){
            ToggleButton1 = Button1;
        }
        /** 
        * @param Button1 Button Input for the first Toggle(pros::E_CONTROLLER_DIGITAL_* format).
        */
        inline void setSToggleButtons(pros::controller_digital_e_t Button1){
            SToggleButton1 = Button1;
        }
        inline void Toggle1F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(ToggleButton1)){
        util::ToggleBool(Toggle1);
        }
        if(Toggle1){
            OnFunction();
        } else {
            OffFunction();
        }
        }
        inline void Toggle2F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(ToggleButton2)){
        util::ToggleBool(Toggle2);
        }
        if(Toggle2){
            OnFunction();
        } else {
            OffFunction();
        }
        }
        inline void Toggle3F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(ToggleButton3)){
        util::ToggleBool(Toggle3);
        }
        if(Toggle3){
            OnFunction();
        } else {
            OffFunction();
        }
        }
        inline void Toggle4F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(ToggleButton4)){
        util::ToggleBool(Toggle4);
        }
        if(Toggle4){
            OnFunction();
        } else {
            OffFunction();
        }
        }
        inline void SToggle1F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(SToggleButton1)){
        util::ToggleBool(SToggle1);
        }
        if(SToggle1){
            OnFunction();
        } else {
            OffFunction();
        }
        }
        inline void SToggle2F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(SToggleButton2)){
        util::ToggleBool(SToggle2);
        }
        if(SToggle2){
            OnFunction();
        } else {
            OffFunction();
        }
        }
        inline void SToggle3F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(SToggleButton3)){
        util::ToggleBool(SToggle3);
        }
        if(SToggle3){
            OnFunction();
        } else {
            OffFunction();
        }
        }
        inline void SToggle4F(void (*OnFunction)(void), void (*OffFunction)(void)){
        if(master.get_digital_new_press(SToggleButton4)){
        util::ToggleBool(SToggle4);
        }
        if(SToggle4){
            OnFunction();
        } else {
            OffFunction();
        }
        }


        
        //Intake
        /**
        @param IntakeButton Var to set the Button to intake.
        */
        pros::controller_digital_e_t IntakeButton;
        /**
        @param OuttakeButton Var to set the Button to outtake.
        */
        pros::controller_digital_e_t OuttakeButton;
        /**
        @param SIntakeButton Var to set the Button to intake while shifted;
        */
        pros::controller_digital_e_t SIntakeButton;
        /**
        @param SOuttakeButton Var to set the Button to outtake while shifted;
        */
        pros::controller_digital_e_t SOuttakeButton;
        /**
        @param IntakeSpeed Var to set the speed the intake runs at(-127 - 127);
        */
        int intakeSpeed = 127;

        /**
        * @param SetIntakeButtons Set the Buttons to be used for the intake function, and the Speed it runs.
        * @param Button1 Button to intake.
        * @param Button2 Button to outtake.
        * @param IntakeSpeed Value(-127 - 127) that the intake will run.
        */
        inline void setIntakeButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2, int IntakeSpeed){
            IntakeButton = Button1;
            OuttakeButton = Button2;
        }
        /**
        * @param SetSIntakeButtons Set the Buttons to be used for the intake function when set to use the shift button, and the Speed it runs.
        * @param Button1 Button to intake.
        * @param Button2 Button to outtake.
        * @param IntakeSpeed Value(-127 - 127) that the intake will run.
        */
        inline void setSIntakeButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2, int IntakeSpeed){
            SIntakeButton = Button1;
            SOuttakeButton = Button2;
        }
        /**
        * @param SetIntakeButtons Set the Buttons to be used for the intake function.
        * @param Button1 Button to intake.
        * @param Button2 Button to outtake.
        * @param IntakeSpeed Value(-127 - 127) that the intake will run.
        */
        inline void setIntakeButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2){
            intakeSpeed = 127;
            IntakeButton = Button1;
            OuttakeButton = Button2;
        }
        /**
        * @param SetIntakeButtons Set the Buttons to be used for the intake function when set to use the shift button, and the Speed it runs.
        * @param Button1 Button to intake.
        * @param Button2 Button to outtake.
        * @param IntakeSpeed Value(-127 - 127) that the intake will run.
        */
        inline void setSIntakeButtons(pros::controller_digital_e_t Button1, pros::controller_digital_e_t Button2){
            intakeSpeed = 127;
            SIntakeButton = Button1;
            SOuttakeButton = Button2;
        }
        /**
        * @param Intake Sets and Runs the Intake Function.
        * @param Slocked Determines weather or not the shift must be active in order to run the intake.
        */
        inline void IntakeF(bool Slocked) {
            if(!Slocked && driverProfile::Active){
                if(!WrongColor){
                    setIntake((master.get_digital(IntakeButton)-master.get_digital(OuttakeButton))*intakeSpeed);
                }
            }else if(Slocked&& driverProfile::Active){
            if(Shift){
                if(!WrongColor){
                    setIntake((master.get_digital(IntakeButton)-master.get_digital(OuttakeButton))*intakeSpeed);
                }
            }
        }
    }
};
    class DataDisplay{
        public:
        //Data Display
        /** 
        * @param BatteryPg Var used to save the line BatteryP will print to.
        */
        int BatteryPg;
        /** 
        * @param Mtr1Pg Var used to save the Motor1Temp will print to.
        */
        int Mtr1Pg;
        /** 
        * @param Mtr2Pg Var used to save the line Motor2Temp will print to.
        */
        int Mtr2Pg;
        /** 
        * @param BsPg Var used to save the line AvgBaseMotorTemp will print to.
        */
        int BsPg;
        /** 
        * @param AvgBaseMotorTemp Var used to calculate and save the Avg Base Tempature.
        */
        int AvgBaseMotorTemp;
        /** 
        * @param SetBaseMotors A function used to declare, and update base motors and their Avg Temp(Put Inside of a task).
        * @param BM1 A Variable used to set the motor used as Base Motor 1.
        * @param BM2 A Variable used to set the motor used as Base Motor 2.
        * @param BM3 A Variable used to set the motor used as Base Motor 3.
        * @param BM4 A Variable used to set the motor used as Base Motor 4.
        * @param BM5 A Variable used to set the motor used as Base Motor 5.
        * @param BM6 A Variable used to set the motor used as Base Motor 6.
        * @param BM7 A Variable used to set the motor used as Base Motor 7.
        * @param BM8 A Variable used to set the motor used as Base Motor 8.
        */
        inline void SetBaseMotors(pros::Motor BM1, pros::Motor BM2, pros::Motor BM3, pros::Motor BM4, pros::Motor BM5, pros::Motor BM6, pros::Motor BM7, pros::Motor BM8){
            AvgBaseMotorTemp = (BM1.get_temperature() + BM2.get_temperature() + BM3.get_temperature() + BM4.get_temperature() + BM5.get_temperature() + BM6.get_temperature() + BM7.get_temperature() + BM8.get_temperature()) / 8;
        }
        /** 
        * @param SetBaseMotors A function used to declare, and update base motors and their Avg Temp(Put Inside of a task).
        * @param BM1 A Variable used to set the motor used as Base Motor 1.
        * @param BM2 A Variable used to set the motor used as Base Motor 2.
        * @param BM3 A Variable used to set the motor used as Base Motor 3.
        * @param BM4 A Variable used to set the motor used as Base Motor 4.
        * @param BM5 A Variable used to set the motor used as Base Motor 5.
        * @param BM6 A Variable used to set the motor used as Base Motor 6.
        */
        inline void SetBaseMotors(pros::Motor BM1, pros::Motor BM2, pros::Motor BM3, pros::Motor BM4, pros::Motor BM5, pros::Motor BM6){
            AvgBaseMotorTemp = (BM1.get_temperature() + BM2.get_temperature() + BM3.get_temperature() + BM4.get_temperature() + BM5.get_temperature() + BM6.get_temperature()) / 6;
        }
        /** 
        * @param SetBaseMotors A function used to declare, and update base motors and their Avg Temp(Put Inside of a task).
        * @param BM1 A Variable used to set the motor used as Base Motor 1.
        * @param BM2 A Variable used to set the motor used as Base Motor 2.
        * @param BM3 A Variable used to set the motor used as Base Motor 3.
        * @param BM4 A Variable used to set the motor used as Base Motor 4.
        */
        inline void SetBaseMotors(pros::Motor BM1, pros::Motor BM2, pros::Motor BM3, pros::Motor BM4){
            AvgBaseMotorTemp = (BM1.get_temperature() + BM2.get_temperature() + BM3.get_temperature() + BM4.get_temperature()) / 4;
        }
        /** 
        * @param SetBaseMotors A function used to declare, and update base motors and their Avg Temp(Put Inside of a task).
        * @param BM1 A Variable used to set the motor used as Base Motor 1.
        * @param BM2 A Variable used to set the motor used as Base Motor 2.
        */
        inline void SetBaseMotors(pros::Motor BM1, pros::Motor BM2){
            AvgBaseMotorTemp = (BM1.get_temperature() + BM2.get_temperature()) / 2;
        }
        /**
        * @param DOn A Variable used to check if BrainDisplay is active.
        */
        bool DOn;
        /**
        * @param BatteryP A Variable used to check if batter percentage display is active.
        */
        bool BatteryP;
        /**
        * @param Motor1TempOn A Variable used to check if Motor1's temp display is active.
        */
        bool Motor1TempOn;
        /** 
        * @param Motor1Temp A Variable used to check and save the currect tempature of Motor1.
        */
        int Motor1Temp;
        /**
        * @param Motor2TempOn A Variable used to check if Motor2's temp display is active.
        */
        bool Motor2TempOn;
        /** 
        * @param Motor2Temp A Variable used to check and save the currect tempature of Motor2.
        */
        int Motor2Temp;
        bool AvgBaseTemp;
        /** 
        * @param BrainDisplay A function used to active Brain data displays, and which data values should be displayed.
        * @param DOn A Variable used to check if BrainDisplay is active.
        * @param BatteryP A Variable used to check if batter percentage display is active.
        * @param Motor1TempOn A Variable used to check if Motor1's temp display is active.
        * @param Motor1 A Variable used to set the motor data used by Motor1Temp.
        * @param Motor2TempOn A Variable used to check if Motor2's temp display is active.
        * @param Motor2 A Variable used to set the motor data used by Motor1Temp.
        * @param AvgBaseTemp A Variable used to check if the Avg Base temp display is active.
        */
        inline void BrainDisplay(bool DOn, bool BatteryP, bool Motor1TempOn, pros::Motor Motor1, bool Motor2TempOn, pros::Motor Motor2, bool AvgBaseTemp){
            if(DOn){
                if(BatteryP) {
                BatteryPg = LineCounter;
                LineCounter = LineCounter + 1;
                }
                if(Motor1TempOn) {
                Motor1Temp = Motor1.get_temperature();
                Mtr1Pg = LineCounter;
                LineCounter = LineCounter + 1;
                }
                if(Motor2TempOn) {
                Motor2Temp = Motor2.get_temperature();
                Mtr2Pg = LineCounter;
                LineCounter = LineCounter + 1;
                }
                if(AvgBaseTemp) {
                BsPg = LineCounter;
                LineCounter = LineCounter + 1;
                }
            }
        }

        inline void BrainDisplay(bool DOn, bool BatteryP, bool Motor1TempOn, pros::Motor Motor1, bool Motor2TempOn, pros::Motor Motor2){
            if(DOn){
                if(BatteryP) {
                    BatteryPg = LineCounter;
                    LineCounter = LineCounter + 1;
                    }
                    if(Motor1TempOn) {
                    Motor1Temp = Motor1.get_temperature();
                    Mtr1Pg = LineCounter;
                    LineCounter = LineCounter + 1;
                    }
                    if(Motor2TempOn) {
                    Motor2Temp = Motor2.get_temperature();
                    Mtr2Pg = LineCounter;
                    LineCounter = LineCounter + 1;
                    }
            }
        }

        inline void BrainDisplay(bool DOn, bool BatteryP, bool Motor1TempOn, pros::Motor Motor1){
            if(DOn){
                if(BatteryP) {
                    BatteryPg = LineCounter;
                    LineCounter = LineCounter + 1;
                    }
                    if(Motor1TempOn) {
                    Motor1Temp = Motor1.get_temperature();
                    Mtr1Pg = LineCounter;
                    LineCounter = LineCounter + 1;
                    }
            }
        }

        inline void BrainDisplay(bool DOn, bool BatteryP){
            if(DOn){
                if(BatteryP) {
                    BatteryPg = LineCounter;
                    LineCounter = LineCounter + 1;
                    }
            }
        }

        inline void BrainDisplay(bool On){
        }
        inline void DisplayTask(){
            while(true){
            if(DOn){
                if(BatteryP) {
                    pros::lcd::print(BatteryPg, "Battery: %f", pros::battery::get_capacity(), "%");
                }
                if(Motor1TempOn) {
                    pros::lcd::print(Mtr1Pg, "Motor1: %f", Motor1Temp, "c");
                }
                if(Motor2TempOn) {
                    pros::lcd::print(Mtr2Pg, "Motor2: %f", Motor2Temp, "c");
                }
                if(AvgBaseTemp) {
                    pros::lcd::print(BsPg, "Base: %f", AvgBaseMotorTemp, "c");
                }
            }
        }
        }
        
    };

};

inline BeltFunctions::driverProfile Profile1;
inline BeltFunctions::DataDisplay DataSet1;
inline void ProfInit(){
    //Profile1
    //driverProfile Profile1;
    Profile1.Active = true;
    //SetButtons
    Profile1.setIntakeButtons(pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1,127);
    //SetVals
    DataSet1.BrainDisplay(false,false,true,12,true,2,true);

}

inline void ProfTask() {
    while (true) {
    //Intake Control
    Profile1.IntakeF(false);
    //Print Data to Brain
    DataSet1.SetBaseMotors(LEFTMOTOR1,LEFTMOTOR2,LEFTMOTOR3,RIGHTMOTOR1,RIGHTMOTOR2,RIGHTMOTOR3);
    }
}

