/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Code modified and produced by FRC team 6821 Rogue Techs                    */
/* for the 2020 season: Infinite Recharge.                                    */
/* Code written by Ashley M. Farrell - Last modified 2/03/2020                */
/*----------------------------------------------------------------------------*/

// FOR PROGRAM TO WORK CTRE-Phoenix LIBRARY MUST BE DOWNLOADED
#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>  // Library to connect the code and the driver station
#include <frc/Joystick.h>  // Library for robot controllers
#include <cameraserver/CameraServer.h>  // Library for the camera to work
#include <frc/drive/DifferentialDrive.h>  // Library for arcade drive
#include <frc/robotdrive.h>  // Library to allow the robot to drive
#include <frc/GenericHID.h>

#include "ctre/Phoenix.h"  // Library for motor controller


// ---------------------------------------------------------------
// Define Motors
// Define the motors to their variable names and corresponding ports on the robot
// ---------------------------------------------------------------

// Define drive motors
VictorSPX *leftMotorMaster = new VictorSPX(1);
VictorSPX *leftMotorSlave = new VictorSPX(2);
VictorSPX *rightMotorMaster = new VictorSPX(3);
VictorSPX *rightMotorSlave = new VictorSPX(4);
// Define manipulator motors
VictorSPX *shooterRotate = new VictorSPX(5);
VictorSPX *shooterWheelA = new VictorSPX(6);
VictorSPX *shooterWheelB = new VictorSPX(7);
VictorSPX *shooterSpring = new VictorSPX(8); 
VictorSPX *climbingLift = new VictorSPX(9);

RobotDrive *myDrive = new RobotDrive(leftMotorMaster, leftMotorSlave, rightMotorMaster, rightMotorSlave);


// ---------------------------------------------------------------
// Define Controllers
// Define which controllers (joysticks) will be the driver or coDriver for control
// ---------------------------------------------------------------

frc::Joystick * driver = new frc::Joystick(0);
frc::Joystick * coDriver = new frc::Joystick(1);


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


  // ---------------------------------------------------------------
  // Define motor followers
  // Assign certain motors to follow the same instructions of their master motor
  // ---------------------------------------------------------------
/*
  leftMotorSlave->Set(ControlMode::Follower, 1);
  rightMotorSlave->Set(ControlMode::Follower, 3);
*/
  // ---------------------------------------------------------------
  // Robot Drive Code
  // Define the robot drive assignments to allow for arcade drive
  // ---------------------------------------------------------------

  //frc::RobotDrive::RobotDrive(leftMotorMaster, leftMotorSlave, rightMotorMaster, rightMotorSlave);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  /*                    Driver Controller Code                    */


}

void Robot::TeleopPeriodic() {

  // ---------------------------------------------------------------
  // Robot Drive Code
  // Define the robot to drive with an arcade drive
  // ---------------------------------------------------------------

  //frc::RobotDrive::ArcadeDrive(driver->GetRawAxis(2), driver->GetRawAxis(1));

  double forwardSpeed = driver->GetRawAxis(1);
  double turnAmount = driver->GetRawAxis(2);

  frc::RobotDrive::ArcadeDrive(forwardSpeed, turnAmount, false);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
