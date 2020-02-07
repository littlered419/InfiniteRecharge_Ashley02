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
#include <frc/RobotDrive.h>  // Library to allow the robot to drive
#include <frc/drive/RobotDriveBase.h>
#include <frc/GenericHID.h>

#include "ctre/Phoenix.h"  // Library for motor controller


// ---------------------------------------------------------------
// Define Motors
// Define the motors to their variable names and corresponding ports on the robot
// ---------------------------------------------------------------

// Define drive motors
WPI_VictorSPX *leftMotorMaster = new WPI_VictorSPX(1);
WPI_VictorSPX *leftMotorSlave = new WPI_VictorSPX(2);
WPI_VictorSPX *rightMotorMaster = new WPI_VictorSPX(3);
WPI_VictorSPX *rightMotorSlave = new WPI_VictorSPX(4);
// Define manipulator motors
WPI_VictorSPX *shooterRotate = new WPI_VictorSPX(5);
WPI_VictorSPX *shooterWheelA = new WPI_VictorSPX(6);
WPI_VictorSPX *shooterWheelB = new WPI_VictorSPX(7);
WPI_VictorSPX *shooterSpring = new WPI_VictorSPX(8); 
WPI_VictorSPX *climbingLift = new WPI_VictorSPX(9);


frc::DifferentialDrive *robotDrive = new frc::DifferentialDrive(*leftMotorMaster, *rightMotorMaster);


// ---------------------------------------------------------------
// Define Controllers
// Define which controllers (joysticks) will be the driver or coDriver for control
// ---------------------------------------------------------------

frc::Joystick *driver = new frc::Joystick(0);
frc::Joystick *coDriver = new frc::Joystick(1);


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


  // ---------------------------------------------------------------
  // Define motor followers
  // Assign certain motors to follow the same instructions of their master motor
  // ---------------------------------------------------------------

  //leftMotorSlave->Set(ControlMode::Follower, 1);
  //rightMotorSlave->Set(ControlMode::Follower, 3);

  // ---------------------------------------------------------------
  // Robot Drive Code
  // Define the robot drive assignments to allow for arcade drive
  // ---------------------------------------------------------------

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

  robotDrive->ArcadeDrive(1, 1, false);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
