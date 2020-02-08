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

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include "ctre/Phoenix.h"  // Library for motor controller


// ---------------------------------------------------------------
// Define Motors
// Define the motors to their variable names and corresponding ports on the robot
// ---------------------------------------------------------------

// Define drive motors
WPI_VictorSPX * leftMotorMaster = new WPI_VictorSPX(1);
WPI_VictorSPX * leftMotorSlave = new WPI_VictorSPX(2);
WPI_VictorSPX * rightMotorMaster = new WPI_VictorSPX(3);
WPI_VictorSPX * rightMotorSlave = new WPI_VictorSPX(4);
// Define manipulator motors
WPI_VictorSPX * shooterRotate = new WPI_VictorSPX(5);
WPI_VictorSPX * shooterWheelA = new WPI_VictorSPX(6);
WPI_VictorSPX * shooterWheelB = new WPI_VictorSPX(7);
WPI_VictorSPX * shooterSpring = new WPI_VictorSPX(8); 
WPI_VictorSPX * climbingLift = new WPI_VictorSPX(9);


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
}

void Robot::TeleopPeriodic() {
  /*                    Driver Controller Code                    */

  // ---------------------------------------------------------------
  // Robot Drive Code
  // Define the robot to drive with an arcade drive using the y (a1) and z (a2) axies
  // ---------------------------------------------------------------

  double forwardSpeed = driver->GetRawAxis(1);
  double turnAmount = driver->GetRawAxis(2);

  robotDrive->ArcadeDrive(forwardSpeed, turnAmount, false);


  // ---------------------------------------------------------------
  // Spring (for shooter) Code
  // Uses the POV to operate the spring going up and down
  // ---------------------------------------------------------------

  int dPOVstate = driver->GetPOV();
  int dPOVswitch;

  float springSpeed = 0.5;

  if(dPOVstate == 0){
    dPOVswitch = 1;
  }
  else if(dPOVstate == 180){
    dPOVswitch = 2;
  }
  else{
    dPOVswitch = 0;
  }

  switch(dPOVswitch){
    // Neutral state --- POV is neither up nor down --- The spring is not moving
    case 0:
      shooterSpring->Set(ControlMode::PercentOutput, 0);
    break;
    // Spring up --- POV is up --- The spring is moving toward the top of the shooter
    case 1:
      shooterSpring->Set(ControlMode::PercentOutput, springSpeed);
    break;
    // Spring down --- POV is down --- The spring is moving toward the base of the shooter
    case 2:
      shooterSpring->Set(ControlMode::PercentOutput, -springSpeed);
    break;
  }



  /*                   CoDriver Controller Code                   */

  // ---------------------------------------------------------------
  // Shooter Rotate Code
  // Uses the y-axis (a1) of the joystick to control the angle of the shooter
  // ---------------------------------------------------------------  

  double rotateSpeed = coDriver->GetRawAxis(1);

  shooterRotate->Set(ControlMode::PercentOutput, rotateSpeed*0.5);


  // ---------------------------------------------------------------
  // Shoot and Intake code
  // Uses the thumb button (b2) to intake and the trigger (b1) to shoot
  // ---------------------------------------------------------------  

  bool shootButton = coDriver->GetRawButton(1);
  bool intakeButton = coDriver->GetRawButton(2);
  float shooterSpeed = 0.5;
  float intakeSpeed = -0.5;
  
  // Define the intake to come first then the shooter code due to driver preference

  // The intake button is pressed and the wheels are intaking power cells
  if(intakeButton == true){
    shooterWheelA->Set(ControlMode::PercentOutput, intakeSpeed);
    shooterWheelB->Set(ControlMode::PercentOutput, -intakeSpeed);
  }
  // The shooter button is pressed and the wheels are shooting
  else if(shootButton == true){
    shooterWheelA->Set(ControlMode::PercentOutput, shooterSpeed);
    shooterWheelB->Set(ControlMode::PercentOutput, -shooterSpeed);
  }
  // Neither button is pressed and the wheels are not moving
  else{
    shooterWheelA->Set(ControlMode::PercentOutput, 0);
    shooterWheelB->Set(ControlMode::PercentOutput, 0);
  }


  // ---------------------------------------------------------------
  // Robot Climb Code (for endgame)
  // Uses the POV to control the state of the lift mechanism
  // ---------------------------------------------------------------

  int cdPOVstate = coDriver->GetPOV();
  int cdPOVswitch;

  float climbSpeed = 0.5;

  if(cdPOVstate == 0){
    cdPOVswitch = 1;
  }
  else if(cdPOVstate == 180){
    cdPOVswitch = 2;
  }
  else{
    cdPOVswitch = 0;
  }

  switch(cdPOVswitch){
    // Neutral state --- POV is neither up nor down --- The climber is not moving
    case 0:
      climbingLift->Set(ControlMode::PercentOutput, 0);
    break;
    // Climb up --- POV is up --- The climber is moving up, away from the robot
    case 1:
      climbingLift->Set(ControlMode::PercentOutput, climbSpeed);
    break;
    // Climb down --- POV is down --- The climber is moving down, towards the robot
    case 2:
      climbingLift->Set(ControlMode::PercentOutput, -climbSpeed);
    break;
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif