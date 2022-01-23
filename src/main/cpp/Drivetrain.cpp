// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain() {
  m_rightGroup.SetInverted(true);

  m_leftEncoder.SetDistancePerPulse(kDrivetrainDistancePerPulse);
  m_rightEncoder.SetDistancePerPulse(kDrivetrainDistancePerPulse);

  m_gyro.Reset();
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();

  frc::SmartDashboard::PutData("Field", &m_fieldSim);
}

void Drivetrain::Periodic() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t{m_leftEncoder.GetDistance()},
                    units::meter_t{m_rightEncoder.GetDistance()});
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
}

void Drivetrain::SimulationPeriodic() {
  m_driveSim.SetInputs(
      m_leftGroup.Get() * frc::RobotController::GetBatteryVoltage(),
      m_rightGroup.Get() * frc::RobotController::GetBatteryVoltage());
  m_driveSim.Update(20_ms);

  m_leftEncoderSim.SetDistance(m_driveSim.GetLeftPosition().value());
  m_rightEncoderSim.SetDistance(m_driveSim.GetRightPosition().value());
  m_leftEncoderSim.SetRate(m_driveSim.GetLeftVelocity().value());
  m_rightEncoderSim.SetRate(m_driveSim.GetRightVelocity().value());

  m_gyroSim.SetAngle(-m_driveSim.GetHeading().Degrees());
}

void Drivetrain::ArcadeDrive(double xSpeed, double zRotation,
                             bool squareInputs) {
  m_drive.ArcadeDrive(xSpeed, zRotation, squareInputs);
}

frc::Pose2d Drivetrain::GetPose() {
  return m_odometry.GetPose();
}

frc::Field2d* Drivetrain::GetField() {
  return &m_fieldSim;
}