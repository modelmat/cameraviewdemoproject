// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/simulation/ADXRS450_GyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <wpi/numbers>

class Drivetrain {
 public:
  Drivetrain();

  void Periodic();
  void SimulationPeriodic();

  void ArcadeDrive(double xSpeed, double zRotation, bool squareInputs = true);

  frc::Pose2d GetPose();
  frc::Field2d* GetField();

 private:
  // Constants
  static constexpr int kEncoderCPR = 4096;
  static constexpr units::meter_t kWheelDiameter = 6_in;
  static constexpr double kDrivetrainDistancePerPulse =
      wpi::numbers::pi * kWheelDiameter.value() / kEncoderCPR;

  // Motors
  frc::PWMSparkMax m_leftFront{1};
  frc::PWMSparkMax m_leftBack{2};
  frc::PWMSparkMax m_rightFront{3};
  frc::PWMSparkMax m_rightBack{4};

  frc::MotorControllerGroup m_leftGroup{m_leftFront, m_leftBack};
  frc::MotorControllerGroup m_rightGroup{m_rightFront, m_rightBack};

  frc::DifferentialDrive m_drive{m_leftGroup, m_rightGroup};

  // Sensors
  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};

  frc::ADXRS450_Gyro m_gyro{};

  // Kinematics
  frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

  // Simulation
  frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
  frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
  frc::sim::ADXRS450_GyroSim m_gyroSim{m_gyro};

  frc::sim::DifferentialDrivetrainSim m_driveSim =
      frc::sim::DifferentialDrivetrainSim::CreateKitbotSim(
          frc::DCMotor::CIM(2), 10.71, kWheelDiameter);

  frc::Field2d m_fieldSim;
};
