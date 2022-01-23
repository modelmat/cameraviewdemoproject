// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/AnalogPotentiometer.h>
#include <frc/Encoder.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/simulation/SimHooks.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <units/angle.h>

#include "PhotonCameraView.h"
#include "PhotonCameraViewTargetCorner.h"
#include "Translation3d.h"

#include <iostream>
#include <opencv2/calib3d.hpp>

#include "Drivetrain.h"
#include <units/angle.h>

/**
 * This sample program shows how to use Mechanism2d - a visual representation of
 * arms, elevators, and other mechanisms on dashboards; driven by a node-based
 * API.
 *
 * <p>Ligaments are based on other ligaments or roots, and roots are contained
 * in the base Mechanism2d object. Use pointers for nodes, and beware not to let
 * the container out of scope - the appended nodes will be recursively
 * destructed!
 */
class Robot : public frc::TimedRobot {
public:
  void RobotInit() override {
    // publish to dashboard
    frc::SmartDashboard::PutData("PhotonCameraView", &m_camera);

    // if issimulation
    ProjectPointsInit();
  }

  void RobotPeriodic() override {
    m_drive.Periodic();
  }

  void TeleopPeriodic() override { 
    m_drive.ArcadeDrive(-m_controller.GetLeftY(), m_controller.GetRightX());
  }

  void SimulationPeriodic() override {
    m_drive.SimulationPeriodic();
    ProjectPoints();

    m_fieldObject->SetPose(m_goalOrigin.X(), m_goalOrigin.Y(), frc::Rotation2d{0_deg});
  }

  void ProjectPointsInit() {
    Translation3d previousPoint = m_pointsOrigin;
    for (auto point : m_pointsRelPointsOrigin) {
      auto currentPoint = previousPoint + point;
      m_pointsOpenCVWorld.push_back(currentPoint.ToPoint3d());
      previousPoint = currentPoint;
    }
  }

  void ProjectPoints() {
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1);
// OpenCV has z forward, x right, y down
// My code / wpilib has x forward, y left, z up
// So z is x, x is -y, y is -z
// but this is also negated for some reason lol
    tvec.at<float>(0, 0) = m_drive.GetPose().Y().value();
    tvec.at<float>(1, 0) = 0; // camera height
    tvec.at<float>(2, 0) = -m_drive.GetPose().X().value();

    std::vector<double> rvec{0, m_drive.GetPose().Rotation().Radians().value(), 0};

    cv::Mat distortion = cv::Mat::zeros(5, 1, CV_32FC1);
    cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_32FC1);
    intrinsics.at<float>(0, 0) = 300.0f; // Focal Length, x
    intrinsics.at<float>(1, 1) = 300.0f; // Focal Length, y
    intrinsics.at<float>(0, 2) = kImageWidth / 2; // Optical Centre, x
    intrinsics.at<float>(1, 2) = kImageHeight / 2; // Optical Centre, y

    std::vector<cv::Point2d> projectedPoints;
    cv::projectPoints(m_pointsOpenCVWorld, // objectPoints
                      rvec,             // rotation vec
                      tvec,             // translation vec
                      intrinsics,       // cameraMatrix
                      distortion,       // distortion cofeficients
                      projectedPoints   // output
    );

    std::vector<cv::Point2d> projectedPointsBetterAxes = {};
    for (auto point : projectedPoints) {
      // OpenCV is top-left origin.
      // We use bottom-left origin
      projectedPointsBetterAxes.push_back({point.x, kImageHeight - point.y});
    }
    m_A->SetPosition(projectedPointsBetterAxes[0].x, projectedPointsBetterAxes[0].y);
    m_B->SetPosition(projectedPointsBetterAxes[1].x, projectedPointsBetterAxes[1].y);
    m_C->SetPosition(projectedPointsBetterAxes[2].x, projectedPointsBetterAxes[2].y);
    m_D->SetPosition(projectedPointsBetterAxes[3].x, projectedPointsBetterAxes[3].y);
    m_E->SetPosition(projectedPointsBetterAxes[4].x, projectedPointsBetterAxes[4].y);
    m_F->SetPosition(projectedPointsBetterAxes[5].x, projectedPointsBetterAxes[5].y);
    m_G->SetPosition(projectedPointsBetterAxes[6].x, projectedPointsBetterAxes[6].y);
    m_H->SetPosition(projectedPointsBetterAxes[7].x, projectedPointsBetterAxes[7].y);
    m_I->SetPosition(projectedPointsBetterAxes[8].x, projectedPointsBetterAxes[8].y);
  }

private:
  frc::XboxController m_controller{0};
  Drivetrain m_drive{};
  frc::FieldObject2d* m_fieldObject = m_drive.GetField()->GetObject("Goal");

  static constexpr int kImageWidth = 1280;
  static constexpr int kImageHeight = 720;

  PhotonCameraView m_camera{kImageWidth, kImageHeight};
  PhotonCameraViewTarget *m_A = m_camera.GetRoot("A", 0, 0);
  PhotonCameraViewTargetCorner *m_B =
      m_A->Append<PhotonCameraViewTargetCorner>("B", 0, 0);
  PhotonCameraViewTargetCorner *m_C =
      m_B->Append<PhotonCameraViewTargetCorner>("C", 0, 0);
  PhotonCameraViewTargetCorner *m_D =
      m_C->Append<PhotonCameraViewTargetCorner>("D", 0, 0);
  PhotonCameraViewTargetCorner *m_E =
      m_D->Append<PhotonCameraViewTargetCorner>("E", 0, 0);
  PhotonCameraViewTargetCorner *m_F =
      m_E->Append<PhotonCameraViewTargetCorner>("F", 0, 0);
  PhotonCameraViewTargetCorner *m_G =
      m_F->Append<PhotonCameraViewTargetCorner>("G", 0, 0);
  PhotonCameraViewTargetCorner *m_H =
      m_G->Append<PhotonCameraViewTargetCorner>("H", 0, 0);
  PhotonCameraViewTargetCorner *m_I =
      m_H->Append<PhotonCameraViewTargetCorner>("I", 0, 0);

  static constexpr units::meter_t kFieldLength = 52_ft + 5.25_in;
  static constexpr units::meter_t kFieldWidth = 26_ft + 11.25_in;

  static constexpr units::meter_t kGoalFloorHeight = /* 6_ft + 9.25_in */ 0_in;

  static constexpr units::meter_t kGoalWidth = 3_ft + 3.25_in;
  static constexpr units::meter_t kGoalHalfHeight = 1_ft + 5_in;

  static constexpr units::meter_t kTapeWidth = 2_in;
  static constexpr units::meter_t kInnerGoalWidth = 3_ft + 3.25_in - 2 * kTapeWidth;
  static constexpr units::meter_t kInnerGoalHalfHeight = kGoalHalfHeight * (kInnerGoalWidth) / kGoalWidth;

  Translation3d m_goalOrigin{
      kFieldLength, 2.4_m, kGoalFloorHeight + kGoalHalfHeight}; // Centre of goal
  Translation3d m_pointsOrigin{
      m_goalOrigin +
      Translation3d{0_m, -kGoalWidth / 2.0, 0_m}}; // Leftmost point
  // Relative to the previous point
  std::vector<Translation3d> m_pointsRelPointsOrigin{
      {0_m, 0_m, 0_m},
      {0_m, kGoalWidth / 4, -kGoalHalfHeight},
      {0_m, kGoalWidth / 2, 0_m},
      {0_m, kGoalWidth / 4, +kGoalHalfHeight},
      {0_m, -kTapeWidth,          0_m},
      {0_m, -kInnerGoalWidth / 4, -kInnerGoalHalfHeight},
      {0_m, -kInnerGoalWidth / 2, 0_m},
      {0_m, -kInnerGoalWidth / 4, +kInnerGoalHalfHeight},
      {0_m, -kTapeWidth,          0_m}};
  std::vector<cv::Point3d> m_pointsOpenCVWorld{};
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
