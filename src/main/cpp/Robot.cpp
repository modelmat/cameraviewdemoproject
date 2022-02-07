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
#include <networktables/NetworkTableInstance.h>

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
    nt::NetworkTableInstance::GetDefault().Flush();
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
    for (auto point : m_pointsRelative) {
      auto currentPoint = previousPoint + point;
      m_pointsAbsolute_point3D.push_back(currentPoint.ToPoint3d());
      previousPoint = currentPoint;
    }
  }

  void ProjectPoints() {
    /* Camera frame is z forward, x right, y down
     *
     *     7 Z-axis
     *    /
     *   /
     *  +-----------> X-axis
     *  |
     *  |
     *  V Y-axis
     *
     * World frame is x (long side), y (short side), z up.
     * 
     *                        7  X-axis
     *                       /
     *             /--------/
     *            / Z-axis /
     *           /     ^  /
     *          /      | /
     *         /       |/
     * Y-axis <--------+ 
     *
     * To convert from world to camera we rotate the axes clockwise by pi/2
     * around the x axis and then anticlockwise pi/2 around the y axis, using 
     * https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations.
     * 
     * i.e. camera [x,y,z]ᵀ = Rᵧ(-π/2) Rₓ(π/2) ( world [x, y, z]ᵀ )
     * 
     * Note: Rotating anticlockwise is given by the direction fingers curl
     *       in the right hand grip rule, when the thumb pointed in direction
     *       of the axis to be rotated about (looking from neg to pos).
     * 
     */
    double Rx_data[3][3] = {
      {1, 0, 0},
      {0, 0, -1},
      {0, 1, 0}
    };
    cv::Mat Rx{3, 3, CV_64F, Rx_data};

    double Ry_data[3][3] = {
      {0, 0, -1},
      {0, 1, 0},
      {1, 0, 0}
    };
    cv::Mat Ry{3, 3, CV_64F, Ry_data};

    using units::math::cos;
    using units::math::sin;

    // Since y is down, we want to rotate axes clockwise, so that angle from
    // Pose2d (-π, π] matches the WPILib convention of CCW positive.
    auto cameraYawOffset = 0_deg;
    auto theta_yaw = m_drive.GetPose().Rotation().Radians() + cameraYawOffset;
    double yawData[3][3] = {
      {cos(theta_yaw), 0, sin(theta_yaw)},
      {0, 1, 0},
      {-sin(theta_yaw), 0, cos(theta_yaw)}
    };
    cv::Mat yaw{3, 3, CV_64F, yawData};
    auto theta_tilt = 45_deg;
    double tiltData[3][3] = {
      {1, 0, 0},
      {0, cos(-theta_tilt), -sin(-theta_tilt)},
      {0, sin(-theta_tilt), cos(-theta_tilt)}
    };
    cv::Mat cameraTilt{3, 3, CV_64F, tiltData};
   
    cv::Mat R = cameraTilt * yaw * Ry * Rx;
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    // https://en.wikipedia.org/wiki/Camera_resectioning#Extrinsic_parameters
    // The position, C of the camera from the world origin is -Rᵀ T;
    // T is the position of origin from camera.
    // C = -R^(-1)T
    // CR = -T
    // T = -R C

    // 0_m is camera height. transform is in world coordinates
    cv::Mat cameraInWorldFrame{Translation3d::FromXYPose(m_drive.GetPose(), 0_m).ToPoint3d()};
    // tvec is in the camera frame
    cv::Mat tvec = - R * cameraInWorldFrame;

    /* "Clean" Camera
    cv::Mat distortion = cv::Mat::zeros(5, 1, CV_32FC1);
    cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_32FC1);
    intrinsics.at<float>(0, 0) = 300.0f; // Focal Length, x
    intrinsics.at<float>(1, 1) = 300.0f; // Focal Length, y
    intrinsics.at<float>(0, 2) = kImageWidth / 2; // Optical Centre, x
    intrinsics.at<float>(1, 2) = kImageHeight / 2; // Optical Centre, y
    */

    // Data from: https://github.com/PhotonVision/photonvision/blob/master/test-resources/calibration/lifecam480p.json
    double distortionData[5] = {0.14382207979312617,
                                 -0.9851192814987014,
                                 -0.018168751047242335,
                                 0.011034504043795105,
                                 1.9833437176538498};
    cv::Mat distortion{5, 1, CV_64F, distortionData};
    double instrinsicsData[3][3] = {
      {699.3778103158814, 0, 345.6059345433618},
      {0, 677.7161226393544, 207.12741326228522},
      {0, 0, 1}
    };
    cv::Mat intrinsics{3, 3, CV_64F, instrinsicsData};

    std::vector<cv::Point3d> filteredPoints{};
    for (auto point : m_pointsAbsolute_point3D) {
      cv::Mat pointInCameraFrame = R * cv::Mat{point} + tvec;
      // Get the z (forward) axis. If behind camera, don't use.
      // TODO: Deal with the disappearing points graphically
      if (pointInCameraFrame.at<double>(2, 0) > 0) {
          filteredPoints.push_back(point);
      }
    }

    std::vector<cv::Point2d> projectedPoints;
    if (filteredPoints.size() == 0) {
      projectedPoints = std::vector<cv::Point2d>(m_pointsAbsolute_point3D.size(), cv::Point2d{0, 0});
    } else {
      cv::projectPoints(filteredPoints,   // objectPoints
                        rvec,             // rotation vec
                        tvec,             // translation vec
                        intrinsics,       // cameraMatrix
                        distortion,       // distortion cofeficients
                        projectedPoints   // output
      );
    }


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

  static constexpr int kImageWidth = 640;
  static constexpr int kImageHeight = 480;

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

  static constexpr units::meter_t kGoalFloorHeight = 6_ft + 9.25_in;

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
  std::vector<Translation3d> m_pointsRelative{
      {0_m, 0_m, 0_m},
      {0_m, kGoalWidth / 4, -kGoalHalfHeight},
      {0_m, kGoalWidth / 2, 0_m},
      {0_m, kGoalWidth / 4, +kGoalHalfHeight},
      {0_m, -kTapeWidth,          0_m},
      {0_m, -kInnerGoalWidth / 4, -kInnerGoalHalfHeight},
      {0_m, -kInnerGoalWidth / 2, 0_m},
      {0_m, -kInnerGoalWidth / 4, +kInnerGoalHalfHeight},
      {0_m, -kTapeWidth,          0_m}};
  std::vector<cv::Point3d> m_pointsAbsolute_point3D{};
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
