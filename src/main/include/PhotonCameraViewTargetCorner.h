// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string_view>

#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>

#include <frc/util/Color8Bit.h>

#include "PhotonCameraViewObject.h"


class PhotonCameraViewTargetCorner : public PhotonCameraViewObject {
 public:
  PhotonCameraViewTargetCorner(std::string_view name, double x,
                               double y, double lineWidth = 2,
                               const frc::Color8Bit& color = frc::Color::kForestGreen);

  void SetColor(const frc::Color8Bit& color);

  void SetPosition(double x, double y);
  void SetX(double x);
  void SetY(double y);

  double GetX();
  double GetY();

  /**
   * Set the line thickness.
   *
   * @param lineWidth the line thickness
   */
  void SetLineWeight(double lineWidth);

 protected:
  void UpdateEntries(std::shared_ptr<nt::NetworkTable> table) override;

 private:
  void Flush();
  double m_x;
  nt::NetworkTableEntry m_xEntry;
  double m_y;
  nt::NetworkTableEntry m_yEntry;
  double m_weight;
  nt::NetworkTableEntry m_weightEntry;
  char m_color[10];
  nt::NetworkTableEntry m_colorEntry;
};
