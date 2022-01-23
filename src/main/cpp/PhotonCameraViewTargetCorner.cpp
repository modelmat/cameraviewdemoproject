// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PhotonCameraViewTargetCorner.h"

#include <cstdio>

PhotonCameraViewTargetCorner::PhotonCameraViewTargetCorner(std::string_view name, double x,
                               double y, double lineWeight,
                               const frc::Color8Bit& color)
    : PhotonCameraViewObject(name),
      m_x{x},
      m_y{y},
      m_weight{lineWeight} {
  SetColor(color);
}

void PhotonCameraViewTargetCorner::UpdateEntries(
    std::shared_ptr<nt::NetworkTable> table) {
  table->GetEntry(".type").SetString("connectedPoint");

  m_colorEntry = table->GetEntry("color");
  m_weightEntry = table->GetEntry("weight");
  m_xEntry = table->GetEntry("x");
  m_yEntry = table->GetEntry("y");
  Flush();
}

void PhotonCameraViewTargetCorner::SetColor(const frc::Color8Bit& color) {
  std::scoped_lock lock(m_mutex);
  std::snprintf(m_color, sizeof(m_color), "#%02X%02X%02X", color.red,
                color.green, color.blue);
  Flush();
}

void PhotonCameraViewTargetCorner::SetPosition(double x, double y) {
  SetX(x);
  SetY(y);
}

void PhotonCameraViewTargetCorner::SetX(double x) {
  std::scoped_lock lock(m_mutex);
  m_x = x;
  Flush();
}

void PhotonCameraViewTargetCorner::SetY(double y) {
  std::scoped_lock lock(m_mutex);
  m_y = y;
  Flush();
}


void PhotonCameraViewTargetCorner::SetLineWeight(double lineWidth) {
  std::scoped_lock lock(m_mutex);
  m_weight = lineWidth;
  Flush();
}

double PhotonCameraViewTargetCorner::GetX() {
  if (m_xEntry) {
    m_x = m_xEntry.GetDouble(0.0);
  }
  return m_x;
}

double PhotonCameraViewTargetCorner::GetY() {
  if (m_yEntry) {
    m_y = m_yEntry.GetDouble(0.0);
  }
  return m_y;
}

#define SAFE_WRITE(data, Type)           \
  if (m_##data##Entry) {                 \
    m_##data##Entry.Set##Type(m_##data); \
  }
void PhotonCameraViewTargetCorner::Flush() {
  SAFE_WRITE(color, String)
  SAFE_WRITE(x, Double)
  SAFE_WRITE(y, Double)
  SAFE_WRITE(weight, Double)
}
