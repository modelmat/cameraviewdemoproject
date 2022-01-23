// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PhotonCameraViewTarget.h"

#include <frc/util/Color8Bit.h>

using namespace frc;

PhotonCameraViewTarget::PhotonCameraViewTarget(std::string_view name, double x, double y,
                                           const private_init&)
    : PhotonCameraViewObject(name), m_x{x}, m_y{y} {}

void PhotonCameraViewTarget::SetPosition(double x, double y) {
  std::scoped_lock lock(m_mutex);
  m_x = x;
  m_y = y;
  Flush();
}

void PhotonCameraViewTarget::UpdateEntries(std::shared_ptr<nt::NetworkTable> table) {
  m_xEntry = table->GetEntry("x");
  m_yEntry = table->GetEntry("y");
  Flush();
}

inline void PhotonCameraViewTarget::Flush() {
  if (m_xEntry) {
    m_xEntry.SetDouble(m_x);
  }
  if (m_yEntry) {
    m_yEntry.SetDouble(m_y);
  }
}
