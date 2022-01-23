// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PhotonCameraView.h"

#include <cstdio>

#include <networktables/NTSendableBuilder.h>

static constexpr char kBackgroundColor[] = "backgroundColor";
static constexpr char kDims[] = "dims";

PhotonCameraView::PhotonCameraView(double width, double height,
                                   const frc::Color8Bit& backgroundColor)
    : m_width{width}, m_height{height} {
  SetBackgroundColor(backgroundColor);
}

PhotonCameraViewTarget* PhotonCameraView::GetRoot(std::string_view name, double x,
                                                double y) {
  auto& obj = m_roots[name];
  if (obj) {
    return obj.get();
  }
  obj = std::make_unique<PhotonCameraViewTarget>(name, x, y,
                                          PhotonCameraViewTarget::private_init{});
  if (m_table) {
    obj->Update(m_table->GetSubTable(name));
  }
  return obj.get();
}

void PhotonCameraView::SetBackgroundColor(const frc::Color8Bit& color) {
  std::snprintf(m_color, sizeof(m_color), "#%02X%02X%02X", color.red,
                color.green, color.blue);
  if (m_table) {
    m_table->GetEntry(kBackgroundColor).SetString(m_color);
  }
}

void PhotonCameraView::InitSendable(nt::NTSendableBuilder& builder) {
  builder.SetSmartDashboardType("PhotonCamera");

  std::scoped_lock lock(m_mutex);
  m_table = builder.GetTable();
  m_table->GetEntry(kDims).SetDoubleArray({m_width, m_height});
  m_table->GetEntry(kBackgroundColor).SetString(m_color);
  for (const auto& entry : m_roots) {
    const auto& root = entry.getValue().get();
    root->Update(m_table->GetSubTable(entry.getKey()));
  }
}
