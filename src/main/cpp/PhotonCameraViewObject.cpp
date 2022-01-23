// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PhotonCameraViewObject.h"

PhotonCameraViewObject::PhotonCameraViewObject(std::string_view name) : m_name{name} {}

const std::string& PhotonCameraViewObject::GetName() const {
  return m_name;
}

void PhotonCameraViewObject::Update(std::shared_ptr<nt::NetworkTable> table) {
  std::scoped_lock lock(m_mutex);
  m_table = table;
  UpdateEntries(m_table);
  for (const wpi::StringMapEntry<std::unique_ptr<PhotonCameraViewObject>>& entry :
       m_objects) {
    entry.getValue()->Update(m_table->GetSubTable(entry.getKey()));
  }
}
