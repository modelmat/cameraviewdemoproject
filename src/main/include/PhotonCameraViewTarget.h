// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string_view>

#include <networktables/NetworkTableEntry.h>

#include "PhotonCameraViewObject.h"

/**
 * Root Mechanism2d node.
 *
 * A root is the anchor point of other nodes (such as ligaments).
 *
 * Do not create objects of this class directly! Obtain pointers from the
 * Mechanism2d.GetRoot() factory method.
 *
 * <p>Append other nodes by using Append().
 */
class PhotonCameraViewTarget : private PhotonCameraViewObject {
  friend class PhotonCameraView;
  struct private_init {};

 public:
  PhotonCameraViewTarget(std::string_view name, double x, double y,
                       const private_init&);

  /**
   * Set the root's position.
   *
   * @param x new x coordinate
   * @param y new y coordinate
   */
  void SetPosition(double x, double y);

  using PhotonCameraViewObject::GetName;

  using PhotonCameraViewObject::Append;

 private:
  void UpdateEntries(std::shared_ptr<nt::NetworkTable> table) override;
  inline void Flush();
  double m_x;
  double m_y;
  nt::NetworkTableEntry m_xEntry;
  nt::NetworkTableEntry m_yEntry;
};
