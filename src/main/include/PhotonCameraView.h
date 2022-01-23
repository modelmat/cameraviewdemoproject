// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string>

#include <networktables/NTSendable.h>
#include <networktables/NetworkTableEntry.h>
#include <wpi/StringMap.h>
#include <wpi/mutex.h>
#include <wpi/sendable/SendableHelper.h>

#include <frc/util/Color8Bit.h>

#include "PhotonCameraViewTarget.h"


class PhotonCameraView : public nt::NTSendable,
                         public wpi::SendableHelper<PhotonCameraView> {
 public:
  /**
   * Create a new Mechanism2d with the given dimensions and background color.
   *
   * The dimensions represent the canvas that all the nodes are drawn on. The
   * default color is dark blue.
   *
   * @param width the width
   * @param height the height
   * @param backgroundColor the background color
   */
  PhotonCameraView(double width, double height,
                   const frc::Color8Bit& backgroundColor = {0, 0, 32});

  /**
   * Get or create a root in this Mechanism2d with the given name and
   * position.
   *
   * <p>If a root with the given name already exists, the given x and y
   * coordinates are not used.
   *
   * @param name the root name
   * @param x the root x coordinate
   * @param y the root y coordinate
   * @return a new root object, or the existing one with the given name.
   */
  PhotonCameraViewTarget* GetRoot(std::string_view name, double x, double y);

  /**
   * Set the Mechanism2d background color.
   *
   * @param color the new background color
   */
  void SetBackgroundColor(const frc::Color8Bit& color);

  void InitSendable(nt::NTSendableBuilder& builder) override;

 private:
  double m_width;
  double m_height;
  char m_color[10];
  mutable wpi::mutex m_mutex;
  std::shared_ptr<nt::NetworkTable> m_table;
  wpi::StringMap<std::unique_ptr<PhotonCameraViewTarget>> m_roots;
};
