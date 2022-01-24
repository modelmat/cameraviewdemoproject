// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <opencv2/core/types.hpp>

#include <frc/geometry/Pose2d.h>


/**
 * Represents a translation in 3d space.
 * This object can be used to represent a point or a vector.
 *
 * This assumes that you are using conventional mathematical axes.
 * When the robot is placed on the origin, facing toward the X direction,
 * moving forward increases the X, whereas moving to the left increases the Y.
 */
class Translation3d {
 public:
  /**
   * Constructs a Translation3d with X and Y components equal to zero.
   */
  constexpr Translation3d() = default;

  /**
   * Constructs a Translation3d with the X, Y and Z components equal to the
   * provided values.
   *
   * @param x The x component of the translation.
   * @param y The y component of the translation.
   * @param z The z component of the translation.
   */
  Translation3d(units::meter_t x, units::meter_t y, units::meter_t z);

  static Translation3d FromXYPose(frc::Pose2d pose, units::meter_t z);

  /**
   * Calculates the distance between two translations in 3d space.
   *
   * This function uses the pythagorean theorem to calculate the distance.
   * distance = std::sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2)
   *
   * @param other The translation to compute the distance to.
   *
   * @return The distance between the two translations.
   */
  units::meter_t Distance(const Translation3d& other) const;

  /**
   * Returns the X component of the translation.
   *
   * @return The x component of the translation.
   */
  units::meter_t X() const { return m_x; }

  /**
   * Returns the Y component of the translation.
   *
   * @return The y component of the translation.
   */
  units::meter_t Y() const { return m_y; }

  /**
   * Returns the Z component of the translation.
   *
   * @return The z component of the translation.
   */
  units::meter_t Z() const { return m_z; }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  units::meter_t Norm() const;

  /**
   * Adds two translations in 3d space and returns the sum. This is similar to
   * vector addition.
   *
   * For example, Translation3d{1.0, 2.5, 4.3} + Translation3d{2.0, 5.5, 6.6} =
   * Translation3d{3.0, 8.0, 10.9}
   *
   * @param other The translation to add.
   *
   * @return The sum of the translations.
   */
  Translation3d operator+(const Translation3d& other) const;

  /**
   * Subtracts the other translation from the other translation and returns the
   * difference.
   *
   * For example, Translation3d{5.0, 4.0, 3.0} - Translation3d{1.0, 2.0, 3.0} =
   * Translation3d{4.0, 2.0, 0.0}
   *
   * @param other The translation to subtract.
   *
   * @return The difference between the two translations.
   */
  Translation3d operator-(const Translation3d& other) const;

  /**
   * Returns the inverse of the current translation. This is equivalent to
   * rotating by 180 degrees, flipping the point over both axes, or simply
   * negating both components of the translation.
   *
   * @return The inverse of the current translation.
   */
  Translation3d operator-() const;

  /**
   * Multiplies the translation by a scalar and returns the new translation.
   *
   * For example, Translation3d{2.0, 2.5, 3.0} * 2 = Translation3d{4.0, 5.0, 6.0}
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The scaled translation.
   */
  Translation3d operator*(double scalar) const;

  /**
   * Divides the translation by a scalar and returns the new translation.
   *
   * For example, Translation3d{2.0, 2.5, 3.0} / 2 = Translation3d{1.0, 1.25, 1.5}
   *
   * @param scalar The scalar to divide by.
   *
   * @return The scaled translation.
   */
  Translation3d operator/(double scalar) const;

  /**
   * Checks equality between this Translation3d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Translation3d& other) const;

  /**
   * Checks inequality between this Translation3d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Translation3d& other) const;

  cv::Point3d ToPoint3d() const;

 private:
  units::meter_t m_x = 0_m;
  units::meter_t m_y = 0_m;
  units::meter_t m_z = 0_m;
};
