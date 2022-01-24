// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Translation3d.h"

#include <units/math.h>

Translation3d::Translation3d(units::meter_t x, units::meter_t y, units::meter_t z)
    : m_x(x), m_y(y), m_z(z) {}

Translation3d Translation3d::FromXYPose(frc::Pose2d pose, units::meter_t z) {
  return {pose.X(), pose.Y(), z};
}

units::meter_t Translation3d::Distance(const Translation3d& other) const {
  return (*this - other).Norm();
}

units::meter_t Translation3d::Norm() const {
  return units::math::sqrt(units::math::pow<2>(X()) + units::math::pow<2>(Y()) + units::math::pow<2>(Z()));
}

Translation3d Translation3d::operator+(const Translation3d& other) const {
  return {X() + other.X(), Y() + other.Y(), Z() + other.Z()};
}

Translation3d Translation3d::operator-(const Translation3d& other) const {
  return *this + -other;
}

Translation3d Translation3d::operator-() const {
  return {-m_x, -m_y, -m_z};
}

Translation3d Translation3d::operator*(double scalar) const {
  return {scalar * m_x, scalar * m_y, scalar * m_z};
}

Translation3d Translation3d::operator/(double scalar) const {
  return *this * (1.0 / scalar);
}

bool Translation3d::operator==(const Translation3d& other) const {
  return units::math::abs(m_x - other.m_x) < 1E-9_m &&
         units::math::abs(m_y - other.m_y) < 1E-9_m && 
         units::math::abs(m_z - other.m_z) < 1E-9_m;
}

bool Translation3d::operator!=(const Translation3d& other) const {
  return !operator==(other);
}

// TODO: Make autoconvetable
cv::Point3d Translation3d::ToPoint3d() const {
  return cv::Point3d(m_x.value(), m_y.value(), m_z.value());
}
