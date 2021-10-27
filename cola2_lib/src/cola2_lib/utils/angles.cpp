/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/utils/angles.h"
#include <cassert>
#include <cmath>

namespace cola2
{
namespace utils
{
double wrapAngle(const double angle)
{
  return (angle + (2.0 * M_PI * std::floor((M_PI - angle) / (2.0 * M_PI))));
}

double wrapAnglePositive(const double angle)
{
  double wrapped_angle = wrapAngle(angle);
  if (wrapped_angle < 0.0)
    wrapped_angle += 2.0 * M_PI;
  return wrapped_angle;
}

double wrapAngleDegrees(const double angle_deg)
{
  return radiansToDegrees(wrapAngle(degreesToRadians(angle_deg)));
}

double wrapAnglePositiveDegrees(const double angle_deg)
{
  return radiansToDegrees(wrapAnglePositive(degreesToRadians(angle_deg)));
}

//*****************************************************************************
// Angle unit conversions
//*****************************************************************************

double degreesToRadians(const double degrees)
{
  return (degrees / 180.0) * M_PI;
}

double radiansToDegrees(const double radians)
{
  return (radians / M_PI) * 180.0;
}

double gradiansToRadians(const double value)
{
  return value * M_PI / 200.0;
}

double radiansToGradians(const double value)
{
  return value * 200.0 / M_PI;
}

double degreeMinutesToDegrees(const double degree_minutes, const char hemisphere)
{
  assert(hemisphere == 'N' || hemisphere == 'S' || hemisphere == 'E' || hemisphere == 'W');

  unsigned int degrees = static_cast<unsigned int>(degree_minutes / 100);
  double minutes = degree_minutes - (degrees * 100);

  if (hemisphere == 'E' || hemisphere == 'N')
  {
    return degrees + (minutes / 60.0);
  }
  else if (hemisphere == 'W' || hemisphere == 'S')
  {
    return -(degrees + (minutes / 60.0));
  }
  return 0.0;
}

double degreeMinutesToDegreesInt(const double degree_minutes, const int hemisphere)
{
  switch (hemisphere)
  {
    case 0:
      return degreeMinutesToDegrees(degree_minutes, static_cast<char>('N'));
    case 1:
      return degreeMinutesToDegrees(degree_minutes, static_cast<char>('S'));
    case 2:
      return degreeMinutesToDegrees(degree_minutes, static_cast<char>('W'));
    case 3:
      return degreeMinutesToDegrees(degree_minutes, static_cast<char>('E'));
    default:
      // std::cerr << "Invalid hemisphere: " << hemisphere << "\n";
      return 0.0;
  }
}

//*****************************************************************************
// Quaternion-Euler conversions
//*****************************************************************************

// From the Matrix 3x3 Class of tf package
Eigen::Vector3d rotation2euler(const Eigen::Matrix3d &rotation, unsigned int solution_number)
{
  Eigen::Vector3d rpy;

  struct Euler
  {
    double yaw;
    double pitch;
    double roll;
  };

  Euler euler_out;
  Euler euler_out2;  // second solution

  // Check that pitch is not at a singularity
  if (std::fabs(rotation(2, 0)) >= 1)
  {
    euler_out.yaw = 0;
    euler_out2.yaw = 0;

    // From difference of angles formula
    if (rotation(2, 0) < 0)  // gimbal locked down
    {
      double delta = std::atan2(rotation(0, 1), rotation(0, 2));
      euler_out.pitch = M_PI / 2.0;
      euler_out2.pitch = M_PI / 2.0;
      euler_out.roll = delta;
      euler_out2.roll = delta;
    }
    else  // gimbal locked up
    {
      double delta = std::atan2(-rotation(0, 1), -rotation(0, 2));
      euler_out.pitch = -M_PI / 2.0;
      euler_out2.pitch = -M_PI / 2.0;
      euler_out.roll = delta;
      euler_out2.roll = delta;
    }
  }
  else
  {
    euler_out.pitch = -std::asin(rotation(2, 0));
    euler_out2.pitch = M_PI - euler_out.pitch;

    euler_out.roll = std::atan2(rotation(2, 1) / std::cos(euler_out.pitch), rotation(2, 2) / std::cos(euler_out.pitch));
    euler_out2.roll =
        std::atan2(rotation(2, 1) / std::cos(euler_out2.pitch), rotation(2, 2) / std::cos(euler_out2.pitch));

    euler_out.yaw = std::atan2(rotation(1, 0) / std::cos(euler_out.pitch), rotation(0, 0) / std::cos(euler_out.pitch));
    euler_out2.yaw =
        std::atan2(rotation(1, 0) / std::cos(euler_out2.pitch), rotation(0, 0) / std::cos(euler_out2.pitch));
  }
  if (solution_number == 1)
  {
    rpy(0) = euler_out.roll;
    rpy(1) = euler_out.pitch;
    rpy(2) = euler_out.yaw;
  }
  else
  {
    rpy(0) = euler_out2.roll;
    rpy(1) = euler_out2.pitch;
    rpy(2) = euler_out2.yaw;
  }

  return rpy;
}

Eigen::Matrix3d euler2rotation(const Eigen::Vector3d &rpy)
{
  return Eigen::Matrix3d(euler2quaternion(rpy));
}

Eigen::Matrix3d euler2rotation(const double roll, const double pitch, const double yaw)
{
  return Eigen::Matrix3d(euler2quaternion(Eigen::Vector3d(roll, pitch, yaw)));
}

Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &quat, unsigned int solution_number)
{
  return rotation2euler(quat.toRotationMatrix(), solution_number);
}

Eigen::Quaterniond euler2quaternion(const double roll, const double pitch, const double yaw)
{
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
}

Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d &rpy)
{
  return euler2quaternion(rpy(0), rpy(1), rpy(2));
}

//*****************************************************************************
// Rotation derivatives
//*****************************************************************************
Eigen::Matrix3d d_rotation_d_roll(const Eigen::Vector3d &rpy)
{
  const double sr = std::sin(rpy(0));
  const double sp = std::sin(rpy(1));
  const double sy = std::sin(rpy(2));
  const double cr = std::cos(rpy(0));
  const double cp = std::cos(rpy(1));
  const double cy = std::cos(rpy(2));
  Eigen::Matrix3d res;
  // first row
  res(0, 0) = 0.0;
  res(0, 1) = cy * sp * cr + sy * sr;
  res(0, 2) = -cy * sp * sr + sy * cr;
  // second row
  res(1, 0) = 0.0;
  res(1, 1) = sy * sp * cr - cy * sr;
  res(1, 2) = -sy * sp * sr - cy * cr;
  // third row
  res(2, 0) = 0.0;
  res(2, 1) = cp * cr;
  res(2, 2) = -cp * sr;
  // return
  return res;
}

Eigen::Matrix3d d_rotation_d_pitch(const Eigen::Vector3d &rpy)
{
  const double sr = std::sin(rpy(0));
  const double sp = std::sin(rpy(1));
  const double sy = std::sin(rpy(2));
  const double cr = std::cos(rpy(0));
  const double cp = std::cos(rpy(1));
  const double cy = std::cos(rpy(2));
  Eigen::Matrix3d res;
  // first row
  res(0, 0) = -cy * sp;
  res(0, 1) = cy * cp * sr;
  res(0, 2) = cy * cp * cr;
  // second row
  res(1, 0) = -sy * sp;
  res(1, 1) = sy * cp * sr;
  res(1, 2) = sy * cp * cr;
  // third row
  res(2, 0) = -cp;
  res(2, 1) = -sp * sr;
  res(2, 2) = -sp * cr;
  // return
  return res;
}

Eigen::Matrix3d d_rotation_d_yaw(const Eigen::Vector3d &rpy)
{
  const double sr = std::sin(rpy(0));
  const double sp = std::sin(rpy(1));
  const double sy = std::sin(rpy(2));
  const double cr = std::cos(rpy(0));
  const double cp = std::cos(rpy(1));
  const double cy = std::cos(rpy(2));
  Eigen::Matrix3d res;
  // first row
  res(0, 0) = -sy * cp;
  res(0, 1) = -sy * sp * sr - cy * cr;
  res(0, 2) = -sy * sp * cr + cy * sr;
  // second row
  res(1, 0) = cy * cp;
  res(1, 1) = cy * sp * sr - sy * cr;
  res(1, 2) = cy * sp * cr + sy * sr;
  // third row
  res(2, 0) = 0.0;
  res(2, 1) = 0.0;
  res(2, 2) = 0.0;
  // return
  return res;
}

//*****************************************************************************
// Cross-product
//*****************************************************************************
Eigen::Matrix3d cross_product_matrix(const Eigen::Vector3d &abc)
{
  Eigen::Matrix3d res = Eigen::Matrix3d::Zero();
  // a
  res(1, 2) = -abc(0);
  res(2, 1) = abc(0);
  // b
  res(0, 2) = abc(0);
  res(2, 0) = -abc(0);
  // c
  res(0, 1) = -abc(0);
  res(1, 0) = abc(0);
  return res;
}

}  // namespace utils
}  // namespace cola2
