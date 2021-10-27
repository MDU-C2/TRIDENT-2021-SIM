/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains functions to manipulate angles.
 */

#pragma once

#include <eigen3/Eigen/Geometry>

namespace cola2
{
namespace utils
{
/**
 * @addtogroup utils
 * @{
 */

/**
 * @brief Wraps an angle (in radians) between [-PI,PI].
 *
 * @param angle Input angle in radians.
 * @return Angle converted to [-PI,PI].
 */
double wrapAngle(const double angle);

/**
 * @brief Wraps an angle (in radians) between [0,2*PI].
 *
 * @param angle Input angle in radians.
 * @return Angle converted to [0,2*PI].
 */
double wrapAnglePositive(const double angle);

/**
 * @brief Wraps an angle (in degrees) between [-180.0,180.0].
 *
 * @param angle_deg Input angle in degrees.
 * @return Angle converted to [-180.0,180.0].
 */
double wrapAngleDegrees(const double angle_deg);

/**
 * @brief Wraps an angle (in degrees) between [0,360.0].
 *
 * @param angle_deg Input angle in degrees.
 * @return Angle converted to [0,360.0].
 */
double wrapAnglePositiveDegrees(const double angle_deg);

//*****************************************************************************
// Angle unit conversions
//*****************************************************************************
/**
 * @brief Converts angle from degrees to radians.
 *
 * @param value Input angle in degrees.
 * @return Angle in radians.
 */
double degreesToRadians(const double value);

/**
 * Converts angle from radians to degrees.
 *
 * @param value Input angle in radians.
 * @return Angle in degrees.
 */
double radiansToDegrees(const double value);

/**
 * @brief Converts angle from gradians to radians.
 *
 * Converts angle from gradians to radians. A gradian is equivalent to 1/400 of a turn, 9/10 of a degree, or PI/200 of
 * a radian.
 *
 * @param value Input angle in gradians.
 * @return Angle in radians.
 */
double gradiansToRadians(const double value);

/**
 * @brief Converts angle from radians to gradians.
 *
 * Converts angle from radians to gradians. A gradian is equivalent to 1/400 of a turn, 9/10 of a degree, or PI/200 of
 * a radian.
 *
 * @param value Input angle in radians.
 * @return Angle in gradians.
 */
double radiansToGradians(const double value);

/**
 * @brief Converts degree_minutes (DDMM.MM) to decimal degrees (DD.DD) taking into account the hemisphere passed as a
 * character.
 *
 * @param degree_minutes Degrees and minutes (DDMM.MM).
 * @param hemisphere Hemisphere ('N', 'S', 'E' or 'W').
 * @return Decimal degrees.
 */
double degreeMinutesToDegrees(const double degree_minutes, const char hemisphere);

/**
 * @brief Converts degree_minutes (DDMM.MM) to decimal degrees (DD.DD) taking into account the hemisphere passed as an
 * integer.
 *
 * @param degree_minutes Degrees and minutes (DDMM.MM).
 * @param hemisphere Hemisphere (0, 1, 2, or 3, corresponding to 'N', 'S', 'E' or 'W').
 * @return Decimal degrees.
 */
double degreeMinutesToDegreesInt(const double degree_minutes, const int hemisphere);

//*****************************************************************************
// Quaternion-Euler conversions
//*****************************************************************************
/**
 * @brief Get vector of Euler angles (roll, pitch, yaw) from rotation matrix.
 *
 * @param rotation Input rotation matrix.
 * @param solution_number Solution number (1 for normal pitch, different than 1 for inverted pitch).
 * @return Euler angles (roll, pitch, yaw).
 */
Eigen::Vector3d rotation2euler(const Eigen::Matrix3d &rotation, unsigned int solution_number = 1);

/**
 * @brief Get rotation matrix from vector of Euler angles (roll, pitch, yaw).
 *
 * @param rpy Euler angles (roll, pitch, yaw).
 * @return Rotation matrix.
 */
Eigen::Matrix3d euler2rotation(const Eigen::Vector3d &rpy);

/**
 * @brief Get rotation matrix from individual Euler angles (roll, pitch, yaw).
 *
 * @param roll Roll angle.
 * @param pitch Pitch angle.
 * @param yaw Yaw angle.
 * @return Rotation matrix.
 */
Eigen::Matrix3d euler2rotation(const double roll, const double pitch, const double yaw);

/**
 * @brief Get vector of Euler angles (roll, pitch, yaw) from quaternion.
 *
 * @param quat Input quaternion.
 * @param solution_number Solution number (1 for normal pitch, different than 1 for inverted pitch).
 * @return Euler angles (roll, pitch, yaw).
 */
Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &quat, unsigned int solution_number = 1);

/**
 * @brief Get quaternion from individual Euler angles (roll, pitch, yaw).
 *
 * @param roll Roll angle.
 * @param pitch Pitch angle.
 * @param yaw Yaw angle.
 * @return Quaternion.
 */
Eigen::Quaterniond euler2quaternion(const double roll, const double pitch, const double yaw);

/**
 * @brief Get quaternion matrix from Euler angles (roll, pitch, yaw).
 *
 * @param rpy Input Euler angles (roll, pitch, yaw).
 * @return Quaternion.
 */
Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d &rpy);

//*****************************************************************************
// Rotation derivatives
//*****************************************************************************
/**
 * @brief Compute the differentiation of the rotation matrix over roll.
 *
 * @param rpy Euler angles that describe the rotation.
 * @return Rotation matrix differentiated over roll and evaluated.
 */
Eigen::Matrix3d d_rotation_d_roll(const Eigen::Vector3d &rpy);

/**
 * @brief Compute the differentiation of the rotation matrix over pitch.
 *
 * @param rpy Euler angles that describe the rotation.
 * @return Rotation matrix differentiated over pitch and evaluated.
 */
Eigen::Matrix3d d_rotation_d_pitch(const Eigen::Vector3d &rpy);

/**
 * @brief Compute the differentiation of the rotation matrix over yaw.
 *
 * @param rpy Euler angles that describe the rotation.
 * @return Rotation matrix differentiated over yaw and evaluated.
 */
Eigen::Matrix3d d_rotation_d_yaw(const Eigen::Vector3d &rpy);

//*****************************************************************************
// Cross-product
//*****************************************************************************
/**
 * @brief Compute the cross-product matrix of a given vector.
 *
 * u x v = [u] * v = [v]T * u
 * The cross-product matrix is a skew-symmetry matrix such as MT = -M.
 *
 * @param abc Vector that we want to obtain the cross-product matrix.
 * @return Cross-product matrix of the vector.
 */
Eigen::Matrix3d cross_product_matrix(const Eigen::Vector3d &abc);

/** @} */
}  // namespace utils
}  // namespace cola2
