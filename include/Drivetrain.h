// Copyright (c) 2023 FRC Team 4918.
// Some software is also Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <ctre/phoenix/sensors/WPI_PigeonIMU.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain
{
public:
   Drivetrain() { m_gyro.Reset(); }

   void Reset(void);

   void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed,
	      units::radians_per_second_t rot,
              bool fieldRelative,
              bool bFreezeDriveMotors = false );
   bool DriveUphill( units::meters_per_second_t sSpeed );
   void UpdateOdometry();

        // Both of the below "maximums" are about twice what the robot
        // can actually do (about 3 meters/second and about 1 rotation/second)
   static constexpr units::meters_per_second_t kMaxSpeed =
       5.5_mps; // about 5.5 meters per second
   static constexpr units::radians_per_second_t kMaxAngularSpeed{
      4.0*std::numbers::pi}; // 2 full rotations per second

private:

#ifdef JAG_PREVIOUS_EXPERIMENTAL_ROBOT
       // These values worked well in our experimental swerve robot:
   frc::Translation2d m_frontLeftLocation{+0.26_m, +0.26_m};
   frc::Translation2d m_frontRightLocation{+0.26_m, -0.26_m};
   frc::Translation2d m_backLeftLocation{-0.26_m, +0.26_m};
   frc::Translation2d m_backRightLocation{-0.26_m, -0.26_m};
#else
       // For the 2023 robot:
       // The fore-and-aft length between turning swerve axles is 25.5"
       // and the side-to-side width between turning swerve axles is 19.5"
       // In metric, those are 0.647 meters and 0.495 meters, and half
       // of those values is: 0.324 meters and 0.248 meters.
   frc::Translation2d m_frontLeftLocation{ +0.324_m, +0.248_m};
   frc::Translation2d m_frontRightLocation{+0.324_m, -0.248_m};
   frc::Translation2d m_backLeftLocation{  -0.324_m, +0.248_m};
   frc::Translation2d m_backRightLocation{ -0.324_m, -0.248_m};
#endif

#ifdef JAG_PREVIOUS_EXPERIMENTAL_ROBOT
   SwerveModule m_frontLeft{8, 9, 0, 36};
   SwerveModule m_frontRight{10, 11, 1, 34};
   SwerveModule m_backLeft{14, 15, 3, 24};
   SwerveModule m_backRight{12, 13, 2, 150};
#else
       // For the 2023 robot, the swerve modules got re-ordered
       // (the front-left and front-right modules got swapped):
   SwerveModule m_frontLeft{ 10, 11, 1, 304};
   SwerveModule m_frontRight{ 8,  9, 0, 126};
   SwerveModule m_backLeft{  14, 15, 3,  24};
   SwerveModule m_backRight{ 12, 13, 2, 150};
#endif

   // frc::AnalogGyro m_gyro{0};
   ctre::phoenix::sensors::WPI_PigeonIMU m_gyro{1};

   frc::SwerveDriveKinematics<4> m_kinematics{
       m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
       m_backRightLocation};

public:
  //  jag; frc::SwerveDriveOdometry<4> m_odometry{
  //  jag;     m_kinematics,
  //  jag;     m_gyro.GetRotation2d(),
  //  jag;     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
  //  jag;      m_backLeft.GetPosition(), m_backRight.GetPosition()}};
  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
      m_kinematics,
      frc::Rotation2d{},
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 0.1}};

   int iCallCount = 0;
};
