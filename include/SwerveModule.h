// Copyright (c) 2023 FRC Team 4918.
// Some software is also Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

// Maybe remove later
//  #include <frc/motorcontrol/PWMSparkMax.h>
#include <rev/CANSparkMax.h>
#include <frc/AnalogInput.h>

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

class SwerveModule
{
public:
   /*
   SwerveModule(int driveMotorChannel, int turningMotorChannel,
                int driveEncoderChannelA, int driveEncoderChannelB,
                int turningEncoderChannelA, int turningEncoderChannelB);
     */

   SwerveModule(int driveMotorCanID,
                int turningMotorCanID,
                int turningEncoderSlot,
                int turningEncoderOffset);

   // commented out 1/13 MM - doesn't appear to be called anywhere
   // frc::SwerveModuleState GetState() const;
   frc::SwerveModulePosition GetPosition() const;
   void SetDesiredState( const frc::SwerveModuleState &state,
                         bool bFreezeDriveMotor = false );

private:
   static constexpr double kWheelRadius = 0.319;
   // CHANGE THIS LATER ^^^^
   static constexpr int kEncoderResolution = 4096;

   static constexpr auto kModuleMaxAngularVelocity =
       std::numbers::pi * 10_rad_per_s; // radians per second
   static constexpr auto kModuleMaxAngularAcceleration =
       std::numbers::pi * 20_rad_per_s / 1_s; // radians per second^2

   rev::CANSparkMax m_driveMotor;
   rev::CANSparkMax m_turningMotor;

   //   frc::Encoder m_driveEncoder;
   rev::SparkMaxRelativeEncoder m_driveEncoder;
   frc::AnalogInput m_turningEncoder;
   int m_turningEncoderOffset;

   frc::PIDController m_drivePIDController{
       2.0,
       0.0,
       0.0};

   frc::ProfiledPIDController<units::radians> m_turningPIDController{
       // 8.0,  //[Something] too violent, 5 too slow
       // 0.0,
       // 0.001,
       // Values above are good for all 4 swerve module turn motors
       8.0,
       0.0,
       0.001,
       {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
                                                                    // was 1.0
   frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0.1_V,
                                                                1_V / 1_mps};
   frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
       0.2_V /*was 1.0*/, 0.025_V / 1_rad_per_s};
};
// originally .5_V on line 72 made 0.025_V and 3_V on line 70 made 1_V
