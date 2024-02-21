// Copyright (c) 2023 FRC Team 4918.
// Some software is also Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <iostream>
#include <iomanip>

#include <frc/Timer.h>

#include "ExampleGlobalMeasurementSensor.h"

using std::cout;
using std::endl;
using std::setw;
using std::setfill;         // so we can use "setfill('0') in cout streams
using std::abs;

void Drivetrain::Drive( units::meters_per_second_t xSpeed,
                        units::meters_per_second_t ySpeed,
                        units::radians_per_second_t rot,
                        bool fieldRelative,
                        bool bFreezeDriveMotors )
{
   auto states = m_kinematics.ToSwerveModuleStates(
       fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                           xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                     : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

   m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

   auto [fl, fr, bl, br] = states;

         // As a special case, if all drive arguments are 0.0, then cross
         // the swerve modules (put them all at 45 degrees, so the robot
         // won't move, and will be difficult to move (even if on a sloping
         // charging station, or bumped by another robot).
         // This happens only during autonomous;
         // if it is desired to make this happen during both autonomous and
         // teleop: comment out the "!fieldRelative" line, because
         // autonomous maneuvers call this function with that argument false
         // when they want to stop the robot completely, but teleop always
         // sets that argument true.
   if ( ( xSpeed < (units::meters_per_second_t)0.001          ) &&
        (         -(units::meters_per_second_t)0.001 < xSpeed ) &&
        ( ySpeed < (units::meters_per_second_t)0.001          ) &&
        (         -(units::meters_per_second_t)0.001 < ySpeed ) &&
        ( !fieldRelative                                      ) &&
        (    rot < (units::radians_per_second_t)0.001         ) &&
        (         -(units::radians_per_second_t)0.001 < rot   )    ) {
      fl.angle = (frc::Rotation2d)(units::degree_t)45.0;
      fr.angle = (frc::Rotation2d)(units::degree_t)-45.0;
      bl.angle = (frc::Rotation2d)(units::degree_t)-45.0;
      br.angle = (frc::Rotation2d)(units::degree_t)45.0;
   }

   if ( 0 == iCallCount%50 )
   {
      //      auto [distance0, angle0] = m_frontLeft.GetPosition();
      //      auto [distance1, angle1] = m_frontRight.GetPosition();
      //      auto [distance2, angle2] = m_backRight.GetPosition();
      //      auto [distance3, angle3] = m_backLeft.GetPosition();

      //      std::cout << "FrontLeft: " << distance0.value() << ", " <<
      //                                angle0.Degrees().value() << std::endl;
      //      std::cout << "FrontRight: " << distance1.value() << ", " <<
      //                                 angle1.Degrees().value() << std::endl;
      //      std::cout << "BackRight: " << distance2.value() << ", " <<
      //                                angle2.Degrees().value() << std::endl;
      //      std::cout << "BackLeft: " << distance3.value() << ", " <<
      //                                angle3.Degrees().value() << std::endl;

      //    std::cout << "Current Gyro Position: " << m_gyro.GetAngle()
      //              << std::endl;
   }
   iCallCount++;

   m_frontLeft.SetDesiredState(  fl, bFreezeDriveMotors );
   m_frontRight.SetDesiredState( fr, bFreezeDriveMotors );
   m_backLeft.SetDesiredState(   bl, bFreezeDriveMotors );
   m_backRight.SetDesiredState(  br, bFreezeDriveMotors );
}


bool Drivetrain::DriveUphill( units::meters_per_second_t sSpeed ) {
   bool bReturnValue = false;
   static int iCallCount = 0;

   static double currPitch = 0.0;
   static double currRawGyro_xyz_dps[3] = { 0.0, 0.0, 0.0 };
   // static double prevPitch = 0.0;
   // static double prevRawGyro_xyz_dps[3] = { 0.0, 0.0, 0.0 };

   // currPitch = m_gyro.GetPitch();              // get current pitch value
   // Previous line doesn't work because the gyro is mounted sideways, so we
   // have to use the roll axis of the gyro to find the pitch of the robot.
   currPitch = m_gyro.GetRoll();              // get current pitch value

   m_gyro.GetRawGyro( currRawGyro_xyz_dps );   // get pitch/roll/yaw rates

            // The equation below is intended to produce a drive speed which
            // goes at a reasonable rate toward the high direction (uphill)
            // If the robot is at a 16 degree tilt and not pitching, it will
            // produce a speed of 1 meter/second; if at the same pitch but
            // also pitching down at 4 degrees/second, the robot will stop.
            // If desired, we could use the sSpeed argument to this function
            // to give control of xSpeed to callers.
// Comment out Drive() temporarily, so the robot doesn't actually move
// and we can look at the Gyro and currRawGyro_zyz_dps[] cout outputs.
// #define JAG_DEBUG 1
#undef JAG_DEBUG
#ifdef JAG_DEBUG
   const auto xSpeed = (units::meters_per_second_t)0.0;
#else
   double currPitchRate;
            // these were 20.0 and -20.0 (and worked OK) until 19mar:
   currPitchRate = std::min(  30.0, currRawGyro_xyz_dps[1] );
   currPitchRate = std::max( -30.0, currPitchRate );
      // These numbers work OK, but need to be improved:
      //                   (currPitch/48.0 - currPitchRate/80.0);
// auto xSpeed = (units::meters_per_second_t)
//                         (currPitch/30.0 + currPitchRate/60.0); // oscillates
//          worked once:   (currPitch/24.0 + currPitchRate/50.0); // oscillates
   auto xSpeed = (units::meters_per_second_t)
                           (currPitch/20.0 + currPitchRate/40.0);
             // Don't go too extreme in speed, if pitch and pitchrate are both
             // in the same direction.  This probably means we went too far
             // across the charging station, and it started tipping back.
   if ( ( 0.0 < currPitch     && 0.0 < currPitchRate ) ||
        ( currPitchRate < 0.0 && currPitch     < 0.0 )    ) {
      xSpeed = (units::meters_per_second_t)(currPitch/20.0);
   }
   xSpeed = std::min(  Drivetrain::kMaxSpeed, xSpeed );
   xSpeed = std::max( -Drivetrain::kMaxSpeed, xSpeed );
          // Make sure robot never goes downhill strongly (that the pitch rate
          // correction factor never overcomes the pitch factor).
          // We allow a small (slow) amount of downhill travel, because the
          // charging station doesn't seem to start tipping until the robot
          // is too far, leading to continual back-and-forth rocking.
          // 0.2 worked well inside, with charging station on carpet;
          // 0.1 worked better with charging station on slick wood sliders
          // (lower friction), which is probably more similar to the real
          // charging station.
   if ( 0.0 < currPitch ) {             // make sure robot never goes downhill
      xSpeed = std::max( (units::meters_per_second_t)-0.1, xSpeed );
   } else {
      xSpeed = std::min( (units::meters_per_second_t)0.1, xSpeed );
   }
#endif
// if ( currRawGyro_xyz_dps[1] )

   if ( 1 == iCallCount%1000 ) {
      cout << "DriveUphill: Pitch: " << currPitch
           << " Rates: "
#ifdef JAG_NOTDEFINED
           << currRawGyro_xyz_dps[0] << "/"
#endif
           << currRawGyro_xyz_dps[1] << "/"
#ifdef JAG_NOTDEFINED
           << currRawGyro_xyz_dps[2]
#endif
           << " xSpeed: " << xSpeed.value() << endl;
   }

   if ( ( 5.0  < currPitch ) ||
        ( currPitch < -5.0 ) ) {
      Drive( (units::meters_per_second_t)xSpeed,
             (units::meters_per_second_t)0.0,
             (units::radians_per_second_t)0.0, false, false );
      bReturnValue = false;     // keep driving
   } else {
      Drive( (units::meters_per_second_t)0.0,
             (units::meters_per_second_t)0.0,
             (units::radians_per_second_t)0.0, false, false );
      bReturnValue = true;      // finished driving uphill; we are now flat
   }

   // prevPitch = currPitch;
   // prevRawGyro_xyz_dps[0] = currRawGyro_xyz_dps[0];
   // prevRawGyro_xyz_dps[1] = currRawGyro_xyz_dps[1];
   // prevRawGyro_xyz_dps[2] = currRawGyro_xyz_dps[2];

   iCallCount++;
   return bReturnValue;
}

void Drivetrain::Reset()
{
   m_gyro.Reset();
   usleep( 10000 );                                     // wait 10 milliseconds
   m_poseEstimator.ResetPosition( m_gyro.GetRotation2d(),
                     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_backLeft.GetPosition(),  m_backRight.GetPosition()  },
                                      { (units::meter_t)0.0,     // X on field
                                        (units::meter_t)0.0,     // Y on field
                                        (units::degree_t)0.0 } );  // rotation 
}

void Drivetrain::UpdateOdometry()
{
   static int iCallCount = 0;

   // m_odometry.Update(m_gyro.GetRotation2d(),
   //                   {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
   //                    m_backLeft.GetPosition(), m_backRight.GetPosition()});
   m_poseEstimator.Update(m_gyro.GetRotation2d(),
                     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_backLeft.GetPosition(),  m_backRight.GetPosition()  });
// if ( 0 == iCallCount%50 ) {
//    frc::Pose2d pose = m_poseEstimator.GetEstimatedPosition();
//    std::cout << "X: " << pose.X().value() << ", Y: " << pose.Y().value() <<
//                 ", Rot: " << pose.Rotation().Degrees().value() << std::endl;
// }
  iCallCount++;

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on
  // latency or timestamps.
//  m_poseEstimator.AddVisionMeasurement(
//      ExampleGlobalMeasurementSensor::GetEstimatedGlobalPose(
//          m_poseEstimator.GetEstimatedPosition()),
//      frc::Timer::GetFPGATimestamp() - 0.3_s);
}
