// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>

// Define JOYSTICK if this is for a joystick, rather than an XBox controller
// #define JOYSTICK 1
//
// Define SPARKMAXDRIVE if the drive motors will be controlled by SparkMaxes;
// if undefined they will be assumed to be TalonSRX controllers
// #define SPARKMAXDRIVE

#ifdef JOYSTICK
#include <frc/Joystick.h>
#else
#include <frc/XboxController.h>
#endif

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>

// The following include files, and the rest of the AprilTags detection code,
// are from the example program here:
// ~/wpilib/2024/vscode/VSCode-linux-x64/data/extensions/
//   wpilibsuite.vscode-wpilib-2024.2.1/resources/cpp/src/examples/
//   AprilTagsVision/cpp/Robot.cpp
#include <cstdio>
#include <span>
#include <sstream>
#include <string>
#include <thread>

#include <cameraserver/CameraServer.h>
#include <fmt/format.h>
#include <frc/TimedRobot.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <units/angle.h>
#include <units/length.h>

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/ADIS16470_IMU.h>


/*
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering (if JOYSTICK is defined), or
 * with split arcade steering and an Xbox controller.
 */

static double desiredYaw;
static double moveToAprilTag;
static units::angle::degree_t gyroYawHeading; //robot yaw (degrees)
static units::angular_velocity::degrees_per_second_t gyroYawRate; //robot rotate rate (degrees/second)

class Robot : public frc::TimedRobot {
#ifdef SPARKMAXDRIVE
  frc::PWMSparkMax m_LeftDriveMotor{     0 };
  frc::PWMSparkMax m_RightDriveMotor{    1 };
#else
  WPI_TalonSRX m_LeftDriveMotor{        3 };
  WPI_VictorSPX m_LeftDriveMotorRear{  11 };
  WPI_TalonSRX m_RightDriveMotor{      12 };
  WPI_VictorSPX m_RightDriveMotorRear{  4 };
#endif
  rev::CANSparkMax motor8{   8, rev::CANSparkMax::MotorType::kBrushed  };
  rev::CANSparkMax motor10{ 10, rev::CANSparkMax::MotorType::kBrushed  };
  rev::CANSparkMax motor11{ 11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax motor16{ 16, rev::CANSparkMax::MotorType::kBrushless};

  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m_LeftDriveMotor.Set(output); },
      [&](double output) { m_RightDriveMotor.Set(output); }};
#ifdef JOYSTICK
        frc::CameraServer::PutVideo("Detected", 640, 480);
  frc::Joystick m_stick{0};
#else
  frc::XboxController m_driverController{0};
#endif

frc::ADIS16470_IMU gyro;                  //MXP port gyro

   /*
    * The VisionThread() function demonstrates the detection of AprilTags.
    * The image is acquired from the USB camera, then any detected AprilTags
    * are marked up on the image and sent to the dashboard.
    */

  static void VisionThread() {
    frc::AprilTagDetector detector;
    // look for tag36h11, correct 3 error bits
    detector.AddFamily("tag36h11", 0);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    frc::AprilTagPoseEstimator::Config poseEstConfig = {
        .tagSize = units::length::inch_t(6.5),
        .fx = 699.3778103158814,
        .fy = 677.7161226393544,
        .cx = 345.6059345433618,
        .cy = 207.12741326228522};
    frc::AprilTagPoseEstimator estimator(poseEstConfig);

    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    // Set the resolution
    camera.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo("Detected", 640, 480);

    //no worky
    // cs::UsbCamera cameraBack = frc::CameraServer::StartAutomaticCapture();
    // cameraBack.SetResolution(640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;
    cv::Mat grayMat;

    // Instantiate once
    std::vector<int64_t> tags;
    // Outline colors in bgr, not rgb
    cv::Scalar outlineColor{0, 255, 0}; // green
    cv::Scalar outlineColorTop{0, 0, 255}; // red
    cv::Scalar outlineColorBottom{255, 0, 0}; // blue
    cv::Scalar crossColor{0, 0, 255};

    // We'll output to NT
    auto tagsTable =
        nt::NetworkTableInstance::GetDefault().GetTable("apriltags");
    auto pubTags = tagsTable->GetIntegerArrayTopic("tags").Publish();

    while (true) {
      // Tell the CvSink to grab a frame from the camera and
      // put it in the source mat.  If there is an error notify the
      // output.
      // grab robot yaw at frame grab
      units::angle::degree_t gyroYawHeadingLocal = gyroYawHeading;
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        continue;
      }

      cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

      cv::Size g_size = grayMat.size();
      frc::AprilTagDetector::Results detections =
          detector.Detect(g_size.width, g_size.height, grayMat.data);

      // have not seen any tags yet
      tags.clear();


      //desiredYaw = 0; // if this is enabled, robot will not remember tag location if it leaves the field of view
      moveToAprilTag = 0;

      for (const frc::AprilTagDetection* detection : detections) {
        // remember we saw this tag
        tags.push_back(detection->GetId());

        // draw lines around the tag
        for (int i = 0; i <= 3; i++) {
          int j = (i + 1) % 4;
          const frc::AprilTagDetection::Point pti = detection->GetCorner(i);
          const frc::AprilTagDetection::Point ptj = detection->GetCorner(j);
          // draws sides different colors
          switch (i) {
            case 0: {
              //bottom
              line(mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
              outlineColorBottom, 2);
              break;
            }
            case 2: {
              //top
              line(mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
              outlineColorTop, 2);
              break;
            }
            default: {
              line(mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
              outlineColor, 2);
            }
          }
        }

        // mark the center of the tag
        const frc::AprilTagDetection::Point c = detection->GetCenter();
        int ll = 10;
        line(mat, cv::Point(c.x - ll, c.y), cv::Point(c.x + ll, c.y),
             crossColor, 2);
        line(mat, cv::Point(c.x, c.y - ll), cv::Point(c.x, c.y + ll),
             crossColor, 2);

        // identify the tag
        int tagId = detection->GetId();
        putText(mat, std::to_string(tagId),
                cv::Point(c.x + ll, c.y), cv::FONT_HERSHEY_SIMPLEX, 1,
                crossColor, 3);

        // determine pose
        frc::Transform3d pose = estimator.Estimate(*detection);

        // put pose into NT
        frc::Rotation3d rotation = pose.Rotation();
        tagsTable->GetEntry(fmt::format("pose_{}", detection->GetId()))
            .SetDoubleArray(
                {{ pose.X().value(),
                   pose.Y().value(),
                   pose.Z().value(),
                   rotation.X().value(),
                   rotation.Y().value(),
                   rotation.Z().value() }});


         // look for speaker tag (11)
        double tagRotDist = (g_size.width/2)-c.x; //tag distance from center
        double tagRotDistDeg = tagRotDist / 10; // tag distance in degrees, roughly
        units::length::meter_t tagDist = pose.Z(); //robot distance from tag

        double desiredDist;
        if (7 == tagId) {
          desiredDist = 2.572; // distance we want to be from april tag
          
          units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
          //printf("april tag bearing: %f\n", tagBearing);
          desiredYaw = (double) tagBearing; // writes desired yaw to be tag location


          //tag distance (z axis)
          //printf("tag distance: %f\n", tagDist.value());
          double moveRate = 1.5*1.5;
          //if (moveToAprilTag < 0) moveRate = -moveRate;
          //double dEventualDist = (double) tagDist + (0.5 / 600.0) * moveRate; // accounts for overshooting  :( broken
          moveToAprilTag = (double) tagDist - desiredDist;
        }
      }
      

      // put list of tags onto NT
      pubTags.Set(tags);

      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
  }
  void MotorInitTalon( WPI_TalonSRX &m_motor )
  {
    m_motor.ConfigFactoryDefault( 10 );
    m_motor.SetInverted( false );
                                             /* Set the peak and nominal outputs */
    m_motor.ConfigNominalOutputForward( 0, 10 );
    m_motor.ConfigNominalOutputReverse( 0, 10 );
    m_motor.ConfigPeakOutputForward(    1, 10 );
    m_motor.ConfigPeakOutputReverse(   -1, 10 );

            /* Set limits to how much current will be sent through the motor */
    m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
#ifdef SAFETY_LIMITS
    m_motor.ConfigPeakCurrentLimit(10);       // limit motor power severely
    m_motor.ConfigContinuousCurrentLimit(10); // to 10 Amps
#else
    m_motor.ConfigPeakCurrentLimit(60);          // 60 works here for miniCIMs,
                                                 // or maybe 40 Amps is enough,
                                                 // but we reduce to 10, 1, 10
    m_motor.ConfigContinuousCurrentLimit(60);    // for safety while debugging
#endif
    m_motor.EnableCurrentLimit(true);

                                          // Config 100% motor output to 12.0V
    m_motor.ConfigVoltageCompSaturation( 12.0 );
    m_motor.EnableVoltageCompensation( false );

    m_motor.SetNeutralMode( NeutralMode::Brake );

  }      // MotorInitTalon()

  void MotorInitVictor( WPI_VictorSPX &m_motor )
  {
    m_motor.ConfigFactoryDefault( 10 );
    m_motor.SetInverted( false );
                                             /* Set the peak and nominal outputs */
    m_motor.ConfigNominalOutputForward( 0, 10 );
    m_motor.ConfigNominalOutputReverse( 0, 10 );
    m_motor.ConfigPeakOutputForward(    1, 10 );
    m_motor.ConfigPeakOutputReverse(   -1, 10 );

            /* Set limits to how much current will be sent through the motor */
    //m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
#ifdef SAFETY_LIMITS
    m_motor.ConfigPeakCurrentLimit(10);       // limit motor power severely
    m_motor.ConfigContinuousCurrentLimit(10); // to 10 Amps
#else
    //m_motor.ConfigPeakCurrentLimit(60);          // 60 works here for miniCIMs,
                                                 // or maybe 40 Amps is enough,
                                                 // but we reduce to 10, 1, 10
    //m_motor.ConfigContinuousCurrentLimit(60);    // for safety while debugging
#endif
    //m_motor.EnableCurrentLimit(true);

                                          // Config 100% motor output to 12.0V
    m_motor.ConfigVoltageCompSaturation( 12.0 );
    m_motor.EnableVoltageCompensation( false );

    m_motor.SetNeutralMode( NeutralMode::Brake );

  }      // MotorInitVictor()

 public:
  Robot() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_LeftDriveMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_RightDriveMotor);
  }

  void RobotInit() override {

    // We need to run our vision program in a separate thread.
    // If not run separately (in parallel), our robot program will never
    // get to execute.
    std::thread visionThread( VisionThread );
    visionThread.detach();

#ifndef SPARKMAXDRIVE
    MotorInitTalon( m_LeftDriveMotor       );
    MotorInitVictor( m_LeftDriveMotorRear  );
    MotorInitTalon( m_RightDriveMotor      );
    MotorInitVictor( m_RightDriveMotorRear );

    m_LeftDriveMotorRear.Follow(  m_LeftDriveMotor  );
    m_RightDriveMotorRear.Follow( m_RightDriveMotor );
#endif

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_RightDriveMotor.SetInverted(true);
#ifndef SPARKMAXDRIVE
    m_RightDriveMotorRear.SetInverted(true);  // is this necessary, or does it
					      // follow the change in direction
					      // of the master?
#endif
    gyro.Calibrate();

  }

  void TestPeriodic() override {
    int bmv1 = 0;
    if (m_driverController.GetXButton() ) {
      bmv1 = 6;
    } else if (m_driverController.GetYButton() ) {
      bmv1 = 12;
    } else {
      bmv1 = 0;
    }
    //Increases voltage depending on left trigger depth
    motor8.SetVoltage(units::volt_t{ bmv1 });
    motor11.SetVoltage(units::volt_t{ 12.0*m_driverController.GetRightTriggerAxis() });
    motor16.SetVoltage(units::volt_t{ 12.0*m_driverController.GetRightTriggerAxis() });
    motor10.SetVoltage(units::volt_t{ 12.0*m_driverController.GetLeftTriggerAxis() });

  }

  void TeleopPeriodic() override {
    double faceAprilTag;
    gyroYawHeading = gyro.GetAngle();
    gyroYawRate = gyro.GetRate();
    //int realGyroYawHeading = (int) gyroYawHeading % 180;

    double dEventualYaw = (double) gyroYawHeading + (0.5 / 600.0) * (double) gyroYawRate * std::abs((double) gyroYawRate); // accounts for overshooting

    //find the shortest degrees to face tag
    int degreesToTurn = (int) ((double) dEventualYaw - desiredYaw) % 360;
    if (degreesToTurn > 180) degreesToTurn -= 360;
    if (degreesToTurn < -180) degreesToTurn += 360;

    // converts degrees to turn to a value between -1 and 1 for use in arcade drive
    faceAprilTag = degreesToTurn * 1.0/10.0;
    faceAprilTag = std::max( -1.0, faceAprilTag );
    faceAprilTag = std::min( 1.0, faceAprilTag );
    

    moveToAprilTag = std::max( -1.0, moveToAprilTag );
    moveToAprilTag = std::min( 1.0, moveToAprilTag );
    



#ifdef JOYSTICK
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), -m_stick.GetX());
#else
    // Drive with split arcade style
    // That means that the Y axis of the left stick moves forward
    // and backward, and the X of the right stick turns left and right.
 

    //Handle Movement
    if (m_driverController.GetAButton()) {
      //point to tag
      m_robotDrive.ArcadeDrive(-m_driverController.GetLeftY(),
                              -faceAprilTag);
    } else if (m_driverController.GetBButton()) {
      //point and move to tag
      m_robotDrive.ArcadeDrive(moveToAprilTag,
                              -faceAprilTag);
    } else {
      m_robotDrive.ArcadeDrive(-m_driverController.GetLeftY(),
                              -m_driverController.GetRightX());
    }
#endif
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

