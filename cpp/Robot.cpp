// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

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
#include <vector>

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
#include <frc/ADIS16470_IMU.h>
#include "Drivetrain.h"
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <iostream>

//#define SAFETY_LIMITS 1;
//#define AUTO 1;


//April Tag Variables
static double desiredYaw;
static double desiredDist;

//Robot Pos Variables
static units::angle::degree_t gyroYawHeading; //robot yaw (degrees)
static units::angular_velocity::degrees_per_second_t gyroYawRate; //robot rotate rate (degrees/second)
static frc::Pose2d pose;

//Camera Variables
cs::UsbCamera camera1;
cs::UsbCamera camera2;
static cs::CvSink cvSink;

//Global Control Variables
static bool invertContols = false;
static cs::UsbCamera selectedCamera = camera1;

//Note Sensor
static bool noteInShooter = false;


class Robot : public frc::TimedRobot {

//Gyro
// frc::ADIS16470_IMU gyro;
//ctre::phoenix::sensors::WPI_PigeonIMU m_gyro{1};

//SHOOTER SPARKMAX
rev::CANSparkMax m_LeftShooterMotor{  13, rev::CANSparkMax::MotorType::kBrushless };
rev::CANSparkMax m_RightShooterMotor{ 10, rev::CANSparkMax::MotorType::kBrushless };
rev::SparkRelativeEncoder m_LeftShooterMotorEncoder{ m_LeftShooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor) };
rev::SparkRelativeEncoder m_RightShooterMotorEncoder{ m_RightShooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor) };

//INTAKE MC
WPI_TalonSRX m_IntakeMotor{3};

//PIVOT MC
//NO HARDSTOP
WPI_TalonSRX m_ShooterPivotMotor{36};

//CLIMBER MC
WPI_VictorSPX m_LeftClimberMotor{11};
WPI_VictorSPX m_RightClimberMotor{4};

//NOTE SENSOR
frc::DigitalInput shooterDIO{0};


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
    camera1 = frc::CameraServer::StartAutomaticCapture(0);
    //camera2 = frc::CameraServer::StartAutomaticCapture(1);

    // Set the resolution
    camera1.SetResolution(640, 480);
    //camera2.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    cvSink = frc::CameraServer::GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo("Detected", 640, 480);
    
    cvSink.SetSource(camera1);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;
    cv::Mat grayMat;
    cv::Mat hsvMat;

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
      desiredDist = 0.0;

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

        // How far we want to be from the tag
        double targetDist;

        //tag rotation
        units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
        desiredYaw = (double) tagBearing; // writes desired yaw to be tag location
        
        
        switch (tagId) {
          //SPEAKERS
          case 4:
          case 7: {
            //tag rotation
            units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
            desiredYaw = (double) tagBearing; // writes desired yaw to be tag location

            //tag distance (z axis)
            targetDist = 4.572; // distance we want to be from april tag
            desiredDist = (double) tagDist - targetDist;
            break;
          }

          //AMPS
          case 6:
          case 5: {
            //tag rotation
            units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
            desiredYaw = (double) tagBearing; // writes desired yaw to be tag location

            //tag distance (z axis)
            targetDist = 1.0; // distance we want to be from april tag
            desiredDist = (double) tagDist - targetDist;
            break;
          }

          //SOURCES
          case 10:
          case 1: {
            //tag rotation
            units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
            desiredYaw = (double) tagBearing; // writes desired yaw to be tag location

            //tag distance (z axis)
            targetDist = 1.0; // distance we want to be from april tag
            desiredDist = (double) tagDist - targetDist;
            break;
          }

          //STAGES
          case 11:
          case 12:
          case 13:
          case 14:
          case 15:
          case 16: {
            targetDist = 1.5; // distance we want to be from april tag
            units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
            desiredYaw = (double) tagBearing; // writes desired yaw to be tag location
            //tag distance (z axis)
            desiredDist = (double) tagDist - targetDist;
            break;
          }
          default: {

          }
        }
        
      }




      //put list of tags onto NT
      pubTags.Set(tags);

      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
  }

void MotorInitSpark(rev::CANSparkMax &m_motor)
   {
            // Set argument to true to also burn defaults into SparkMax flash.
      m_motor.RestoreFactoryDefaults(false);

      m_motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,
                              false);
      m_motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                              false);
      m_motor.SetInverted(false);           // set forward direction of motor.

           /* Set limits to how much current will be sent through the motor */
#ifdef SAFETY_LIMITS
                       //  2 Amps below 5000 RPM, above 5000 RPM it ramps from
                       //  2 Amps down to  1 Amp  at 5700 RPM
      m_motor.SetSmartCurrentLimit( 2, 1, 5000);
#else
                       // 30 Amps below 5000 RPM, above 5000 RPM it ramps from
                       // 30 Amps down to 10 Amps at 5700 RPM
      m_motor.SetSmartCurrentLimit(30, 10, 5000);
#endif
      m_motor.GetForwardLimitSwitch(
                                rev::SparkMaxLimitSwitch::Type::kNormallyOpen)
                                     .EnableLimitSwitch(false);

                                          // Config 100% motor output to 12.0V
      m_motor.EnableVoltageCompensation(12.0);

                 // Set ramp rate (how fast motor accelerates or decelerates).
                 // We may have to try different RampRates here to
                 // eliminate chattering.
      m_motor.SetClosedLoopRampRate(0.2);
      m_motor.SetOpenLoopRampRate(0.2);

      // m_motor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );
      m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

      // The following call saves all settings permanently in the SparkMax's
      // flash memory, so the settings survive even through a brownout.
      // These statements should be uncommented for at least one deploy-enable
      // Roborio cycle after any of the above settings change, but they should
      // be commented out between changes, to keep from using all the
      // SparkMax's flash-write cycles (because it has a limited number
      // of flash-write cycles).
      // m_motor.BurnFlash();

   } // MotorInitSpark()

void MotorInitTalon( WPI_TalonSRX &m_motor )
  {
    m_motor.ConfigFactoryDefault( 10 );
    m_motor.SetSensorPhase( true );  // invert encoder value positive/negative
    m_motor.SetInverted( false );

                                     /* Configure Sensor Source for Primary PID */
           /* Config to stop motor immediately when limit switch is closed. */
                                                    // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                     FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
//       m_motor.ConfigForwardLimitSwitchSource(
//                   LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
//                   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
//       m_motor.ConfigReverseLimitSwitchSource(
//                   LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
//                   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
          m_motor.OverrideLimitSwitchesEnable(true);
      }

         /*
          * Configure Talon SRX Output and Sensor direction.
          * Invert Motor to have green LEDs when driving Talon Forward
          * ( Requesting Positive Output ),
          * Phase sensor to have positive increment when driving Talon Forward
          * (Green LED)
          */
      m_motor.SetSensorPhase(true);   // invert encoder value positive/negative
      m_motor.SetInverted(false);     // invert direction of motor itself.
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
                                                 // or maybe 40 Amps is enough,
                                                 // but we reduce to 10, 1, 10
    m_motor.ConfigContinuousCurrentLimit(60);    // for safety while debugging
#endif
    m_motor.EnableCurrentLimit(true);

                                          // Config 100% motor output to 12.0V
    m_motor.ConfigVoltageCompSaturation( 12.0 );
    m_motor.EnableVoltageCompensation( false );

                     /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15,   10 );
         m_motor.Config_kP( 0, 0.2,    10 );
         m_motor.Config_kI( 0, 0.0002, 10 );
         m_motor.Config_kD( 0, 10.0,   10 );
      } else {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15, 10 );
         m_motor.Config_kP( 0, 0.0, 10 );
         m_motor.Config_kI( 0, 0.0, 10 );
         m_motor.Config_kD( 0, 0.0, 10 );
      }

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
  //m_motor.ConfigPeakCurrentLimit(10);       // limit motor power severely
  //m_motor.ConfigContinuousCurrentLimit(10); // to 10 Amps
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
 void RobotInit() override {

    // We need to run our vision program in a separate thread.
    // If not run separately (in parallel), our robot program will never
    // get to execute.
    std::thread visionThread( VisionThread );
    visionThread.detach();

    MotorInitSpark( m_LeftShooterMotor );
    MotorInitSpark( m_RightShooterMotor );
    MotorInitVictor( m_LeftClimberMotor);
    MotorInitVictor( m_RightClimberMotor);
    MotorInitTalon( m_IntakeMotor);
    MotorInitTalon( m_ShooterPivotMotor);
      m_ShooterPivotMotor.ConfigPeakCurrentLimit(12);       // limit motor power severely
      m_ShooterPivotMotor.ConfigContinuousCurrentLimit(12); // to 1 Amps

#ifndef SPARKMAXDRIVE
    /*
    MotorInitTalon( m_LeftDriveMotor       );
    MotorInitVictor( m_LeftDriveMotorRear  );
    MotorInitTalon( m_RightDriveMotor      );
    MotorInitVictor( m_RightDriveMotorRear );
    

    m_LeftDriveMotorRear.Follow(  m_LeftDriveMotor  );
    m_RightDriveMotorRear.Follow( m_RightDriveMotor );
    */
#endif

#ifndef SPARKMAXDRIVE
    /* m_RightDriveMotorRear.SetInverted(true);  // is this necessary, or does it
					      // follow the change in direction
					      // of the master? */
#endif
    // gyro.Calibrate();

  }

  void AutonomousInit() override {
    m_swerve.Reset();
  }

  void AutonomousPeriodic() override {
    static int iCallCount = 0;
    static int poseState = 0;
    iCallCount++;

    gyroYawHeading = m_swerve.GetYaw();
    gyroYawRate = m_swerve.GetRate();
    noteInShooter = !shooterDIO.Get();

    //DriveWithJoystick(false);

#ifndef AUTO    
    std::cout << DriveToPose({ 
                  (units::foot_t) 4.0,
                  (units::foot_t) 0.0,
                  (units::degree_t) 0.0
                }, false)
              << std::endl;
#endif

#ifdef AUTO
    switch (poseState) {
      case 0: {
        if (DriveToPose({ 
              (units::foot_t) 4.0,
              (units::foot_t) 0.0,
              (units::degree_t) 0.0
            }, false)) {
          poseState++;
        }
        break;
      }
      case 1: {
        if (DriveToPose({ 
              (units::foot_t) -4.0,
              (units::foot_t) 0.0,
              (units::degree_t) 0.0
            }, false)) {
          poseState++;
        }
        break;
      }
      case 2: {
        m_LeftShooterMotor.SetVoltage(units::volt_t{ -12.0 });
        m_RightShooterMotor.SetVoltage(units::volt_t{ 12.0 });
        break;
      }
    }
#endif
    
    //switch

    
    m_swerve.UpdateOdometry();
  }

  void TestInit() override {
    
  }

  void TestPeriodic() override {
    //OperatorControls();
    // Climbers (Right and Left Triggers)
    // - is default
    // if (A) is held, climbers spin in reverse
    int goReverse = 1;
    if (m_operatorController.GetYButton()) {
      goReverse = -1;
    }
    if (m_operatorController.GetRightTriggerAxis() > 0.3) {
      m_RightClimberMotor.SetVoltage(units::volt_t{ -6.0*m_operatorController.GetRightTriggerAxis()*goReverse });
    } else {
      m_RightClimberMotor.SetVoltage(units::volt_t{ 0 });
    }
    if (m_operatorController.GetLeftTriggerAxis() > 0.3) {
      m_LeftClimberMotor.SetVoltage( units::volt_t{ 6.0*m_operatorController.GetLeftTriggerAxis()*goReverse });
    } else {
      m_LeftClimberMotor.SetVoltage(units::volt_t{ 0 });
    }
  }


  void TeleopPeriodic() override {
    noteInShooter = !shooterDIO.Get();
 
    DriveWithJoystick(true);
    OperatorControls();

    m_swerve.UpdateOdometry();
  }

 private:
  frc::XboxController m_driverController{0};
  frc::XboxController m_operatorController{1};
  Drivetrain m_swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{   10 / 1_s};
  
   // Separate the slew rates for driving during autonomous
   frc::SlewRateLimiter<units::scalar> m_xspeedLimiterA{ 5 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_yspeedLimiterA{ 5 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_rotLimiterA{    5 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_driverController.GetLeftY(), 0.1)) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_driverController.GetLeftX(), 0.1)) *
                        Drivetrain::kMaxSpeed;


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_driverController.GetRightX(), 0.1)) *
                     Drivetrain::kMaxAngularSpeed;


    //The line below is not working
    //m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
    //m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod() > (units::time::second_t) 0);
    //m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
      
    //m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, m_driverController.GetBButton());



    gyroYawHeading = m_swerve.GetYaw();
    gyroYawRate = m_swerve.GetRate();

    double dEventualYaw = (double) gyroYawHeading + (0.5 / 600.0) * (double) gyroYawRate * std::abs((double) gyroYawRate); // accounts for overshooting

    //find the shortest degrees to face tag
    int degreesToTurn = (int) ((double) dEventualYaw - desiredYaw) % 360;
    if (degreesToTurn > 180) degreesToTurn -= 360;
    if (degreesToTurn < -180) degreesToTurn += 360;

    // Converts degrees from vision thread to radians per second.
    // Also converts 
    units::angular_velocity::radians_per_second_t faceAprilTag = (units::angular_velocity::radians_per_second_t) degreesToTurn * M_PI / 180;
    


    units::velocity::meters_per_second_t moveToAprilTag;
    moveToAprilTag = (units::velocity::meters_per_second_t) desiredDist;
    /**
     * Driver Controller
    */

  
  // <Swap Camera> / Invert Controls (X)
  if (m_driverController.GetXButton()) {
    // if (selectedCamera == camera1) {
    //   selectedCamera = camera2;
    // } else {
    //   selectedCamera = camera1;
    // }
    // cvSink.SetSource(selectedCamera);
    //invertContols = !invertContols; // swaps controls direction
    //fieldRelative = false;
  }

  // Relative Switch (A)
  //fieldRelative = !m_driverController.GetRightTriggerAxis();

  // Look To April Tag (Left Bumper)
  if (m_driverController.GetLeftBumper()) {
    rot = -faceAprilTag;
    // std::cout << (double) faceAprilTag
    //           << std::endl;
  }

  // Look / Go To April Tag (Right Bumper)
  if (m_driverController.GetRightBumper()) {
    //rot = -faceAprilTag;
    //fieldRelative = false;
    //ySpeed = (units::velocity::meters_per_second_t) desiredDist * 5; // 5 meters per second
  }

  // Reset Gyro and Position (Y)
  if (m_driverController.GetYButton()) {
    m_swerve.Reset();
  }

  //Note Tracking (???)
    
  //we'll get here

  // Brake (B) and drive
  m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, m_driverController.GetBButton());

  }

  void OperatorControls() {
    //FILE *fptr = NULL;
    static int iCallCount = 0;
    iCallCount++;
    // if (0 == iCallCount) {
    //   fptr = fopen("/tmp/log.txt", "a");
    // }

    // Intake (Right Bumper)
    if (m_operatorController.GetRightBumper()/* && !noteInShooter*/) {
      m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
      if (0 == iCallCount%2) {
        
        // std::cout << "current outputery: "
        //           << m_IntakeMotor.GetOutputCurrent()
        //           << ",  current stator: "
        //           << m_IntakeMotor.GetStatorCurrent()
        //           << std::endl;

        //fprintf(fptr, "amps: %f\n", m_IntakeMotor.GetOutputCurrent());
      }
    } 
    // Reverse Intake (X)
    else if (m_operatorController.GetXButton()) {
      m_IntakeMotor.SetVoltage(units::volt_t{ 9.0 });
      if (0 == iCallCount%2) {
        
        // std::cout << "current outputery: "
        //           << m_IntakeMotor.GetOutputCurrent()
        //           << ",  current stator: "
        //           << m_IntakeMotor.GetStatorCurrent()
        //           << std::endl;

        //fprintf(fptr, "amps: %f\n", m_IntakeMotor.GetOutputCurrent());
      }
    } else {
      m_IntakeMotor.SetVoltage(units::volt_t{0});

    };

    // Shooter (Left Bumper)
    if (m_operatorController.GetLeftBumper()) {
      m_LeftShooterMotor.SetVoltage(units::volt_t{ -12.0 });
      m_RightShooterMotor.SetVoltage(units::volt_t{ 12.0 });
      //once shooter reaches full power
      /*if (m_LeftShooterMotorEncoder.GetVelocity() > 4000.0) {
        m_IntakeMotor.SetVoltage(units::volt_t{ -6.0 });
      }*/
      std::cout << "Lshooter:"
                << m_LeftShooterMotorEncoder.GetVelocity()
                << "Rshooter:"
                << m_RightShooterMotorEncoder.GetVelocity()
                << std::endl;
      //fprintf(fptr, "rmp: %f\n", m_IntakeMotor.GetOutputCurrent());
    } else {
      m_LeftShooterMotor.SetVoltage( units::volt_t{ 0 });
      m_RightShooterMotor.SetVoltage(units::volt_t{ 0 });
    };

    // Climbers (Left and Right Triggers)
    if (m_operatorController.GetRightTriggerAxis() > 0.3) {
      m_RightClimberMotor.SetVoltage(units::volt_t{ -6.0*m_operatorController.GetRightTriggerAxis() });
    } else {
      m_RightClimberMotor.SetVoltage(units::volt_t{ 0 });
    }
    if (m_operatorController.GetLeftTriggerAxis() > 0.3) {
      m_LeftClimberMotor.SetVoltage( units::volt_t{ 6.0*m_operatorController.GetLeftTriggerAxis() }); //left climber goes other way
    } else {
      m_LeftClimberMotor.SetVoltage(units::volt_t{ 0 });
    }

    // Pivot up (Left Bumper)
    if (m_operatorController.GetLeftY() > 0.1) {
      m_ShooterPivotMotor.SetVoltage(units::volt_t{ -12.0*m_operatorController.GetLeftY() });
      //m_ShooterPivotMotor.Set
      std::cout << m_ShooterPivotMotor.GetSelectedSensorPosition()
                << "  Vel: "
                << m_ShooterPivotMotor.GetSelectedSensorVelocity()
                << "  Amplitude: "
                << m_ShooterPivotMotor.GetSensorCollection().GetPinStateQuadA()
                << std::endl;
    }
    // Pivot down (Right Bumper)
    else if (m_operatorController.GetLeftY() < -0.1) {
      m_ShooterPivotMotor.SetVoltage(units::volt_t{ -12.0*m_operatorController.GetLeftY() });
    } else {
      m_ShooterPivotMotor.SetVoltage(units::volt_t{ 0 });
    }

    // SUPER E BRAKE INCASE OF SUPER DUPER EMERGENCY
    // (BROKEN)
    if (m_operatorController.GetYButton()) {
      m_swerve.Drive((units::velocity::meters_per_second_t) 0, (units::velocity::meters_per_second_t) 0, (units::angular_velocity::radians_per_second_t) 0, true, true);
      //m_operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
    }
    
  }

  bool DriveToPose( frc::Pose2d DestinationPose, bool bFreezeDriveMotors ) {
      static bool bReturnValue = true;
      static int iCallCount = 0;

      static double dBiggestX = 0.0,  dBiggestY = 0.0;
      static double dSmallestX = 0.0, dSmallestY = 0.0;
      frc::Transform2d transform = DestinationPose - 
                               m_swerve.m_poseEstimator.GetEstimatedPosition();
      dBiggestX = std::max( transform.X().value(), dBiggestX );
      dBiggestY = std::max( transform.Y().value(), dBiggestY );
      dSmallestX = std::min( transform.X().value(), dSmallestX );
      dSmallestY = std::min( transform.Y().value(), dSmallestY );
      if ( !bFreezeDriveMotors && 0 == iCallCount%100 ) {
         std::cout << "DTP() X/X, Y/Y, Rot: "
              << dSmallestX << "/" << dBiggestX << ", "
              << dSmallestY << "/" << dBiggestY << ", "
              << transform.Rotation().Degrees().value()
              << std::endl;
      }
// jag; 07apr2023    (Limiter --> LimiterA in 3 places)
      auto xSpeed = m_xspeedLimiterA.Calculate(
                        transform.X().value() / 2.0 ) * Drivetrain::kMaxSpeed;
      auto ySpeed = m_yspeedLimiterA.Calculate(
                        transform.Y().value() / 2.0 ) * Drivetrain::kMaxSpeed;
      auto rot =
             m_rotLimiterA.Calculate( transform.Rotation().Degrees().value() /
                                       180.00 ) * Drivetrain::kMaxAngularSpeed;

      xSpeed = std::min( Drivetrain::kMaxSpeed, xSpeed );
      ySpeed = std::min( Drivetrain::kMaxSpeed, ySpeed );
      rot    = std::min( Drivetrain::kMaxAngularSpeed, rot );
      xSpeed = std::max( -Drivetrain::kMaxSpeed, xSpeed );
      ySpeed = std::max( -Drivetrain::kMaxSpeed, ySpeed );
      rot    = std::max( -Drivetrain::kMaxAngularSpeed, rot );

//    if ( 4 == iCallCount%50 ) {
//       std::cout << "xSpeed: " << xSpeed.value() << std::endl;
//    }

      m_swerve.Drive( xSpeed, ySpeed, rot, true, bFreezeDriveMotors );

                    // If any (X/Y/Rotation) of the criteria for being
                    // at the desired pose (within 20 cm X and Y, and
                    // within 10 degrees yaw) are still not met...
      if ( ( transform.X() < (units::length::meter_t)-0.20 ) ||
           ( (units::length::meter_t)0.20 < transform.X()  ) ||
           ( transform.Y() < (units::length::meter_t)-0.20 ) ||
           ( (units::length::meter_t)0.20 < transform.Y()  ) ||
           ( transform.Rotation().Degrees() < (units::degree_t)-10.0 ) ||
           ( (units::degree_t)10.0 < transform.Rotation().Degrees()  )    ) {
         bReturnValue = false;  // we are not yet at the desired pose
      } else {
         bReturnValue = true;   // we are at the desired pose
      }
      iCallCount++;
      return bReturnValue;
   }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  std::cout << "I am rat I am a theif I will steal your ethernet cable -The Rat";
  return frc::StartRobot<Robot>();
}
#endif