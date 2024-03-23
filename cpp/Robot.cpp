// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>

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
#include <frc/DriverStation.h>
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
#include <frc/Servo.h>

//#define SAFETY_LIMITS 1
#define AUTO 1


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
//static bool invertContols = false;
static cs::UsbCamera selectedCamera = camera1;
static int ra = 1; // Reversed auto, 1 for red, -1 for blue

//Note Sensor
static bool noteInShooter = false;
static bool prevNoteInShooter = false;

//AUTONOMOUS
static int poseState = 0;
static int poseStatePrev = 0;
int iCallCount = 0;
static frc::Pose2d startPose = { (units::foot_t) 0.0,
                                 (units::foot_t) 0.0,
                                 (units::degree_t) 0.0 };

// Auto route poseState position is autonomous switch case
// Ex. 10, start switch case at case 10, which is the start of a route
#define AUTO1_START 0
#define AUTO2_START 100
#define AUTO3_START 200

struct sState {
      bool   joyButton[12];
      bool   conButton[13];
   } sCurrState, sPrevState;

// Console button 12 is the leftmost missile switch.
#define BUTTON_SWITCH1 ( sCurrState.conButton[12] )
#define BUTTON_SWITCH2 ( sCurrState.conButton[9]  )
#define BUTTON_SWITCH3 ( sCurrState.conButton[10] )
#define BUTTON_SWITCH4 ( sCurrState.conButton[11] )

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
WPI_TalonSRX m_AmpArmMotor{36};

//CLIMBER MC
WPI_VictorSPX m_LeftClimberMotor{11};
WPI_VictorSPX m_RightClimberMotor{4};

//NOTE SENSOR
frc::DigitalInput shooterDIO{9};

//NOTE STOPPER
// #include <frc/PWM.h>
// #include <wpi/sendable/SendableHelper.h>
frc::Servo m_NoteStopper {2};

//FILE POINTER (for log file on roborio)
FILE *logfptr = NULL;

//JOYSTICKint
frc::Joystick m_Console{3};

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
    camera1 = frc::CameraServer::StartAutomaticCapture(0); //Note Camera
    camera2 = frc::CameraServer::StartAutomaticCapture(1); //April Tag Camera

    // Set the resolution
    camera1.SetResolution(320, 240); //160, 120
    camera2.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    cvSink = frc::CameraServer::GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo("Detected", 640, 480);
    
    cvSink.SetSource(camera2);

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
        //units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
        //desiredYaw = (double) tagBearing; // writes desired yaw to be tag location
        
        if (frc::DriverStation::kRed == frc::DriverStation::GetAlliance()) {
          switch (tagId) {
            case 4: {
              //printf("saw tag 4 (Red team)/n");
              // Tag distance (Z-Axis)
              targetDist = 2.642; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }

            //AMPS
            /*case 5: {
              // Tag distance (z axis)
              targetDist = 1.0; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }

            //SOURCES
            case 10: {
              // Tag distance (z axis)
              targetDist = 1.0; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }

            //STAGES
            case 11:
            case 12:
            case 13: {
              // Tag distance (z axis)
              targetDist = 1.5; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }
            default: {

            }*/
          }


        } else if (frc::DriverStation::kBlue == frc::DriverStation::GetAlliance()) {
          switch (tagId) {
            case 7: {
              //printf("saw tag 7 (Blue team)/n");
              // Tag distance (z axis)
              targetDist = 2.642; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }

            //AMPS
            /*case 6: {
              // Tag distance (z axis)
              targetDist = 1.0; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }

            //SOURCES
            case 10: {
              // Tag distance (z axis)
              targetDist = 1.0; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }

            //STAGES
            case 14:
            case 15:
            case 16: {
              // Tag distance (z axis)
              targetDist = 1.0; // Distance we want to be from april tag
              desiredDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }
            default: {

            }*/
          }
        }

      }




      //put list of tags onto NT
      pubTags.Set(tags);

      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
  }

    /*---------------------------------------------------------------------*/
      /* GetAllVariables()                                                   */
      /* Retrieves all variable values from sensors, encoders,               */
      /* the limelight, etc.  It should be called at the beginning of        */
      /* every 20-millisecond tick.  Doing it this way, rather than          */
      /* having each function retrieve the values it needs when it needs     */
      /* them, should minimize CANbus traffic and keep the robot CPU fast.   */
      /*---------------------------------------------------------------------*/
void GetAllVariables()  {
    sPrevState = sCurrState;

    for ( int iLoopCount=1; iLoopCount<=12; iLoopCount++ ) {
            sCurrState.conButton[iLoopCount] = m_Console.GetRawButton(iLoopCount);
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
      m_motor.SetClosedLoopRampRate(0.2); //0.2
      m_motor.SetOpenLoopRampRate(0.2); //0.2

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
    MotorInitTalon( m_AmpArmMotor);
      m_AmpArmMotor.ConfigPeakCurrentLimit(12);       // limit motor power severely
      m_AmpArmMotor.ConfigContinuousCurrentLimit(12); // to 1 Amps

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
    GetAllVariables();
    // Reversed auto, 1 for red, -1 for blue
    ra = (frc::DriverStation::kRed == frc::DriverStation::GetAlliance()) ? 1 : -1; // :)
    
    logfptr = fopen("/home/lvuser/RoborioLogAUTOnomous2024.txt", "a");


    // Reset
    m_LeftShooterMotor.SetVoltage(units::volt_t{ 0.0 });
    m_RightShooterMotor.SetVoltage(units::volt_t{ 0.0 });
    //m_swerve.Reset();
    iCallCount = 0;
    m_NoteStopper.SetAngle(0);

    // Check which autonomous to use
    // If multiple switches are flipped, priority goes to smallest number
    if (BUTTON_SWITCH1) {
      poseState = AUTO1_START;
      m_swerve.ResetPose({ (units::foot_t) 0.0,
                           (units::foot_t) 0.0,
                           (units::degree_t) 60.0 }); //60.0


      printf("Auto route 1 started on ");
      if (1 == ra) printf("red team.\n");
      else printf("blue team.");
      
    }
    else if (BUTTON_SWITCH2) {
      poseState = AUTO2_START;
      m_swerve.ResetPose({ (units::foot_t) 0.0,
                           (units::foot_t) 0.0,
                           (units::degree_t) 60.0 });
      

      printf("Auto route 2 started on ");
      if (1 == ra) printf("red team.");
      else printf("blue team.\n");
    }
    else if (BUTTON_SWITCH3) {
      poseState = AUTO3_START;
      m_swerve.ResetPose({ (units::foot_t) 0.0,
                           (units::foot_t) 0.0,
                           (units::degree_t) 0.0 });


      printf("Auto route 3 started on ");
      if (1 == ra) printf("red team.");
      else printf("blue team.\n");
    }
    else {
      poseState = AUTO1_START;
      m_swerve.ResetPose({ (units::foot_t) 0.0,
                           (units::foot_t) 0.0,
                           (units::degree_t) 60.0 }); //60.0
      printf("No auto route selected, defaulting to route #1\n");
    }

    printf("AUTONOMOUS INITIALIZED");

    ExecutiveImportGradleSourceDirective(); //splish splash sploosh
  }

  void AutonomousPeriodic() override {
    static int iCallCountprev = 0;
    iCallCount++;
    
    // std::cout << DriveToPose()
    //           << std::endl;

    gyroYawHeading = m_swerve.GetYaw();
    gyroYawRate = m_swerve.GetRate();
    noteInShooter = !shooterDIO.Get();
    //auto faceAprilTag = GetATagVariables();

    //DriveWithJoystick(false);
    if (poseStatePrev != poseState) {
      std::cout << "State = " << poseState
                << std::endl;
    }
    poseStatePrev = poseState;

    /**
     * AUTONOMOUS STAGES
    */
    switch (poseState) {

      // ROUTE #1
      //Go to shooter position / shoot [
      case 0: {
        if (DriveToPose({ 
                   (units::foot_t) 0.0,
              ra * (units::foot_t) 0.0,
              ra * (units::degree_t) 60.0 //60.0
            }, false) && 
            iCallCount > 20) { poseState++; }

      }
      break;

      case 1: {
        if (DriveToPose({ 
                   (units::foot_t) -5.5,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 60.0 // -30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      }
      break;

      case 2: {
        if (DriveToPose({ 
                   (units::foot_t) -5.5,
              ra * (units::foot_t) -1.0,
              (units::degree_t) ((desiredYaw < 50 && desiredYaw > -50) ? desiredYaw : ra * 30) // 30.0 // -30.0 //30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      } 
      break;

      case 3: {
        shooterSetVoltage(units::volt_t{ 11.0 });
        drivebrake();

        if (m_LeftShooterMotorEncoder.GetVelocity() <= -3100 && (iCallCountprev + 100 <= iCallCount)) {
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); 
          iCallCountprev = iCallCount;
          poseState++;
        }
      }
      break;

      case 4: {
        drivebrake();
        if (iCallCountprev + 25 <= iCallCount) { //wait here with the motor spinning for 200 milliseconds
          shooterSetVoltage(units::volt_t{ 0.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
          poseState++; 
        }
      }
      break;
      //Go to shooter position / shoot ]

      //Go to Stage Note / intake [
      case 5: {
        if (DriveToPose({ 
                   (units::foot_t) -8.0, //-3.0 //-5.0 //-7.0
              ra * (units::foot_t) -1.83, //5.0 //-1.0 //-1.42
              ra * (units::degree_t) 0.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      }
      break;

      case 6: {
        drivebrake();
        if (iCallCountprev + 300 <= iCallCount || noteInShooter) {
          m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
          poseState++;
        } 
      }
      break;
      //Go to Stage Note / intake ]

      //Go to shooter position / shoot [
      case 7: {
        if (DriveToPose({ 
                    (units::foot_t) -5.5, //-3.6
                ra * (units::foot_t) -1.0, //-3.6 //3.2
                ra * (units::degree_t) 0.0 //30 //-25.0
            }, false)) {m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 }); poseState++; iCallCountprev = iCallCount; }
      }
      break;

      case 8: {
        if (DriveToPose({ 
                   (units::foot_t) -5.5, //-5.0 //ig -4.8
              ra * (units::foot_t) -1.0, //-1.0 //ig -0.5
              ra * (units::degree_t) 30.0
            }, false) /*&& 
            iCallCountprev + 50 < iCallCount*/) { poseState++; }
      }
      break;
      
      case 9: {
        shooterSetVoltage(units::volt_t{ 11.0 });
        drivebrake();

        if (m_LeftShooterMotorEncoder.GetVelocity() <= -3100 && (iCallCountprev + 100 <= iCallCount)) {
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); 
          iCallCountprev = iCallCount;
          poseState++;
        }
      }
      break;

      case 10 : {
        drivebrake();
        if (iCallCountprev + 25 <= iCallCount) { //wait here with the motor spinning for 200 milliseconds
          shooterSetVoltage(units::volt_t{ 0.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
          poseState++; 
        }
      }
      break;
      //Go to shooter position / shoot ]

      //Go to NoteA behind the stage / intake [
      //5a
      case 11: {
        if (DriveToPose({ 
                   (units::foot_t) -5.5,
              ra * (units::foot_t) -7.42, //-5.42
              ra * (units::degree_t) 0.0
            }, false)) { m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); poseState++; }
      }
      break; 

      case 12: {
        if (DriveToPose({ 
                   (units::foot_t) -10.75,
              ra * (units::foot_t) -5.42,
              ra * (units::degree_t) 0.0
            }, false)) { m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); poseState++; }
      }
      break; 

      //5b
      case 13: {
        if (DriveToPose({ 
                   (units::foot_t) -15.33,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 0.0
            }, false)) { m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); iCallCount = iCallCountprev; poseState++; }
      }
      break; 

      //5c
      case 14: {
        if (DriveToPose({ 
                   (units::foot_t) -25.41,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 0.0
            }, false)) {iCallCount = iCallCountprev; poseState++; }
      }
      break;  

      case 15: {
        drivebrake();
        if (iCallCountprev + 300 <= iCallCount || noteInShooter) {
          m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
          poseState++;
        } 
      }
      break;
      //Go to NoteA behind the stage / intake ]

      //6
      //Go to shooter position / shoot [
      case 16: {
        if (DriveToPose({ 
                   (units::foot_t) -13.0, //-10.0
              ra * (units::foot_t) -1.0, //4.0
              ra * (units::degree_t) 0.0
            }, false)) { m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 }); poseState++; }
      }
      break;  

      case 17: {
        if (DriveToPose({ 
                   (units::foot_t) -13.0, //-10.0
              ra * (units::foot_t) 4.0,
              ra * (units::degree_t) 0.0
            }, false)) { poseState++; }
      }
      break;

      case 18: {
       if (DriveToPose({ 
                   (units::foot_t) -5.25,
              ra * (units::foot_t) 4.0,
              ra * (units::degree_t) 5.0
            }, false)) { iCallCountprev = iCallCount; poseState=21; }
      }
      break;

      case 19: {
       if (DriveToPose({ 
                   (units::foot_t) -5.5,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 0.0
            }, false)) { poseState++; }
      }
      break;

      case 20: {
       if (DriveToPose({ 
                   (units::foot_t) -5.5,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 30.0
            }, false)) { iCallCountprev = iCallCount; poseState++; }
      }
      break;

      case 21: {
        shooterSetVoltage(units::volt_t{ 11.0 });
        drivebrake();

        if (m_LeftShooterMotorEncoder.GetVelocity() <= -3100 && (iCallCountprev + 100 <= iCallCount)) {
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); 
          iCallCountprev = iCallCount;
          poseState++;
        }
      } 
      break;

      case 22: {
        drivebrake();
        if (iCallCountprev + 25 <= iCallCount) { //wait here with the motor spinning for 200 milliseconds
          shooterSetVoltage(units::volt_t{ 0.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
          poseState=-1; 
        }
      }
      break;
      //Go to shooter position / shoot ]


      // ROUTE #1 END



      // ROUTE #2
      // Go to shooter position / shoot [ 
      case 100: {
        if (DriveToPose({ 
                   (units::foot_t) 0.0,
              ra * (units::foot_t) 0.0,
              ra * (units::degree_t) 60.0 //60.0
            }, false) && 
            iCallCount > 20) { poseState++; }

      }
      break;

      case 101: {
        if (DriveToPose({ 
                   (units::foot_t) -6.0,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 60.0 // -30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      }
      break;

      case 102: {
        if (DriveToPose({ 
                   (units::foot_t) -6.0,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 30.0 // -30.0 //30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      } 
      break;

      case 103: {
        shooterSetVoltage(units::volt_t{ 11.0 });
        drivebrake();

        if (m_LeftShooterMotorEncoder.GetVelocity() <= -3100 && (iCallCountprev + 100 <= iCallCount)) {
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); 
          iCallCountprev = iCallCount;
          poseState++;
        }
      }
      break;

      case 104: {
        drivebrake();
        if (iCallCountprev + 25 <= iCallCount) { //wait here with the motor spinning for 200 milliseconds
          shooterSetVoltage(units::volt_t{ 0.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
          poseState++; 
        }
      }
      break;
      // Go to shooter position / shoot ]
      
      // Go to Note-B / Intake [
      case 105: {
        if (DriveToPose({ 
                   (units::foot_t) -6.75,
              ra * (units::foot_t) -7.75, //-5.75
              ra * (units::degree_t) 30.0 // -30.0 //30.0
            }, false)) { poseState=107; iCallCountprev = iCallCount; }
      } 
      break;

      case 106: {
        if (DriveToPose({ 
                   (units::foot_t) -19.75,
              ra * (units::foot_t) -14.0, //-10.08
              ra * (units::degree_t) 0.0 // -30.0 //30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      } 
      break;

      case 107: {
        if (DriveToPose({ 
                   (units::foot_t) -25.25,
              ra * (units::foot_t) -14.0, //10.08
              ra * (units::degree_t) 0.0 // -30.0 //30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      } 
      break;

      case 108: {
       if (iCallCountprev + 300 <= iCallCount || noteInShooter) {
          m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
          poseState++;
        } 
      }
      break;
      // Go to Note-B / Intake ]
      
      // Go to shooter position / shoot [ 
      case 109: {
        if (DriveToPose({ 
                   (units::foot_t) -6.75,
              ra * (units::foot_t) -7.75, //-5.75
              ra * (units::degree_t) 0.0 // -30.0 //30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      } 
      break;

      case 110: {
        if (DriveToPose({ 
                   (units::foot_t) -6.0,
              ra * (units::foot_t) -1.0, //WE SHOULD MAKE THIS 0.0!!
              ra * (units::degree_t) 30.0 // -30.0 //30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      } 
      break;

      case 111: {
        shooterSetVoltage(units::volt_t{ 11.0 });
        drivebrake();

        if (m_LeftShooterMotorEncoder.GetVelocity() <= -3100 && (iCallCountprev + 100 <= iCallCount)) {
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); 
          iCallCountprev = iCallCount;
          poseState++;
        }
      }
      break;

      case 112: {
        drivebrake();
        if (iCallCountprev + 25 <= iCallCount) { //wait here with the motor spinning for 200 milliseconds
          shooterSetVoltage(units::volt_t{ 0.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
          poseState=-1; 
        }
      }
      break;
      // Go to shooter position / shoot ]

      // ROUTE #2 END



      // ROUTE #3
      // ROUTE #2
      // Go to shooter position / shoot [ 
      case 200: {
        if (DriveToPose({ 
                   (units::foot_t) 0.0,
              ra * (units::foot_t) 0.0,
              ra * (units::degree_t) 60.0 //60.0
            }, false) && 
            iCallCount > 20) { poseState++; }

      }
      break;

      case 201: {
        if (DriveToPose({ 
                   (units::foot_t) -6.0,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 60.0 // -30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      }
      break;

      case 202: {
        if (DriveToPose({ 
                   (units::foot_t) -6.0,
              ra * (units::foot_t) -1.0,
              ra * (units::degree_t) 30.0 // -30.0 //30.0
            }, false)) { poseState++; iCallCountprev = iCallCount; }
      } 
      break;

      case 203: {
        shooterSetVoltage(units::volt_t{ 11.0 });
        drivebrake();

        if (m_LeftShooterMotorEncoder.GetVelocity() <= -3100 && (iCallCountprev + 100 <= iCallCount)) {
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 }); 
          iCallCountprev = iCallCount;
          poseState++;
        }
      }
      break;

      case 204: {
        drivebrake();
        if (iCallCountprev + 25 <= iCallCount) { //wait here with the motor spinning for 200 milliseconds
          shooterSetVoltage(units::volt_t{ 0.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
          poseState++; 
        }
      }
      break;
      // Go to shooter position / shoot ]

      // RETURN
      case 1000: {
        if (iCallCountprev + 300 <= iCallCount) {
          if (DriveToPose(startPose, false)) {
            poseState = -1;
          }
        }
      }
      break;


      default: {
        m_swerve.Drive(units::velocity::meters_per_second_t{ 0.0 }, 
                       units::velocity::meters_per_second_t{ 0.0 },
                       units::angular_velocity::radians_per_second_t{ 0.0 }, 
                       false, true);
        m_LeftShooterMotor.SetVoltage(units::volt_t{ 0.0 });
        m_RightShooterMotor.SetVoltage(units::volt_t{ 0.0 });
        m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
      }


    }
    
    //switch

    
    m_swerve.UpdateOdometry();
  }
  
  void AutonomousExit() override {
    fclose(logfptr);
  }


  void TestInit() override {
    m_NoteStopper.SetAngle(0);
    iCallCount = 0;
    //superImportantRobotEXE();
    ExecutiveImportGradleSourceDirective();
  }

  void TestPeriodic() override {
    static bool amping;
    static int ampingStage;
    static int iCallCountprev;
    iCallCount++;

    GetAllVariables();

    //OperatorControls();
    // Climbers (Right and Left Triggers)
    // - is default
    // if (Y) is held, climbers spin in reverse
    int goReverse = m_operatorController.GetXButton() ? -1 : 1;
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

    // TEST BUTTONS
    if (m_driverController.GetBackButton()) {
    std::cout << "HELLO WORLD"
              << std::endl;
    }

    if (m_operatorController.GetBButton()) {
      m_NoteStopper.SetAngle(0.0);
      m_AmpArmMotor.SetVoltage(units::volt_t{ 3.0 });
    } else if (m_operatorController.GetAButton()) {
      m_NoteStopper.SetAngle(0.0);
      m_AmpArmMotor.SetVoltage(units::volt_t{ -3.0 });
    } else {
      m_AmpArmMotor.SetVoltage(units::volt_t{ 0.0 });
    }

    if (m_operatorController.GetYButtonPressed()) {
      if (!amping) {
        amping = true;
        ampingStage = 0;
        iCallCountprev = iCallCount;
      }
    }

    if (amping) {
      switch (ampingStage) {
        case 0: {
          shooterSetVoltage(units::volt_t{ 3.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -6.0 });
          if (iCallCountprev + 12 <= iCallCount) {
            ampingStage++;
          }
          break;
        }
        case 1: {
          //m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
          m_NoteStopper.SetAngle(135);
          iCallCountprev = iCallCount;
          ampingStage++;
          break;
        }
        case 2: {
          if (iCallCountprev + 24 <= iCallCount) {
            ampingStage = -1;
          }
          break;
        }
        case 3: {
          
          break;
        }

        default: {
          m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
          shooterSetVoltage(units::volt_t{ 0.0 });
          ampingStage = 0;
          amping = false;
        }
      }
    }



// if (m_Console.) {
    //   std::cout << "AGHAgAGAG"
    //             << std::endl;
    // }

    // PILOT SWITCH 1 (L)
    if (BUTTON_SWITCH1) {
      if (frc::DriverStation::kRed == frc::DriverStation::GetAlliance()) {
        std::cout << "BATTERY VOLTAGE RED: "
                  << frc::DriverStation::GetBatteryVoltage()
                  << std::endl;
      }
      else if (frc::DriverStation::kBlue == frc::DriverStation::GetAlliance()) {
        std::cout << "BATTERY VOLTAGE BLUE: "
                  << frc::DriverStation::GetBatteryVoltage()
                  << std::endl;
      }
      else {
        std::cout << "Error: Cannot start OmniSharp because Mono version >=3.10.0 is required"
                  << std::endl;
      }
    }

    // PILOT SWITCH 2
    if (BUTTON_SWITCH2) {
      if (frc::DriverStation::kRed == frc::DriverStation::GetAlliance()) {
        std::cout << "RED 1 INITIATED"
                  << std::endl;
      }
      else if (frc::DriverStation::kBlue == frc::DriverStation::GetAlliance()) {
        std::cout << "BLUE 1 INITIATED"
                  << std::endl;
      }
      else {
        std::cout << "NO ALLIANCE SELECTED"
                  << std::endl;
      }
    }

    // PILOT SWITCH 3
    if (BUTTON_SWITCH3) {
      if (frc::DriverStation::kRed == frc::DriverStation::GetAlliance()) {
        std::cout << "RED 2 INITIATED"
                  << std::endl;
      }
      else if (frc::DriverStation::kBlue == frc::DriverStation::GetAlliance()) {
        std::cout << "BLUE 2 INITIATED"
                  << std::endl;
      }
      else {
        std::cout << "NO ALLIANCE SELECTED"
                  << std::endl;
      }   
    }

    // PILOT SWITCH 4 (R)
    if (BUTTON_SWITCH4) {
      if (frc::DriverStation::kRed == frc::DriverStation::GetAlliance()) {
        std::cout << "RED 3 INTIATED"
                  << std::endl;
        m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.02);
      }
      else if (frc::DriverStation::kBlue == frc::DriverStation::GetAlliance()) {
        std::cout << "BLUE 3 INITIATED"
                  << std::endl;
        m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
      }
      else {
        std::cout << "NO ALLIANCE SELECTED"
                  << std::endl;
      }
    }
  }


  void TeleopInit() override {
    m_NoteStopper.SetAngle(0);
    logfptr = fopen("/home/lvuser/RoborioLogTeleop2024.txt", "a");
    ExecutiveImportGradleSourceDirective();
  }

  void TeleopPeriodic() override {
    prevNoteInShooter = noteInShooter;
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
    // SLOOOOOOWMODE
    // low speed by 0.3 if holding r-trigger //0.2
    double lowGear = m_driverController.GetRightTriggerAxis() > 0.1 ? 0.3 : 1.0;

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_driverController.GetLeftY(), 0.2)) *
                        Drivetrain::kMaxSpeed * lowGear;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_driverController.GetLeftX(), 0.2)) *
                        Drivetrain::kMaxSpeed * lowGear;


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_driverController.GetRightX(), 0.2)) *
                     Drivetrain::kMaxAngularSpeed * lowGear;



    auto faceAprilTag = GetATagVariables();

    

    /**
     * Driver Controller
    */
  
  //AVAILABLE BUTTONS
  //LEFT TRIGGER
  //RIGHT TRIGGER
  //POV
  //START BUTTON (HAMBURGER)
  //BACK BUTTON
  //LEFT SITCK PUSH
  //RIGHT STICK PUSH
  
  // if (m_driverController.GetStartButton()) {
  //   m_swerve.Drive()
  // }

  // <Swap Camera> (X)
  if (m_driverController.GetXButton()) {
    if (selectedCamera == camera1) {
      selectedCamera = camera2;
    } else {
      selectedCamera = camera1;
    }
    cvSink.SetSource(selectedCamera);
  }

  // Relative Switch (A)
  fieldRelative = !m_driverController.GetAButton();

  // Look To April Tag (Left Bumper)
  if (m_driverController.GetLeftBumper()) {
    rot = -faceAprilTag*10;
    fieldRelative = true;
    // std::cout << (double) faceAprilTag
    //           << std::endl;
  }

  // Look / Go To April Tag (Right Bumper)
  if (m_driverController.GetRightBumper()) {
    rot = -faceAprilTag*10;
    fieldRelative = false;
    xSpeed = (units::velocity::meters_per_second_t) desiredDist * 2.0;
  }

  // Reset Gyro and Position (Y)
  if (m_driverController.GetYButton()) {
    m_swerve.Reset();
  }
  




  //Note Tracking (???)
    
  //we'll get here

  // Brake (B) and drive
  m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, m_driverController.GetBButton());

  // fprintf(logfptr, "SH RPM: %5.0f\n", m_swerve.);
  }

  void OperatorControls() {
    static bool stopIntake = false;
    static int iCallCount = 0;
    iCallCount++;
    static bool amping;
    static int ampingStage;
    static int iCallCountprev;
  
        // std::cout << "Lshooter: "
        //           << m_LeftShooterMotorEncoder.GetVelocity()
        //           << ",   Rshooter: "
        //           << m_RightShooterMotorEncoder.GetVelocity()
        //           << std::endl
        //           << "Left motor pose: "
        //           << m_LeftShooterMotorEncoder.GetPosition()
        //           << ",   Right motor pose"
        //           << m_RightShooterMotorEncoder.GetPosition()
        //           << std::endl;

    // std::cout << "shooter sensor: "
    //           << shooterDIO.Get()
    //           << std::endl;


    if (noteInShooter) {
      stopIntake = true;
      //printf("AAAGHH STOP THE NOTE Please\n");
    }
    //Reset stopIntake
    if (!m_operatorController.GetRightBumper()) {
      stopIntake = false;
    }

    // Intake (Right Bumper)
    if (m_operatorController.GetRightBumper() && (!stopIntake || 
                                                 (m_LeftShooterMotorEncoder.GetVelocity() < -3100.0 && 
                                                  m_operatorController.GetLeftBumper()) ||
                                                  m_operatorController.GetStartButton() // Override sensor
                                                 )) {
      m_IntakeMotor.SetVoltage(units::volt_t{ -12.0 });
      // std::cout << "IntRun "
      //           << std::endl;
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

        if (NULL != logfptr) {
          fprintf(logfptr, "amps: %f\n", m_IntakeMotor.GetOutputCurrent());
        }
      }
    } else if (!amping) {
      m_IntakeMotor.SetVoltage(units::volt_t{0});
    };

    // Shooter (Left Bumper)
    if (m_operatorController.GetLeftBumper()) {
      m_LeftShooterMotor.SetVoltage(units::volt_t{ -11.0 }); // normal value -11.0
      m_RightShooterMotor.SetVoltage(units::volt_t{ 11.0 }); // normal value 11.0
      // std::cout << "LshootRpm:"
      //           << m_LeftShooterMotorEncoder.GetVelocity()
      //           << " RshootRpm:"
      //           << m_RightShooterMotorEncoder.GetVelocity()
      //           << std::endl;
    }  
    else if (!amping) {
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


    //EPIC AMPING!!
    if (m_operatorController.GetLeftBumperPressed()) {
      iCallCountprev = iCallCount;
    }

    if (m_operatorController.GetBButton()) {
      m_NoteStopper.SetAngle(0.0);
      m_AmpArmMotor.SetVoltage(units::volt_t{ 6.0 });
    } else if (m_operatorController.GetAButton()) {
      m_NoteStopper.SetAngle(0.0);
      m_AmpArmMotor.SetVoltage(units::volt_t{ -6.0 });
    } 
    //Retract intermitantly when shooting
    else if (m_operatorController.GetLeftBumper()) {
      if (iCallCountprev + 50 <= iCallCount) {
        m_AmpArmMotor.SetVoltage(units::volt_t{ -2.0 });
      }
      // retract for 5
      if (iCallCountprev + 53 <= iCallCount) {
        m_AmpArmMotor.SetVoltage(units::volt_t{ 0.0 });
        iCallCountprev = iCallCount;
      }
    }
    //Else turn off
    else {
      m_AmpArmMotor.SetVoltage(units::volt_t{ 0.0 });
    }

    if (m_operatorController.GetYButtonPressed()) {
      if (!amping) {
        amping = true;
        ampingStage = 0;
        iCallCountprev = iCallCount;
      }
    }

    if (amping) {
      switch (ampingStage) {
        case 0: {
          shooterSetVoltage(units::volt_t{ 3.0 });
          m_IntakeMotor.SetVoltage(units::volt_t{ -6.0 });
          if (iCallCountprev + 6 <= iCallCount) {
            ampingStage++;
          }
          break;
        }
        case 1: {
          //m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
          m_NoteStopper.SetAngle(135);
          iCallCountprev = iCallCount;
          ampingStage++;
          break;
        }
        case 2: {
          if (iCallCountprev + 24 <= iCallCount) {
            ampingStage = -1;
          }
          break;
        }
        case 3: {
          
          break;
        }

        default: {
          m_IntakeMotor.SetVoltage(units::volt_t{ 0.0 });
          shooterSetVoltage(units::volt_t{ 0.0 });
          ampingStage = 0;
          amping = false;
        }
      }
    }

    
  }

  void TeleopExit() override {
    fclose(logfptr);
  }

  bool DriveToPose( frc::Pose2d DestinationPose, bool bFreezeDriveMotors ) {
      static bool bReturnValue = true;
      static int iCallCount = 0;

      static double dBiggestX = 0.0,  dBiggestY = 0.0;
      static double dSmallestX = 0.0, dSmallestY = 0.0;
      // Problem code below
      // frc::Transform2d transform = DestinationPose - 
      //                          m_swerve.m_poseEstimator.GetEstimatedPosition();
      frc::Transform2d transform = {
        DestinationPose.X() - m_swerve.m_poseEstimator.GetEstimatedPosition().X(),
        DestinationPose.Y() - m_swerve.m_poseEstimator.GetEstimatedPosition().Y(),
        DestinationPose.Rotation() - m_swerve.m_poseEstimator.GetEstimatedPosition().Rotation(),
      };


              //find the shortest degrees to face tag
      int AutodegreesToTurn = (int) ((double) transform.Rotation().Degrees()) % 360;
      if (AutodegreesToTurn > 180) AutodegreesToTurn -= 360;
      if (AutodegreesToTurn < -180) AutodegreesToTurn += 360;

      auto rot =
        m_rotLimiterA.Calculate( AutodegreesToTurn /
                                  180.00 ) * Drivetrain::kMaxAngularSpeed;
      // jag; 07apr2023    (Limiter --> LimiterA in 3 places)
      auto xSpeed = m_xspeedLimiterA.Calculate(
                        transform.X().value() / 2.0 ) * Drivetrain::kMaxSpeed;
      auto ySpeed = m_yspeedLimiterA.Calculate(
                        transform.Y().value() / 2.0 ) * Drivetrain::kMaxSpeed;

      if (AutodegreesToTurn <= -20 || 20 <= AutodegreesToTurn) {
        xSpeed = (units::velocity::meters_per_second_t) 0.0;
        ySpeed = (units::velocity::meters_per_second_t) 0.0;

        rot    = std::min( Drivetrain::kMaxAngularSpeed, rot );
        rot    = std::max( -Drivetrain::kMaxAngularSpeed, rot );

      } else if (AutodegreesToTurn <= -1 || 1 <= AutodegreesToTurn) {
        rot    = std::min( Drivetrain::kMaxAngularSpeed, rot );
        rot    = std::max( -Drivetrain::kMaxAngularSpeed, rot );

      } else {
        rot = (units::angular_velocity::radians_per_second_t) 0.0;
      }

      dBiggestX = std::max( transform.X().value(), dBiggestX );
      dBiggestY = std::max( transform.Y().value(), dBiggestY );
      dSmallestX = std::min( transform.X().value(), dSmallestX );
      dSmallestY = std::min( transform.Y().value(), dSmallestY );

      xSpeed = std::min( Drivetrain::kMaxSpeed, xSpeed );
      ySpeed = std::min( Drivetrain::kMaxSpeed, ySpeed );
      xSpeed = std::max( -Drivetrain::kMaxSpeed, xSpeed );
      ySpeed = std::max( -Drivetrain::kMaxSpeed, ySpeed );
      

      if ( !bFreezeDriveMotors && 0 == iCallCount%10 ) {
        //  std::cout << "DTP() X/X, Y/Y, Rot: "
        //       // << dSmallestX << "/" << dBiggestX << ", "
        //       // << dSmallestY << "/" << dBiggestY << ", "
        //       << transform.X().value() << "/"
        //       << transform.Y().value() << "   "
        //       << transform.Rotation().Degrees().value() << "  "
        //       << (double) xSpeed << "/"
        //       << (double) ySpeed << "/"
        //       << (double) rot << "   "
        //       << AutodegreesToTurn
        //       // << std::endl
        //       // << "Destination Pose: "
        //       // << DestinationPose.X().value() << ", "
        //       // << DestinationPose.Y().value() << ", "
        //       // << DestinationPose.Rotation().Degrees().value()
        //       // << std::endl
        //       << "Estimated Pose: "
        //       << m_swerve.m_poseEstimator.GetEstimatedPosition().X().value() << ", "
        //       << m_swerve.m_poseEstimator.GetEstimatedPosition().Y().value() << ", "
        //       << m_swerve.m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value()
        //       << std::endl;
              }
            

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

  units::angular_velocity::radians_per_second_t GetATagVariables() {
    gyroYawHeading = m_swerve.GetYaw();
    gyroYawRate = m_swerve.GetRate();

    double dEventualYaw = (double) gyroYawHeading + (0.5 / 600.0) * (double) gyroYawRate * std::abs((double) gyroYawRate); // accounts for overshooting

    //find the shortest degrees to face tag
    int degreesToTurn = (int) ((double) dEventualYaw - desiredYaw) % 360;
    if (degreesToTurn > 180) degreesToTurn -= 360;
    if (degreesToTurn < -180) degreesToTurn += 360;

    // Converts degrees from vision thread to radians per second.
    // Also converts 
    return (units::angular_velocity::radians_per_second_t) degreesToTurn * M_PI / 180;
  }

  void shooterSetVoltage(units::volt_t voltage){
    m_RightShooterMotor.SetVoltage(voltage);
    m_LeftShooterMotor.SetVoltage(-voltage);
  }

  void drivebrake(){
    m_swerve.Drive(units::velocity::meters_per_second_t{ 0.0 }, 
                       units::velocity::meters_per_second_t{ 0.0 },
                       units::angular_velocity::radians_per_second_t{ 0.0 }, 
                       false, true);
  }
  
  #define ROBOCTOPI_RAND_MAX 39
  const std::string splashes[ROBOCTOPI_RAND_MAX+1] = {
      "I am a rat I am a theif I will still your ethernet cable. - The Rat",
      "erROAR",
      "Thanks C418!",
      "we love std::",
      "Lets try this, hahahahaha...",
      "|:~{",
      "People told me, you've changed you've changed and I pooped my pants",
      "I touched the intake and it feel apart. I dont think the robot is ready yet",
      "Finally fixed ... only took us 7 hours",
      "Minecraft Music <3",
      "Roboctopi <3",
      "BURT",
      "Team Fourty-Nine Eighteen",
      "Thank you Jeff! <3"
      "HANK THE TANK",
      "go check out stantoncomet.github.io",
      "Time for a snack break",
      "Crabs are like jelly donuts",
      "We're all gay for each other",
      "When in doubt, pull it out",
      "River Otters are so cool!",
      "Underwater robotics forever!!!!",
      "Come on down to Elevated!",
      "Yeah, Might be all that you get",
      "Yeah, Guess this might as well be it",
      "Heaven knows I tried",
      "☝️🤓 errmm actually",
      "I'm an object in motion, I've lost all emotion, my two legs are broken, but look at me dance",
      "There's a hole in the bottom of my brain...",
      "I wish I could act in a show on TV, 'Cause then I could practice not being me",
      "I'm the BEARer of bad news",
      "The coders are crying for help",
      "Put your hands up 'cause I won't",
      "Don't look at me, I'm just too dumb",
      "You like JAZZ?",
      "You got this! We believe in you!",
      "You got this! We believe in you!",
      "You got this! We believe in you!",
      "You got this! We believe in you!",
      "You got this! We believe in you!",
      "But I'm weak, and what's wrong with that?"
    };

  /**
   * Required for proper functionality of the ADIX-BT0M converter
  */
  void ExecutiveImportGradleSourceDirective() {
    int randMessage = rand() % ROBOCTOPI_RAND_MAX;
    std::cout << splashes[randMessage]
              << std::endl;
    return;
  }

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif