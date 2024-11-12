// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//CTRE Imports
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
//REV Imports
import com.revrobotics.CANSparkBase.IdleMode;

//WPI Imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 public final class Constants 
 {

  public static final class ModuleConstants 
  {

      public static final double kDrivingMotorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRpm / 60;
      
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);//0.1016m
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;// 0.1016*3.14=
      
      public static final double kDriveMotorGearRatio = 1 / 5.14; //L4   L3=1/6.12
      public static final double kTurningMotorGearRatio = 1 /12.8; //   1/18.0
      
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * (2 * Math.PI);

      public static final double kTurningEncoderPositionFactor =2 * Math.PI; // radians
      public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
      
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
      
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)*kDriveMotorGearRatio; 
      public static final double kDriveRPM = kDriveEncoderRPM2MeterPerSec*60;
      public static final double kDriveRPMRatio = kDriveRPM/kDriveMotorGearRatio;
      public static final double kDriveRPMPi = kDriveRPMRatio/Math.PI;
      public static final double kDriveRPMSpeed = kDriveRPMPi/kWheelDiameterMeters;

      /**************************************
       * Turning REV SparkMax PID settings *
       **************************************/

      //14,285%
      public static final double kPTurning = 0.5;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kITurning = 0;
      public static final double kDTurning = 0;
      public static final double kTurningFF = 1/473; // = 1/kV
      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;

      /**********************************
       * Driving TALON Fx PID settings *
       *********************************/
      /**************************************************************
       * @category Drive Motor Characterization Values From SYSID 
       * TO DO: This must be tuned to specific robot
       * ************************************************************/
      public static final double driveKS = 0.1;//<-MNL 11/11/2024 0.1; //0.32;  // Add 0.1 V output to overcome static friction
      public static final double driveKV = 0.12; //1.51; // A velocity target of 1 rps results in 0.12 V output
      public static final double driveKA = 0.11;  //0.27;

        //11,69%
      public static final double kPdriving = 0.0665;//<--MNL11/11/2024 0.0665;//ANTERIOR 0,0665 ; 0.05872615 // kP = 0.11 An error of 1 rps results in 0.11 V output
      public static final double kIdriving =0.0;
      public static final double kDdriving=0.0;
      
      public static final double kFFdriving = 1/kDriveWheelFreeSpeedRps;
      public static final double kDrivingMinOutput= -1;
      public static final double kDrivingMaxOutput = 1;


      /* Swerve Current Limiting */
     //TalonFX
      public static final int driveCurrentLimit = 70; //supply current
      public static final int driveCurrentThreshold = 120; //stator current

      public static final double driveCurrentThresholdTime = 0.1;
      public static final boolean driveEnableCurrentLimit = true;

      public static double kDriveClosedLoopRamp=0.25;

      //SparkMax
      public static final int kTurningMotorCurrentLimit = 20; // amps
      
      public static final double kTurningEncoderPositionPIDMinInput =0 ;
      public static final double kTurningEncoderPositionPIDMaxInput= 2 * Math.PI;

      /* Neutral Modes */
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake; // template was coast SparkMax
      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake; // TalonFx

      

  }

  public static final class DriveConstants 
  {

      public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
      
        /**************************************
         * Specify the kinematics of our robot*
         * ************************************/
      public static final double kTrackWidth = Units.inchesToMeters(23);
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(19.5);
      // Distance between front and back wheels
      public static final double kDriveRadius = Math.hypot(kTrackWidth/2, kWheelBase/2);
      // Distance from robot center to furthest module.
      public static final double parametro = 2;

      /**********************************************************************
       * Swerve Drive Object - It specifies the location of each swerve     *
       * module on the robot this way the wpi library can construct the     *
       * geometry of our robot setup and do all the calculations            *
       * @see  Modules Location: FL= +X,+Y; FR= +X,-Y; BL=-X, +Y; BR=-X, -Y,*
       **********************************************************************/
      public static final SwerveDriveKinematics kDriveKinematics = 
      new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //+ - antes
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  //+ + antes
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //- - antes
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) //- + antes 
                                ); 

      // 
      /**MNL  17/10/2024
      * kFrontLeftChassisAngularOffset = 0: This means that when your front left module's turning motor is at its zero position, 
      * the module is pointing straight forward relative to the chassis.
      * kFrontRightChassisAngularOffset = 0: This suggests your front right module is also pointing forward when its turning motor 
      * is at zero.
      * kBackLeftChassisAngularOffset = Math.PI: This means that your back left module is pointing 180 degrees from the front left 
      * module. So, when its turning motor is at zero, the module is pointing straight backward relative to the chassis.
      * kBackRightChassisAngularOffset = Math.PI: This means that your back right module is also pointing straight backward 
      * when its turning motor is at zero.
      ******/

      public static final double kFrontLeftChassisAngularOffset = 0;//
      public static final double kFrontRightChassisAngularOffset = Math.PI;
      public static final double kBackLeftChassisAngularOffset = 0;
      public static final double kBackRightChassisAngularOffset = Math.PI;

      // Portas dos Kraken
      public static final int kFrontLeftDriveMotorPort = 15;//10
      public static final int kBackLeftDriveMotorPort = 12;//14
      public static final int kFrontRightDriveMotorPort = 10;//15
      public static final int kBackRightDriveMotorPort = 14;//12

      //Portas dos Sparks
      public static final int kFrontLeftTurningMotorPort = 16;//9
      public static final int kBackLeftTurningMotorPort = 11;//13
      public static final int kFrontRightTurningMotorPort = 9;//16
      public static final int kBackRightTurningMotorPort = 13;//11

      //CanCoder ports
      public static final int kFrontLeftDriveAbsoluteEncoderPort = 5;//7
      public static final int kBackLeftDriveAbsoluteEncoderPort = 4;//6
      public static final int kFrontRightDriveAbsoluteEncoderPort = 7;//5
      public static final int kBackRightDriveAbsoluteEncoderPort = 6;//4

     
      
        
      

      /**SINTONIA DAS RODAS
       * FL => FRONT LEFT 
       * FR => FRONT RIGHT
       * BL => BACK LEFT 
       * BR => BACK RIGHT
       */

      public static final Rotation2d angleOffsetFLTurning = Rotation2d.fromDegrees(-4.85);
      public static final Rotation2d angleOffsetFRTurning = Rotation2d.fromDegrees(-77.43);
      public static final Rotation2d angleOffsetBLTurning = Rotation2d.fromDegrees(142.207);
      public static final Rotation2d angleOffsetBRTurning = Rotation2d.fromDegrees(-91.14);


      /*******************************************************
       * constante que limita a velocidade maxima drive teleop*
       * ******************************************************/
      
      public static final double kPhysicalMaxSpeedMetersPerSecond = 6; //kRAKEN - GEAR RATIO L4
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
      
      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.052;// anterior 1.175  4,25531914893617  
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.75;//anterior 2
      
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.0;//anterior 3
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2.0;//anterior 3
      

      public static final double kDirectionSlewRate = 1.2; // radians per second
      public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
      public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    

  }

 /*************************************************************
  * constante que limita a velocidade maxima drive autonomous *
  *************************************************************/
  public static final class AutoConstants 
  {
      public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 6;  //4
      public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;//10,20
      
      public static final double kMaxAccelerationMetersPerSecondSquared = 3; //3
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2; //4
      
      public static final double kPXController = 1.5;
      public static final double kPYController = 1.5;
      public static final double kPThetaController = 3;
      
      public static final double kOffset = Units.inchesToMeters(9);
      public static final double kOffsetSide = Units.inchesToMeters(3);

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                        new TrapezoidProfile.Constraints( kMaxAngularSpeedRadiansPerSecond,
                                                          kMaxAngularAccelerationRadiansPerSecondSquared
                                                        );
      
      public static final double kPTranslationPath = 0.14;//Anterior 0.125,
      public static final double kPRotationPath = .35;
      
      public  static final HolonomicPathFollowerConfig pathAutoConfig = 
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                  new PIDConstants(kPTranslationPath, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(kPRotationPath, 0.0, 0.0), // Rotation PID constants
                  4.8, // Max module speed, in m/s
                  DriveConstants.kDriveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                  new ReplanningConfig() // Default path replanning config. See the API for the options here
            );
            
  }


  public static final class OIConstants 
  {
      
      public static final class JoystickDriverConstants 
      {
        //Joystick Driver
        public static final int kDriverControllerPort = 0;
        
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        
        public static final double kDeadband = 0.1;// 0.05 anterior

        public static final int kRumbleOn = 1;
        public static final int kRumbleOff = 0;
      
      }

      public static final class JoystickOperatorConstants 
      {
        //Joystick Operator
        public static final int kOperatorControllerPort = 3;
        public static final double kOperatorDeathBand = 0.05;
        public static final int kX = 1;
        public static final int kA = 2;
        public static final int kB = 3;
        public static final int kY = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftTrigger = 7;
        public static final int kRightTrigger = 8;
        public static final int kBack = 9;
        public static final int kStart = 10;
        public static final int kLeftStick = 11;
        public static final int kRightStick = 12;
      }

      
      //Set up camera resolution

      public static final int cameraWheightRes = 320;
      public static final int camerHeightRes = 240;
      public static final double kDeadband =0.1;
}

  
  /*******************************************************************
   * @param https://docs.revrobotics.com/brushless/neo/v1.1/neo-v1
   * 
   *****************************************************************/
  

  public static final class NeoMotorConstants 
  {
    public static final double kFreeSpeedRpm = 5676;
  }

  /***********************************************************************************************************************************
   * @param https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
   * 
   * @param https://store.ctr-electronics.com/announcing-kraken-x60/?srsltid=AfmBOorh3sPSXQ-WmuWYeJlxrIkATC1wRVPc0V65woNtynzQ1Sil1Ueh
   *
   *************************************************************************************************************************************/
  public static final class KrakenMotorConstants 
  {
    public static final double kFreeSpeedRpm = 6000;
  }

  
}
