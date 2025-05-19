// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


//CTRE Imports
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.ClosedLoopSlot;
//REV Imports
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

 //Constants class
 public final class Constants 
 {


  

  public static final class ModuleConstants 
  {

      public static final double kDrivingMotorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRpm / 60;
      
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);//0.1016m
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;// 0.1016*3.14=
      
      public static final double kDriveL3= 1.0/6.12;
      public static final double kTurningL3= 1.0/12.8;

      public static final double kDriveL4= 1.0/5.14;
      public static final double kTurningL4= 1.0/12.8;

      
      public static final double kDriveMotorGearRatio = kDriveL3;   //1 / 5.14; //L4   L3=1/6.12
      public static final double kTurningMotorGearRatio = kTurningL3; //1 /12.8; //   1/18.0
      
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * (2 * Math.PI);

      //public static final double kTurningEncoderPositionFactor =2 * Math.PI; // radians
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
      public static final double kPTurning = 2.85;//ANTERIOR 0,5 //0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kITurning = 0;
      public static final double kDTurning = 0;


      //The F parameter should only be set when using a velocity-based PID controller, 
      //and should be set to zero otherwise to avoid unwanted behavior.
      public static final double kTurningFF = 0; //1.0/473.0; //
      
      
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
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast; // template was coast SparkMax
      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake; // TalonFx

      
      //Spark Slot
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
  }

  //Drive Constants
  public static final class DriveConstants 
  {

      //public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
      
        /**************************************
         * Specify the kinematics of our robot*
         * ************************************/
      public static final double kTrackWidth =Units.inchesToMeters(21);
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(21);
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
      public static final int kFrontLeftDriveMotorPort = 15;
      public static final int kBackLeftDriveMotorPort = 12;
      public static final int kFrontRightDriveMotorPort = 10;
      public static final int kBackRightDriveMotorPort = 14;

      //Portas dos Sparks
      public static final int kFrontLeftTurningMotorPort = 16;
      public static final int kBackLeftTurningMotorPort = 11;
      public static final int kFrontRightTurningMotorPort = 9;
      public static final int kBackRightTurningMotorPort = 13;

      //CanCoder ports
      public static final int kFrontLeftDriveAbsoluteEncoderPort = 5;
      public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
      public static final int kFrontRightDriveAbsoluteEncoderPort = 7;
      public static final int kBackRightDriveAbsoluteEncoderPort = 6;
      public static final int kPigeonPort = 30;//7563
      public static final int kPigeonPort2 = 31;//9085
     
      
        
      

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
      
      public static final double kPhysicalMaxSpeedMetersPerSecond = 5; //kRAKEN - GEAR RATIO L4
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
      
      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.052;// anterior 1.175  4,25531914893617  
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.75;//anterior 2
      
      //Slew Rate adjustments
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.5;//anterior 2 - 2/12/2024 MNL
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2.5;//anterior 2 2/12/2024 MNL
      

      //it's not currently used
      /*public static final double kDirectionSlewRate = 1.2; // radians per second
      public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
      public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)*/
    
      
      public static final String limelightFront = "limelight-front";
      public static final String limelightBack = "limelight-back";

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
      
  }

  //Path Planner Constants
  public static final class PathPlannerConstants
  {
    public static final double kPTranslationPath = 10;//Anterior 0.125,
    public static final double kITranslationPath = 0.08;
      public static final double kPRotationPath = 3.0;//3.0; //3.0; 1.6

      public static final PPHolonomicDriveController AutoConfig = new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(kPTranslationPath, kITranslationPath, 0), // Translation PID constants
        new PIDConstants(kPRotationPath, 0.0, 0.0) // Rotation PID constants
      );

      public static final double maxAccelerationPath = 1;//1.75//1.0//5.0 //3.0
      public static final double maxAngularVelocityRadPerSec = Units.degreesToRadians(540);
      public static final double maxAngularAccelerationRadPerSecSq = Units.degreesToRadians(720);
  }

  //Subsystem Constants
  public static final class SubsystemsConstants
  {
    //Lift Constants
    public static final class LiftConstants
    {
      //Lift Motor ports
      public static final int kLiftLeftMotorId = 25;
      //public static final int kLiftRightMotorId = 23;

      //Lift Constants gear box
      public static final double kLiftPulleyRatio = 20.0/42.0;
     
      //public static final double kLiftPulleyRatio = 42.0/20.0;
      public static final double kLiftTuboDiam = 0.0381; //0.033 
      public static final double  kLiftGearRatio = 1.0/3.0*kLiftPulleyRatio*kLiftTuboDiam*Math.PI;

      public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake; // TalonFx

      
      public static final double KS = 0.0;//0.25;//<-MNL 11/11/2024 0.1; //0.32;  // Add 0.1 V output to overcome static friction
      public static final double KV = 0.0;//0.12; //1.51; // A velocity target of 1 rps results in 0.12 V output
      public static final double KA = 0.0;//0.01;  //0.27;

      //TO DO: This must be tuned to specific robot
      public static final double kP = 50.0;//50//35// 0.0665;//ANTERIOR 0,0665 ; 0.05872615 // kP = 0.11 An error of 1 rps results in 0.11 V output
      public static final double kI =0.0;
      public static final double kD=0.0;
      
      public static final double kFF = 1.0/6000.0;
      public static final double kMinOutput= -1;
      public static final double kMaxOutput = 1;

      //Motion Magic PID - TO DO
      public static final double KS1 =0.0;//0.25;//<-MNL 11/11/2024 0.1; //0.32;  // Add 0.1 V output to overcome static friction
      public static final double KV1 = 0.0;//0.12; //1.51; // A velocity target of 1 rps results in 0.12 V output
      public static final double KA1 = 0.0;//0.01;  //0.27;
      public static final double kP1 = 2.4;
      public static final double kI1 =0.0;
      public static final double kD1=0.0;


      /* Current Limiting */
     //TalonFX
      public static final int kCurrentLimit = 70; //supply current
      public static final int kCurrentThreshold = 120; //stator current
      public static final double kCurrentThresholdTime = 0.1;
      public static final boolean kEnableCurrentLimit = true;
      public static final double kClosedLoopRamp=0.25;
      public static boolean kInverted = false;
    
      
      /* 
      //Lift SparkMax
      public static final IdleMode kLiftIdleMode = IdleMode.kBrake;
      public static final int kLiftCurrentLimit = 40;
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
      public static final double kLiftEncoderPositionPIDMinInput =0 ;
      public static final double kLiftEncoderPositionPIDMaxInput= 2 * Math.PI;

      //Lift Encoder
      public static final double kLiftPositionConversionFactor = kLiftGearRatio* Math.PI * kLiftPulleyRatio * kLiftTuboDiam; // rotation to meters
      public static final double kLiftVelocityConversionFactor = kLiftPositionConversionFactor/60; //meters/sec (2 * Math.PI) / 60.0; // radians per second
      //Lift PID
      public static final double kPLiftUp = 1.0;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kILiftUp = 0;
      public static final double kDLiftUp = 0;
      public static final double kLiftUpFF = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kLiftMinOutput = -1;
      public static final double kLiftMaxOutput = 1;

      public static final double kPLiftDown = 0.5;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kILiftDown = 0;
      public static final double kDLiftDown = 0;
      public static final double kLiftDownFF = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      
      //Lift Soft Limits
      public static final double kSoftLimitMin = -0.1;
      public static final double kSoftLimitMax = 1.0;*/
      
    } 

    public static final class CoralIntakeConstants
    {
      
      //Coral Motor ports
      public static final int kCoralMotorId = 20;//3;

      // TO DO: This must be tuned to specific robot
      public static final double  kGearRatio = 1.0;
      public static final double kWheelDiameter = 0.0313;//
      

      public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake; // 
     

      
      public static final double KS = 0.1;//<-MNL 11/11/2024 0.1; //0.32;  // Add 0.1 V output to overcome static friction
      public static final double KV = 0.12; //1.51; // A velocity target of 1 rps results in 0.12 V output
      public static final double KA = 0.11;  //0.27;

      //TO DO: This must be tuned to specific robot
      public static final double kP =2.0;//<--MNL11/11/2024 0.0665;//ANTERIOR 0,0665 ; 0.05872615 // kP = 0.11 An error of 1 rps results in 0.11 V output
      public static final double kI =0.0;
      public static final double kD=0.0;
      
      public static final double kFF = 1.0/6000.0;
      public static final double kMinOutput= -1;
      public static final double kMaxOutput = 1;

      public static final double kP1 = 2.4;
      public static final double kI1 =0.0;
      public static final double kD1=0.0;


      /* Current Limiting */
     //TalonFX
      public static final int kCurrentLimit = 70; //supply current
      public static final int kCurrentThreshold = 120; //stator current
      public static final double kCurrentThresholdTime = 0.1;
      public static final boolean kEnableCurrentLimit = true;
      public static final double kClosedLoopRamp=0.25;
      public static boolean kInverted = false;


    }

    public static final class AlgaIntakeConstants
    {
      //Alga Motor ports
      public static final int kMotorId =23 ; 

      // TO DO: This must be tuned to specific robot
      public static final double  kGearRatio = (1.0/3.0);
      public static final double kPulleyRatio = 41.0/31.84;
      public static final double kWheelDiameter = 0.04147;
      
      
      //Alga SparkMax
      public static final IdleMode kAlgaIdleMode = IdleMode.kBrake;
      public static final int kAlgaCurrentLimit = 20;
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
      public static final double kAlgaEncoderPositionPIDMinInput =0 ;
      public static final double kAlgaEncoderPositionPIDMaxInput= 2 * Math.PI;

      //AlgaPositions
      public static final double articulatorVelocity= 0.2;
      public static final double pickAlgaePIDPosition= 3.25;
      public static final double startAlgaePIDPosition= 0;

      public static final double shooterAlgaeVelocity= 0.5;



      //Alga Encoder - TO DO: This must be tuned to specific robot
      public static final double kAlgaPositionConversionFactor =Math.PI* kGearRatio *kPulleyRatio *kWheelDiameter;//rotation to Meters
      public static final double kAlgaVelocityConversionFactor = kAlgaPositionConversionFactor /60.0;//rotation to m/s

      //Alga PID
      public static final double kP = 1;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kFF = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;

    }

    //ArticuladorAlgae Constants - SparkMax
    public static final class ArticuladorAlgaeConstants
    {
      //ArticuladorAlgae
      public static final int kArticuladorAlgaeMotorId = 22;

      // TO DO: This must be tuned to specific robot
      public static final double  kGearRatio = 1.0/12.0;
      public static final double kPulleyRatio = 42.0/96.0;
      
      
      //SoftLimit
      public static final double kSoftLimitMin = 0;
      public static final double kSoftLimitMax = 80;

      //ArticuladorAlgae SparkMax
      public static final IdleMode kArticuladorAlgaeIdleMode = IdleMode.kBrake;
      public static final int kArticuladorAlgaeCurrentLimit = 60;
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
      public static final double kArticuladorAlgaeEncoderPositionPIDMinInput =0 ;
      public static final double kArticuladorAlgaeEncoderPositionPIDMaxInput= 2 * Math.PI;

      //ArticuladorAlgae Encoder
      
      //TO DO: This must be tuned to specific robot
      public static final double kArticuladorAlgaePositionConversionFactor = 360*kGearRatio *kPulleyRatio; //rotation to graus
      public static final double kArticuladorAlgaeVelocityConversionFactor = kArticuladorAlgaePositionConversionFactor/60.0; //rotation to m/s

      //ArticuladorAlgae PID Up
      public static final double kPslot0 = 0.01;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kIslot0 = 0;
      public static final double kDslot0 = 0;
      public static final double kFFslot0 = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kMinOutputSlot0 = -1;
      public static final double kMaxOutputSlot0 = 1;

      //ArticuladorAlgae PID Down
      public static final double kPslot1 = 0.01;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kIslot1 = 0;
      public static final double kDslot1 = 0;
      public static final double kFFslot1 = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      
    } 

    //ArticuladorCoral Constants - SparkMax
    public static final class ArticuladorCoralConstants
    {
      //ArticuladorCoral Motor ports
      public static final int kArticuladorCoralMotorId = 21;


      // TO DO: This must be tuned to specific robot
      public static final double kGearRatio = (1.0/3.0)*(20.0/68.0); // 
      public static final double kPulleyRatio = 20.0/42.0;
      public static final double kWheelDiameter = 0.013; //meters

      public static final double kCoralOffSet = 5; //meters
      

      //ArticuladorCoral SparkMax
      public static final IdleMode kArticuladorCoralIdleMode = IdleMode.kBrake;
      public static final int kArticuladorCoralCurrentLimit = 40;
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
      public static final double kArticuladorCoralEncoderPositionPIDMinInput =0 ;
      public static final double kArticuladorCoralEncoderPositionPIDMaxInput= 2 * Math.PI;

      //ArticuladorCoral Encoder
      
      public static final double kArticuladorCoralPositionConversionFactor =Math.PI* kGearRatio *kPulleyRatio*100; //rotation to graus
      public static final double kArticuladorCoralVelocityConversionFactor = kArticuladorCoralPositionConversionFactor/60.0; //rotation to m/s

      //ArticuladorCoral PID Up
      public static final double kPslot0 = 0.3;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kIslot0 = 0;
      public static final double kDslot0 = 0;
      public static final double kFFslot0 = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kslot0MinOutput = -1;
      public static final double kslot0MaxOutput = 1;

       //ArticuladorCoral PID Up
       public static final double kSslot1 = 0.0;//0.25; // Add 0.25 V output to overcome static friction
       public static final double kVslot1 = 0.0;//0.12; // A velocity target of 1 rps results in 0.12 V output
       public static final double kAslot1 = 0.0;//0.01; // An acceleration of 1 rps/s requires 0.01 V output
       public static final double kPslot1 = 0.1;//0.15//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
       public static final double kIslot1 = 0;
       public static final double kDslot1 = 0;
       public static final double kFFslot1 = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      // public static final double kslot1MinOutput = -1;
       //public static final double kslot1MaxOutput = 1;

        //ArticuladorCoral PID Up
      public static final double kPslot2= 1.0;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kIslot2 = 0;
      public static final double kDslot2 = 0;
      public static final double kFFslot2 = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kslot2MinOutput = -1;
      public static final double kslot2MaxOutput = 1;

      


    public static final double kSoftLimitMin = 0;
    public static final double kSoftLimitMax = 82;
      
    } 

    //Ramp Constants - SparkMax
    public static final class RampConstants
    {
      //Ramp
      public static final int kRampMotorId = 17;
     

      //  TO DO: This must be tuned to specific robot
      public static final double kGearRatio = 1.0/9.0; // 
      public static final double kPulleyRatio = 20.0/42.0;
      
      

      //Ramp SparkMax
      public static final IdleMode kRampIdleMode = IdleMode.kBrake;
      public static final int kRampCurrentLimit = 60;
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
      public static final double kRampEncoderPositionPIDMinInput =0 ;
      public static final double kRampEncoderPositionPIDMaxInput= 2 * Math.PI;

      //Ramp Encoder
      //TO DO: This must be tuned to specific robot
      public static final double kRampPositionConversionFactor = 360* kGearRatio * kPulleyRatio; //rotation to Meters
      public static final double kRampVelocityConversionFactor = kRampPositionConversionFactor/60.0; //rotation to m/s

      //Ramp PID Up
      public static final double kPslot0 = 0.15;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kIslot0  = 0;
      public static final double kDslot0  = 0;
      public static final double kFFslot0  = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kMinOutputslot0  = -1;
      public static final double kMaxOutputslot0  = 1;

      //Ramp PID Down
      public static final double kPslot1 = 1.3;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kIslot1 = 0;
      public static final double kDslot1 = 0.7;
      public static final double kFFslot1 = 0; // = input = 0 to position control and 1.0/473.0 to velocity control

      
      public static final double startPosition= 0;
      public static final double pickPosition= -4;

      public static double kSoftLimitMax = 110;
      public static double kSoftLimitMin = 0;
    } 



    public static final class EscaladorConstants
    {
      //Escalador Motor ports
      public static final int kMotor1Id = 26;//02/18/2025 estava igual ao modulo serve front left
      //public static final int kMotor2Id = 25;
      public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake; // TalonFx

      //  TO DO: This must be tuned to specific robot
      public static final double kGearRatio = 1.0/144.0; // 

      //Climber SparkMax
      public static final IdleMode kIdleMode = IdleMode.kBrake;
      //public static final int kCurrentLimit = 95;
      public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;
      public static final double kEncoderPositionPIDMinInput =0 ;
      public static final double kEncoderPositionPIDMaxInput= 2 * Math.PI;

      //Climber Encoder
      //TO DO: This must be tuned to specific robot
      public static final double kPositionConversionFactor = Math.PI* kGearRatio ; //rotation to Meters
      public static final double kVelocityConversionFactor = kPositionConversionFactor/60.0; //rotation to m/s

      //Climber PID 
      public static final double kP = 100;//ANTERIOR 0,25 ; 0.2142875; 0.175; 0.125; 0.23
      public static final double kI= 0;
      public static final double kD = 0;
      public static final double kFF = 0; // = input = 0 to position control and 1.0/473.0 to velocity control
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
      public static double kSoftLimitMax;
      public static double kSoftLimitMin;

      
    //Escalator

    //public static final double escaladorVelocity= 0.8;

    public static final double escaladorClimbPosition= 1.18;

    public static final double escaladorBackPosition= 0;
      

    /* 
      public static final double KS = 0.1;//<-MNL 11/11/2024 0.1; //0.32;  // Add 0.1 V output to overcome static friction
      public static final double KV = 0.12; //1.51; // A velocity target of 1 rps results in 0.12 V output
      public static final double KA = 0.11;  //0.27;

      //TO DO: This must be tuned to specific robot
      public static final double kP = 0.0665;//<--MNL11/11/2024 0.0665;//ANTERIOR 0,0665 ; 0.05872615 // kP = 0.11 An error of 1 rps results in 0.11 V output
      public static final double kI =0.0;
      public static final double kD=0.0;
      
      public static final double kFF = 1.0/6000.0;
      public static final double kMinOutput= -1;
      public static final double kMaxOutput = 1;

      public static final double kP1 = 2.4;
      public static final double kI1 =0.0;
      public static final double kD1=0.0;


      /* Current Limiting */
     //TalonFX
      public static final int kCurrentLimit = 70; //supply current
      public static final int kCurrentThreshold = 120; //stator current
      public static final double kCurrentThresholdTime = 0.1;
      public static final boolean kEnableCurrentLimit = true;
      public static final double kClosedLoopRamp = 0.25;
      public static boolean kInverted = false;

    }
  } 


  //OI Constants
  public static final class OIConstants 
  {
      
      public static final class JoystickDriverConstants 
      {
        //Joystick Driver
        public static final int kDriverControllerPort = 0;
        
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        //public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        
        public static final double kDeadband = 0.1;// 0.05 anterior
        //public static final int kRumbleOn = 1;
        //public static final int kRumbleOff = 0;
      
      }


      public static final class JoystickOperatorConstants 
      {

      //Joystick Provisory Operator
      public static final int kOperatorControllerPort = 1;
        
      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      //public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;
      
      public static final double kDeadband = 0.1;// 0.05 anterior
      }

      /*/public static final class JoystickOperatorConstants 
      {
        //Joystick Operator
        public static final int kOperatorControllerPort = 1;
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
      }*/



      public static final class MesinhaJoy1Constants {
        //First part of operator Hub
        public static final int kJoy1Port = 1;
        public static final double kOperatorDeathBand = 0.05;
        public static final int kNine = 9;      //ok
        public static final int kTen = 10;        //ok
        public static final int kTwelve = 12;         //ok
        public static final int kEleven = 11;       //ok   
        public static final int kSix = 6;       //ok  
        public static final int kEight = 8;         //ok
        public static final int kSeven = 7;      //ok 
        public static final int kOne = 1; 

        public static final int kBlackAxisUp = 0;     //ok
        public static final int kBlackAxisSides = 1;  //ok 
      }



      public static final class MesinhaJoy2Constants {
        //Second part of operator Hub
        public static final int kJoy2Port = 2;
        public static final double kOperatorDeathBand = 0.05;
        public static final int kThree = 3;      //ok
        public static final int kTwo = 2;      //ok
        public static final int kOne = 1; 
        public static final int kSix = 6;      //ok 
        
        public static final int kSeven = 7;      //ok 
        public static final int kEight = 8;    
        
        public static final int kRedAxisUp = 0;     //ok
        public static final int kRedAxisSides = 1;  //ok

      }


      
      
      public static final double kDeadband =0.1;//0.1

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
