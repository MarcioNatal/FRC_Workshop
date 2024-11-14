// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

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
public final class Constants {
  public static final class Swerve {
    //Removido, use o kTrackWidth e kWheelBase
    // Locations for the swerve drive modules relative to the robot center.
    //public static final Translation2d flModuleOffset = new Translation2d(0.381, 0.381);
    //public static final Translation2d frModuleOffset = new Translation2d(0.381, -0.381);
    //public static final Translation2d blModuleOffset = new Translation2d(-0.381, 0.381);
    //public static final Translation2d brModuleOffset = new Translation2d(-0.381, -0.381);

    //Angle offset para determinar a frente do robô de acordo com o pigeon2 (novo 0 graus)
    public static final double kAngleOffset = 0.0;
    public static final double maxModuleSpeed = Units.feetToMeters(16.5);//4.5; // M/S

    public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);
  }
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/6.12;//1 / 5.8462;//how uch the motor spins to make the wheel spin 1 revolution
    //neo = 42 counts per revolution
    //6.12:1 gear ratio (MK4i)
    //kraken = 2048 (?) counts per revolution
    //kDriveMotorGearRatio = 1 motor count = 1 revolution
    public static final double kTurningMotorGearRatio = 1.0/(150.0/7.0);//1 / 18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double driveCurrentThreshold = 120;
    public static final boolean driveEnableCurrentLimit = true;
    public static final double kDriveClosedLoopRamp=0.25;
    public static final double kPTurning = 0.5;
  }
  public static final class DriveConstants {

      public static final double kTrackWidth = 0.6;//Units.inchesToMeters(21);
      // Distance between right and left wheels
      public static final double kWheelBase = 0.6;//Units.inchesToMeters(25.5);
      // Distance between front and back wheels
//Assumindo que o centro do robô é o centro da rodas:
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
              
      /*public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));*/

      public static final int kFrontLeftDriveMotorPort = 1;
      public static final int kBackLeftDriveMotorPort = 4;//2;
      public static final int kFrontRightDriveMotorPort = 2;//3;
      public static final int kBackRightDriveMotorPort = 3;//4;

      public static final int kFrontLeftTurningMotorPort = 21;//11;
      public static final int kBackLeftTurningMotorPort = 24;//22;
      public static final int kFrontRightTurningMotorPort = 22;//33;
      public static final int kBackRightTurningMotorPort = 23;//44;

      public static final boolean kFrontLeftTurningEncoderReversed = true;
      public static final boolean kBackLeftTurningEncoderReversed = true;
      public static final boolean kFrontRightTurningEncoderReversed = true;
      public static final boolean kBackRightTurningEncoderReversed = true;

      public static final boolean kFrontLeftDriveEncoderReversed = false;
      public static final boolean kBackLeftDriveEncoderReversed = false;
      public static final boolean kFrontRightDriveEncoderReversed = false;
      public static final boolean kBackRightDriveEncoderReversed = false;

      public static final int kFrontLeftDriveAbsoluteEncoderPort = 11;//10;
      public static final int kBackLeftDriveAbsoluteEncoderPort = 14;//20;
      public static final int kFrontRightDriveAbsoluteEncoderPort = 12;//30;
      public static final int kBackRightDriveAbsoluteEncoderPort = 13;//40;

      public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
      public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

      public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.610+Math.PI;//+math.pi para inverter
      public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.126+Math.PI;
      public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -2.057;
      public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.961;

      public static final double kPhysicalMaxSpeedMetersPerSecond = Swerve.maxModuleSpeed;//5;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;// / 4;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
              kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1;//4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;//10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 2*2*Math.PI;//Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;//the one that will move the robot sideways
    public static final int kDriverXAxis = 0;//the one that will move the robot forward
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
    //Zero the robot reference for testing, shouldn't be used in competition, make it 0 to disable
    public static final int kZeroHeadingButtonIdx = 8;
    //Zero the robot reference using vision, might not be needed in competition, but it's a good idea to have it, zero to disable
    public static final int kZeroVisionHeadingButtonIdx = 7;
    //2 speeds: (defaults to average when not pressing)
    public static final int kDriverSlow = 5;
    public static final int kDriverTurbo = 6;

    public static final double kDeadband = 0.05;
  }
}
