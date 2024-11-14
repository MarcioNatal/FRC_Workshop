// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.RobotContainer;
import frc.robot.Util.Utilidades;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystem extends SubsystemBase {

  //private final SimSwerveModule[] modules;//simulado
  private double kLimeAngleOffset = 0.0;
  private final SwerveModule[] modules;//REAL
  private final SwerveDriveKinematics kinematics;
  //private final SwerveDriveOdometry odometry;
  final SwerveDrivePoseEstimator odometry;
  public boolean isOnRedAlliance = false;//Assumir que está na aliança azul
  public boolean kVisionButtonEnabled = Constants.OperatorConstants.kZeroHeadingButtonIdx != 0;
  public double SpeedLimiter = 0.6;//default speed 70%

  Pigeon2 gyro = new Pigeon2(5);
  //private SimGyro gyro;
  
  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> publisher_target = NetworkTableInstance.getDefault()
  .getStructArrayTopic("/SwerveTargers", SwerveModuleState.struct).publish();
  //StructArrayPublisher<SwerveModuleState> calibration_publisher = NetworkTableInstance.getDefault()
  //.getStructArrayTopic("/SwerveCANs", SwerveModuleState.struct).publish();

  private Field2d field = new Field2d();

  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  // Locations for the swerve drive modules relative to the robot center.
  //Translation2d m_frontLeftLocation = Constants.Swerve.flModuleOffset;//new Translation2d(0.381, 0.381);
  //Translation2d m_frontRightLocation = Constants.Swerve.frModuleOffset;//new Translation2d(0.381, -0.381);
  //Translation2d m_backLeftLocation = Constants.Swerve.blModuleOffset;//new Translation2d(-0.381, 0.381);
  //Translation2d m_backRightLocation = Constants.Swerve.brModuleOffset;//new Translation2d(-0.381, -0.381);

  /*private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, gyro.getRotation2d(), 
    new SwerveModulePosition[]{
        new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
        new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
        new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
        new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))  
    }, new Pose2d(5.0, 13.5, new Rotation2d()));//isso ta certo??
*/

  public SwerveSubsystem() {
    //real
    modules = new SwerveModule[]{
      backLeft,
      backRight,
      frontLeft,
      frontRight
    };
    //simulation
    /* 
    gyro = new SimGyro();
    modules = new SimSwerveModule[]{
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule()
    };*/

    kinematics = DriveConstants.kDriveKinematics;/*new SwerveDriveKinematics(//ordem estava errada, não resolveu
      m_backLeftLocation, m_backRightLocation, m_frontLeftLocation, m_frontRightLocation
    );*/
    

    /*
    kinematics = new SwerveDriveKinematics(
      Constants.Swerve.flModuleOffset,
      Constants.Swerve.frModuleOffset,
      Constants.Swerve.blModuleOffset,
      Constants.Swerve.brModuleOffset
    );*/

    //sim
    //odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
    //real:
    odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getPositions(), new Pose2d(0, 0, new Rotation2d()));
    //new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          Constants.Swerve.translationConstants,
          Constants.Swerve.rotationConstants
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  public void zeroHeading() {
    //gyro.reset();//é assim mesmo?
    //real
    gyro.reset();
    //simulation
    //gyro.currentRotation = new Rotation2d();
  }

  public double getHeading() {//TO.DO: Auto calibrate angle with aprilltags
    //real
    return Math.IEEEremainder(gyro.getYaw().getValue()+Constants.Swerve.kAngleOffset+kLimeAngleOffset, 360);
    //return Math.IEEEremainder(gyro.getYaw().getValue()+Constants.Swerve.kAngleOffset, 360);
    //simulation
    //return Math.IEEEremainder(gyro.getRotation2d().getDegrees(), 360);
  }

  public double getHeading_noVision() {
    return Math.IEEEremainder(gyro.getYaw().getValue()+Constants.Swerve.kAngleOffset, 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // Update the simulated gyro, not needed in a real project
    //simulation only
    //gyro.updateRotation(getSpeeds().omegaRadiansPerSecond);

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red)
      {
        isOnRedAlliance = true;
      }
      else
      {
        isOnRedAlliance = false;
      }
    }

    odometry.update(gyro.getRotation2d(), getPositions());

    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        odometry.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {//TO.DO: Check that angle, seems ok, but is it?
      LimelightHelpers.SetRobotOrientation("limelight", -gyro.getAngle(),0,0,0,0,0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
        //TODO: auto calibrate in slow movements and or stopped (done) and or button pressed (done)
        //if the robot is currently disabled or the calibration button was pressed
        //need a way to get if button kZeroVisionHeadingButtonIdx is currently pressed
        if ((kVisionButtonEnabled && RobotContainer.XboxCommander.getHID().getRawButton(Constants.OperatorConstants.kZeroVisionHeadingButtonIdx)) || (DriverStation.isDisabled() || DriverStation.isTest() || (!DriverStation.isTeleopEnabled() && !DriverStation.isAutonomousEnabled())))//Enter this only if disabled, takes 2ms to process
        {//grab current vision angle and use as offset
          //System.out.println("Disabled robot tasks");
          LimelightResults results = LimelightHelpers.getLatestResults("limelight");
          if (results.valid) 
          {
            // AprilTags/Fiducials
            if (results.targets_Fiducials.length > 0) 
            {
              LimelightTarget_Fiducial tag = results.targets_Fiducials[0];//select the main target
              double id = tag.fiducialID;          // Tag ID
              //String family = tag.fiducialFamily;   // Tag family (e.g., "16h5")
              
              // 3D Pose Data
              Pose3d robotPoseField = tag.getRobotPose_FieldSpace();    // Robot's pose in field space
              Rotation3d rotacao = robotPoseField.getRotation();
              double val = Units.radiansToDegrees(rotacao.getZ());//get yaw
              //kLimeAngleOffset = val;
              //if id equals to 3, 4 or 13
              /* Doesn't matter!!!!
              if(id == 3 || id == 4 || id == 13)//apriltags on the red alliance side
              {
                if (isOnRedAlliance)//olhando pra red sendo da red, vira pro outro lado
                {}
                else
                {val = val + 180;}
              }
              else if(id == 7 || id == 8 || id == 14)//apriltags on the blue alliance side
              {
                if (isOnRedAlliance)
                {}
                else//olhando pra blue sendo da blue, vira pro outro lado
                {val = val + 180;}
              }*/
              val -= getHeading_noVision();//subtract the current offset
              /*Pose3d cameraPoseTag = tag.getCameraPose_TargetSpace();   // Camera's pose relative to tag
              Pose3d robotPoseTag = tag.getRobotPose_TargetSpace();     // Robot's pose relative to tag
              Pose3d tagPoseCamera = tag.getTargetPose_CameraSpace();   // Tag's pose relative to camera
              Pose3d tagPoseRobot = tag.getTargetPose_RobotSpace();     // Tag's pose relative to robot
              
              // 2D targeting data
              double tx = tag.tx;                  // Horizontal offset from crosshair
              double ty = tag.ty;                  // Vertical offset from crosshair
              double ta = tag.ta;                  // Target area (0-100% of image)
              */
              //debugging
              //System.out.println("Tag position data: "+robotPoseField);
              //get difference between kLimeAngleOffset and val and if over 1, set kLimeAngleOffset to val
              if(Math.abs(Utilidades.AngleDist(kLimeAngleOffset,val))> 2)
              {
                //limit the doubles to 2 decimal places
                System.out.println("Tag [" + id + "] with yaw [" + String.format("%.2f", kLimeAngleOffset) + "] set to [" + String.format("%.2f", val) + "] Diff: "+Math.abs(Utilidades.AngleDist(kLimeAngleOffset, val)));// etrot <"+ String.format("%.2f", rotacao.getX())+","+String.format("%.2f", rotacao.getY())+","+String.format("%.2f", rotacao.getZ())+">");
                kLimeAngleOffset = val;
              }
            }
          }
        }
      }
    }

    field.setRobotPose(getPose());
    
    publisher.set(new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    });
    publisher_target.set(new SwerveModuleState[] {
      frontLeft.targetState,
      frontRight.targetState,
      backLeft.targetState,
      backRight.targetState
    });
    
//TO.DO: Debugging and calibration data, remove for competition!
    /*SmartDashboard.putNumber("SwerveCalibration/FL", frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("SwerveCalibration/FR", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("SwerveCalibration/BL", backLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("SwerveCalibration/BR", backRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("SwerveRawMotor/FL", frontLeft.getDrivePosition());
    SmartDashboard.putNumber("SwerveRawMotor/FR", frontRight.getDrivePosition());
    SmartDashboard.putNumber("SwerveRawMotor/BL", backLeft.getDrivePosition());
    SmartDashboard.putNumber("SwerveRawMotor/BR", backRight.getDrivePosition());*/
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();//getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }
  
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void SlowerSpeed()
  {
    SpeedLimiter = 0.2;
    System.out.println("Slow speed");
  }
  public void NormalSpeed()
  {
    SpeedLimiter = 0.6;
    System.out.println("Normal speed");
  }
  public void MaxSpeed()
  {
    SpeedLimiter = 1;
    System.out.println("Max speed");
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxModuleSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Basic simulation of a swerve module, will just hold its current state and not use any hardware
   */
  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Basic simulation of a gyro, will just hold its current state and not use any hardware
   */
  class SimGyro {
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currentRotation;
    }

    public void updateRotation(double angularVelRps){
      currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
  }
}
