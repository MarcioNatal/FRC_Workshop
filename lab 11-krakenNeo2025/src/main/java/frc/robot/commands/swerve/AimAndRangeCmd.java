// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
// WPI imports
import edu.wpi.first.wpilibj2.command.Command;
// Robot imports
import frc.robot.RobotContainer;

import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Utils.SelectScore;
import frc.robot.Utils.TouchScreenInterface.VirtualButton;


import frc.robot.subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndRangeCmd extends Command 
{
  // Instantiate the object
  private final static SwerveSubsystem swerveDrive = RobotContainer.swerveDrive;
  private  Pose2d targetPosition = new Pose2d(0, 0, null);
    
  private static String limelightName = "limelight-front";

 

  // Variables
  //private final double kX, kY, kRot; // Replace with the correct class ID for the game piece
  private static boolean finish = false; 

  //Max speed variables
  private final double kMaxSpeed2Drive = 0.15;                   // Simple speed limit so we don't drive too fast
  private final double kMaxSpeed2Steer = 0.15; 
  private final double kMaxSpeed2Rot = 0.05; 

  //PID offset variables
  private final double kPsteer = 2.0;                    // how hard to turn toward the target
  private final double kPdrive = 2.0; //0.26 
  private final double kProt = 0.01; //0.26 
  //PID controller
  private PIDController pidDrive =  new PIDController(kPdrive, 0, 0);
  private PIDController pidSteer = new PIDController(kPsteer,0,0);
  private PIDController pidRot = new PIDController(kProt,0,0);

 
  
  

  //private final TrapezoidProfile.Constraints m_constraints =  new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  //private final ProfiledPIDController mController = new ProfiledPIDController(KP, KI, KD, m_constraints,kDt);

  // Declare the boolean variables   
  private boolean kA = false;
  private boolean kB = false;
  private boolean kC = false;
  private boolean kD = false;
  private boolean kE = false;
  private boolean kF = false;
  private boolean kG = false;
  private boolean kH = false;
  private boolean kI = false;
  private boolean kJ = false;
  private boolean kK = false;
  private boolean kL = false;


  //double kX, double kY , double kRot
  /** Creates a new AimAndRangeCmd. */
  public AimAndRangeCmd()
  {
    //AimingUtils.trackReefPosition(limelightName,Id1, Id2);

    // Set the class ID
    /*this.kX = kX;
    this.kY = kY;
    this.kRot = kRot;*/
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
      // Switch to pipeline 1
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    System.out.println("Aim and Range Command Started!!");

    
    // Set variables
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    kA = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kA);
    kB = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kB);  
    kC = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kC);
    kD = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kD);
    kE = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kE);
    kF = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kF);
    kG = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kG);
    kH = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kH);
    kI = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kI);
    kJ = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kJ);
    kK = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kK);
    kL = RobotContainer.touchScreenInterface.getButtonValue(VirtualButton.kL);


    
    // Get raw AprilTag/Fiducial data
    if (kA)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 1);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
        System.out.println("Position A selected !!");
    }
    else if (kB)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 2);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kC)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 3);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kD)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 4);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kE)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 5);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kF)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 6);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kG)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 7);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
        System.out.println("Position G selected !!");

    }
    else if (kH)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 8);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
        System.out.println("Position H selected !!");
    }
    else if (kI)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 9);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kJ)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 10);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kK)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 11);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    else if (kL)
    {
        //AimingUtils.trackReefPosition(limelightName, 0, 12);
        targetPosition =SelectScore.poseToScore(RobotContainer.touchScreenInterface);
    }
    
   // Check if a valid detection was found
   if (!finish) 
   {

    
    double steer_cmd = kMaxSpeed2Steer * pidSteer.calculate(robotPosition().getY(),targetPosition.getY());
    // try to drive forward until the target area reaches our desired area
    //double drive_cmd = MathUtil.clamp(pidDrive.calculate(distanceToTarget, kDistanceSetpoint),-kMaxSpeed2Drive,kMaxSpeed2Drive);
    double drive_cmd = kMaxSpeed2Drive *pidDrive.calculate(robotPosition().getX(),targetPosition.getX());

    double rot_cmd = kMaxSpeed2Rot *pidRot.calculate(robotPosition().getRotation().getDegrees(),targetPosition.getRotation().getDegrees());
    // Drive the swerve subsystem with the calculated speeds
    
    // try to drive forward until the target area reaches our desired area
    //double drive_cmd = MathUtil.clamp(pidDrive.calculate(distanceToTarget, kDistanceSetpoint),-kMaxSpeed2Drive,kMaxSpeed2Drive);
   
    // Drive the swerve subsystem with the calculated speeds
    swerveDrive.driveToTarget(() -> drive_cmd, () -> steer_cmd, () -> rot_cmd, () -> true);

    //System.out.println("steer cmd"+steer_cmd);
    //System.out.println("target Y"+targetPosition.getY());

    //System.out.println("drive cmd"+drive_cmd);
    //System.out.println("target x"+targetPosition.getX());

    //System.out.println("rot_cmd"+robotPosition().getRotation().getDegrees());
    //System.out.println("target rotation"+targetPosition.getRotation().getDegrees());

    //testar metodo
    /*swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(drive_cmd, 
                                                                      steer_cmd, 
                                                                      rot_cmd, 
                                                                      swerveDrive.getGyroRotation2d()));*/

    
  

        
    }
    
      
  }

  //Helper method to calculate robot position

  /**
   * Method to get the robot's current position
   * @return robotPosition
   */
  public Pose2d robotPosition()
  {

    // Get the current robot pose from the pose estimator
    Pose2d currentPose = swerveDrive.getPoseEstimator();

    // Get the robot's current position
    //Translation2d robotPosition = currentPose.getTranslation();
    //targetPosition = currentPose;
    return currentPose;
  }
 /* 
  // Helper method to calculate distance to a fiducial
  private double calculateDistance(RawFiducial fiducial) 
  {
      // Get the robot's current pose
      Pose2d currentPose = swerveDrive.getPoseEstimator();

      // Get the fiducial's position (assuming fiducial.x and fiducial.y are in meters)
      Translation2d fiducialPosition = new Translation2d(fiducial.txnc, fiducial.tync);

      // Calculate the distance between the robot and the fiducial
      double distance = currentPose.getTranslation().getDistance(fiducialPosition);

      return distance;
  }*/

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    swerveDrive.stopModules();
    System.out.println("Aim and Range Command Done!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // Optionally, add a condition to end the command
        // For example, you can check if the robot is close enough to the target
        //Pose2d distanceToTarget = AimingUtils.estimateDistanceToTargetFromPose(swerveDrive, targetPosition);
        
        return false;//finish || (distanceToTarget.getX() < 0.1 && distanceToTarget.getY() < 0.1 && distanceToTarget.getRotation().getDegrees() < 5); // Example threshold distance in meters
  }

}
