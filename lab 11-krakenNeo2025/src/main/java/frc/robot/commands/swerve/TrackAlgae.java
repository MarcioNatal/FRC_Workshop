// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;



import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.Limelight.AimingUtils;
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Limelight.LimelightHelpers.RawDetection;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackAlgae extends Command 
{

  // Instantiate the object
  public final SwerveSubsystem swerveDrive = RobotContainer.swerveDrive;
  //public final LimelightHelpers limelightHelpers = RobotContainer.limelightHelpers;
  //private  Translation2d targetPosition = new Translation2d(0, 0);
  
  // Limelight name
  private static String limelightName = "limelight-back";

  // Variables
  private final int expectedClassId = 0; // Replace with the correct class ID for the game piece
  public static boolean finish = false;
  public static boolean runMotor = false;

  //PID offset variables
  private final double kPsteer = 0.03;                    // how hard to turn toward the target
  private final double kPdrive = 2.0; //0.26 

  //Max speed variables
  private final double kMaxSpeed2Drive = 0.2;                   // Simple speed limit so we don't drive too fast
  private final double kMaxPeed2Steer = 0.2; 

  private final double kDistanceSetpoint = 0.5;
  // Variables
  
  private final double maxArea = 0.15;

  private  double ta = 0;


  //PID controller
  private PIDController pidDrive =  new PIDController(kPdrive, 0, 0);
  private PIDController pidSteer = new PIDController(kPsteer,0,0);
 
  

  /** Creates a new TrackAlgae. */
  public TrackAlgae() 
  {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    // Set pipeline
    LimelightHelpers.setPipelineIndex(limelightName, 1);

    // Set variables
    runMotor =false;
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //double distanceToTarget = AimingUtils.estimateDistanceToTarget(limelightName);
    
    //System.out.println ("Distance to target" + distanceToTarget);
    
    //SmartDashboard.putNumber("Distance to target", distanceToTarget);
    
    boolean alvoValido = LimelightHelpers.getTV(limelightName);

    RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
    

    // Iterate through all detections to find the best one
    for (RawDetection detection : detections) 
    {
      if(alvoValido){ System.out.println("Validou");}

      if(detection.classId==expectedClassId)
      { 
          System.out.println("Class ok");
      }

      if(detection.ta > 0.15){ System.out.println("ta.0.3");}
    
      if (detection.classId == expectedClassId && detection.ta > maxArea && alvoValido) 
        {
          System.out.println("Liga motor");
          ta = detection.ta;
          System.out.println("TA =  " + ta);

           runMotor=true;
        }

    
    }

    // Check if a valid detection was found
  
    if (runMotor) 
      {
        System.out.println("Run motor");
        System.out.println ("Print ta" + ta);
        SmartDashboard.putNumber("TA medidas",ta);
      
        
        
        // Get the angular velocity needed to aim at the target
          double angularVelocity = AimingUtils.aimAtTarget(limelightName);
          System.out.println("angular velocity" + angularVelocity);
          // Get the forward speed needed to range to the target
          //double forwardSpeed = AimingUtils.rangeToTarget(limelightName);
          System.out.println("forward speed ta" + ta );

          // Start with proportional steering
          //double steer_cmd = MathUtil.clamp(pidSteer.calculate(angularVelocity, 0),-kMaxPeed2Steer,kMaxPeed2Steer);

          double steer_cmd = kMaxPeed2Steer * pidSteer.calculate(angularVelocity, 0);
          // try to drive forward until the target area reaches our desired area
          //double drive_cmd = MathUtil.clamp(pidDrive.calculate(distanceToTarget, kDistanceSetpoint),-kMaxSpeed2Drive,kMaxSpeed2Drive);
          double drive_cmd = kMaxSpeed2Drive *pidDrive.calculate(ta, kDistanceSetpoint);
          // Drive the swerve subsystem with the calculated speeds
          swerveDrive.driveToTarget(() -> drive_cmd, () -> steer_cmd, () -> 0.0, () -> true);
        
      } 
    if (ta > 0.6)
      {
        
        //finish = true;
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //stop motors
    swerveDrive.stopModules();
      
    // Set pipeline to track apriltags
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    System.out.println("Track Algae Command Done!!");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    
    return finish; 
  }
}
