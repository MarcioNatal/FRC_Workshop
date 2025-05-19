// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Timer;
// WPI imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeaveCmd extends Command 
{
  // Instantiate the object
  private final static SwerveSubsystem swerveDrive = RobotContainer.swerveDrive;
 
  private static boolean getPose = false;  

  //Max speed variables
  
 
  
  //PID offset variables
  //private final double kPsteer = 0.26;                    // how hard to turn toward the target
  //private final double kPdrive = 2.0; //0.26 
  //private final double kProt = 0.01; //0.26 
  //PID controller
  

  private double kXposition=0;
  //private double kYposition=0;	
  //private double kRotPosition=0;
  private double currentXposition=0;
  
  private final double speed = 0.05; 

  private boolean finish = false;

  private final Timer timer = new Timer();


  //double kX, double kY , double kRot
  /** Creates a new AimAndRangeCmd. */
  public LeaveCmd()
  {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
      // Switch to pipeline 1
    
    System.out.println("Leave Command Started!!");
    timer.reset();
    timer.start();
    //pidDrive.setTolerance(0.02);

    
    // Set variables
    getPose = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    currentXposition = swerveDrive.getPoseEstimator().getX();
    if (getPose) 
    {
      kXposition = swerveDrive.getPoseEstimator().getX();
      //kYposition = swerveDrive.getPoseEstimator().getY();
      //kRotPosition = swerveDrive.getPoseEstimator().getRotation().getDegrees();
     
      getPose = false;
    }

    /*double steer_cmd = kMaxSpeed2Steer * pidSteer.calculate(swerveDrive.getPoseEstimator().getX(),kYposition);
    double drive_cmd = kMaxSpeed2Drive *pidDrive.calculate(swerveDrive.getPoseEstimator().getX(),(kXposition+0.2));
    double rot_cmd = kMaxSpeed2Rot *pidRot.calculate(swerveDrive.getPoseEstimator().getRotation().getDegrees(),kRotPosition);*/

    //swerveDrive.driveToTarget(() -> -drive_cmd, () ->0.0, () -> 0.0, () -> false);
       
    if (currentXposition > (kXposition + 0.1)||timer.hasElapsed(0.25))
    
    {
      System.out.println("Current Position: " + currentXposition + " Initial Position: " + kXposition);
      finish = true;
      System.out.println("Leave Command time is up!!");
    }
    else
    {

      swerveDrive.driveToTarget(() -> speed, () -> 0.0, () -> 0.0, () -> false);
      finish = false;
    }

    // Check if 0.5 seconds have passed or the robot has moved 0.1 meters
    /*if (timer.hasElapsed(1.0) || pidDrive.atSetpoint()) 
    {
      finish = true;
      System.out.println("Leave Command time is up!!");
    }*/
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    timer.stop();
    swerveDrive.stopModules();
    System.out.println("Leave Command Done!!");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // Optionally, add a condition to end the command
        // For example, you can check if the robot is close enough to the target
        
        
        return finish;
  }

}
