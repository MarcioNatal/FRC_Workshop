// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

public class DriveCommand extends Command 
{
  public DriveSubSystem drive = RobotContainer.drive;
  private double setPoint = 0.0;
  private double positionDegrees;
  //private double initialPosition;
  //private double tolerance = 0.05;//5%
  private boolean finish = false;
  private boolean kMode;
  
  
  /** Creates a new DriveCommand. */
  public DriveCommand(boolean mode) 
  {
    kMode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    finish=false;
    
    System.out.println("Drivecommand iniciado");
    
    setPoint = SmartDashboard.getNumber("Position Degrees", 0);
    //initialPosition=drive.getMotorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
    positionDegrees = drive.getMotorPosition();
    System.out.println("setPoint"+setPoint);
    System.out.println("Current Position"+positionDegrees);
    
    

    //if (setPoint* (1-tolerance) < positionDegrees && positionDegrees < setPoint * (1+tolerance)) finish=true;
    if ((positionDegrees)>=setPoint)
    {
      finish=true;
      System.out.println("Final Position"+positionDegrees);
    }
    else
    {
      drive.driveToPosition(setPoint, kMode);
      
    }

    
   
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
    //drive.stopMotor();
    drive.driveToPosition(setPoint, kMode);
   
    System.out.println("Drivecommand encerrado");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return finish;
  }
}
