// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

public class DriveNoPidCmd extends Command 
{
  public DriveSubSystem drive = RobotContainer.drive;
  private double setPoint;
  private double positionDegrees;
  //private double tolerance = 0.05;//5%
  private boolean finish = false;
  private boolean getInitialPosition;
  
  
  /** Creates a new DriveCommand. */
  public DriveNoPidCmd( ) 
  {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    //drive.resetEncoder();
    //getInitialPosition = true;
    finish = false;
    getInitialPosition=true;
    
    System.out.println("DriveNoPidCmd iniciado");
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
    if(getInitialPosition)
    {
      positionDegrees = drive.getMotorPosition();
      System.out.println("Posicao inicial" + positionDegrees);
      getInitialPosition=false;

    }

    double currentPosition = drive.getMotorPosition();
    System.out.println("Posicao atual" + currentPosition);

    
    setPoint = SmartDashboard.getNumber("Position Degrees", 0);
    System.out.println("setPoint"+setPoint);
    
   

    //if (setPoint* (1-tolerance) < positionDegrees && positionDegrees < setPoint * (1+tolerance)) finish=true;
    if ((currentPosition-positionDegrees)>setPoint)
    {
      finish=true;
     
    }
    else if ((currentPosition-positionDegrees)<setPoint)   
    {
      drive.runMotor(0.1);
      System.out.println("Drive Motor");
    } 
        
   
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
    drive.stopMotor();
    getInitialPosition=true;
   
    System.out.println("DriveNoPIDCmd encerrado");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    
    return finish;
  }
}
