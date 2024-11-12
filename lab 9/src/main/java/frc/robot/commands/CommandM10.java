// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemM10;

public class CommandM10 extends Command 
{
  public SubsystemM10 subM10 = RobotContainer.subM10;

  private final double KP=0.2000;
  private final double KI=0.0;
  private final double KD=0.0;
  private int count=0;

  //Trapezoidal constants
  private static double kDt = 0.02;
  private static double kMaxVelocity = 50;
  private static double kMaxAcceleration = 10;

  private static TrapezoidProfile.Constraints m_constrains = new TrapezoidProfile.Constraints (kMaxVelocity,kMaxAcceleration);
  private ProfiledPIDController mController = new ProfiledPIDController(KP, KI, KD, m_constrains,kDt);
  

  /** Creates a new DriveCommand. */
  public CommandM10() 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subM10);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Command M10 iniciado");
    subM10.resetEncoder();
    mController.reset(0,0);
    //finish = false;
    count = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    count++;
    if (count>1){

    }
        double output = mController.calculate(subM10.getPosition(),
                    (SmartDashboard.getNumber("Parametro Movimento/ SetPoint Voltas M10",0)));//setpoint da posicao

    subM10.driveMotor(output);

    /*
    if(mController.atSetpoint())
    {
      count++;
    }
    if (count>50)
    { 
      finish = true;
    }
    */
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    subM10.driveMotor(0.0);
    System.out.println("Command M10 encerrado");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return RobotContainer.m_driverController.getHID().getAButton();
  }
}
