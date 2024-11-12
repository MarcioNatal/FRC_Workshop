// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemM10;
import frc.robot.subsystems.SubsystemM14;

public class Command2Motores extends Command 
{
  public SubsystemM10 subM10 = RobotContainer.subM10;
  public SubsystemM14 subM14 = RobotContainer.subM14;


  private final double KP=0.2000;
  private final double KI=0.0;
  private final double KD=0.0;
  private int count=0;

  //Trapezoidal constants
  private static double kDt = 0.02;
  private static double kMaxVelocity = 100; //rps
  private static double kMaxAcceleration = 20;

  private static TrapezoidProfile.Constraints m_constrains = new TrapezoidProfile.Constraints (kMaxVelocity,kMaxAcceleration);
  private ProfiledPIDController mControllerM10 = new ProfiledPIDController(KP, KI, KD, m_constrains,kDt);
  private ProfiledPIDController mControllerM14 = new ProfiledPIDController(KP, KI, KD, m_constrains,kDt);
  

  /** Creates a new DriveCommand. */
  public Command2Motores() 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subM10,subM14);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Command 2 motores iniciado");
    //subM10.resetEncoder();
    //subM14.resetEncoder();
    mControllerM10.reset(subM10.getPosition(),0);
    mControllerM14.reset(subM14.getPosition(),0);
    //finish = false;
    count = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    count++;
    if (count>1){
    double outputM10 = mControllerM10.calculate(subM10.getPosition(),
                    -100);//setpoint da posicao

    subM10.driveMotor(outputM10);

    double outputM14 = mControllerM14.calculate(subM14.getPosition(),
                    -100);//setpoint da posicao

    subM14.driveMotor(outputM14);
    

    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    subM10.driveMotor(0.0);
    System.out.println("Command 2 motores encerrado");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return RobotContainer.m_driverController.getHID().getAButton();
  }
}
