// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class DriveTeleop extends Command 
{
  private final SwerveSubsystem swerveDrive = RobotContainer.swerveDrive;

/**
 * Suppliers are especially useful in cases where you need to calculate values dynamically 
 * based on factors that can change over time.
 *The Supplier interface gives you a powerful mechanism to make your FRC robot code more 
 *flexible and adaptable.
 **/
  private Supplier<Double> kForward, kSideways, kRotate;

  private Supplier<Boolean> kFieldOriented;
  
  /** Creates a new DriveTeleop. */
  public DriveTeleop (Supplier<Double> xForwardFunction, 
                      Supplier<Double> ySidewaysFunction,
                      Supplier<Double> rotateFunction, 
                      Supplier<Boolean>  fieldOrientedFunction)
  {
    this.kForward = xForwardFunction;
    this.kSideways = ySidewaysFunction;
    this.kRotate = rotateFunction;
    this.kFieldOriented = fieldOrientedFunction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    swerveDrive.driveRobotOriented(kForward,kSideways,kRotate,kFieldOriented);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    swerveDrive.stopModules();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
