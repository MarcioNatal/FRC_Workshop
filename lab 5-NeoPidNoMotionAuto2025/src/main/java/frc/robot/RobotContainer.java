// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPositionCmd;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveNoPidCmd;
import frc.robot.subsystems.DriveSubSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  public static DriveSubSystem drive = new DriveSubSystem();
  public static DriveCommand driveCommand = new DriveCommand(true);
  public static DriveNoPidCmd driveNoPidCommand = new DriveNoPidCmd();

  public static AutoPositionCmd autoPositionCmd = new AutoPositionCmd();

  // Replace with CommandPS4Controller or CommandJoystick if needed
 public static CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
  //
   //drive.setDefaultCommand(driveCommand);
 
    
    
                            // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
   

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    m_driverController.start().whileTrue(new InstantCommand (drive::resetEncoder));
    
    //Motor Position open loop control
    m_driverController.a().whileTrue(driveNoPidCommand);
    
    //Motor Position control using pid controller
    m_driverController.b().whileTrue(new DriveCommand(true));

    //Motor Position control using Motion controller
    m_driverController.x().whileTrue(new DriveCommand(false));

    m_driverController.y().onTrue(autoPositionCmd);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    
    // An example command will be run in autonomous
    return new SequentialCommandGroup
    (
      new InstantCommand(() -> SmartDashboard.getNumber("Position1", 0.0)),
      new InstantCommand(() -> SmartDashboard.getNumber("Position2", 0.0)),
      new InstantCommand(() -> SmartDashboard.getNumber("Position3", 0.0)),
      
      new InstantCommand(drive::resetEncoder),
      
      new RunCommand (() -> drive.driveToPosition(SmartDashboard.getNumber("Position1", 0.0), false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()>=SmartDashboard.getNumber("Position1", 0.0)),

      new WaitCommand(0.5),

      new RunCommand (() -> drive.driveToPosition(SmartDashboard.getNumber("Position2", 0.0), false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()>=SmartDashboard.getNumber("Position2", 0.0)),

      new WaitCommand(1.0),

      new RunCommand (() -> drive.driveToPosition(SmartDashboard.getNumber("Position3", 0.0), false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()>=SmartDashboard.getNumber("Position3", 0.0)),

      new WaitCommand(0.5),

      new RunCommand (() -> drive.driveToPosition(0.0, false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()<=0.0)
    );
  }
}
