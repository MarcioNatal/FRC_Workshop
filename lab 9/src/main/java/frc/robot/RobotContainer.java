// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Command2Motores;
import frc.robot.commands.CommandM10;
import frc.robot.commands.CommandM14;
import frc.robot.subsystems.SubsystemM10;
import frc.robot.subsystems.SubsystemM14;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  public static SubsystemM10 subM10 = new SubsystemM10();
  public static SubsystemM14 subM14 = new SubsystemM14();
  public static CommandM10 commandM10 = new CommandM10();
  public static CommandM14 commandM14 = new CommandM14();
  public static Command2Motores command2Motores = new Command2Motores();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    subM10.setDefaultCommand(new RunCommand(()->subM10.driveMotor(m_driverController.getLeftX()),subM10));
    subM14.setDefaultCommand(new RunCommand(()->subM14.driveMotor(m_driverController.getLeftY()),subM14));
    
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().onTrue(commandM10);
    m_driverController.y().onTrue(commandM14);
    m_driverController.x().onTrue(command2Motores);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An example command will be run in autonomous
    return null;
  }
}
