// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //old joystick setup
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoytick = new Joystick(OperatorConstants.kDriverControllerPort);

  
  //new joystick setup

  // The robot's subsystems
  //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // The driver's controller
  //XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  public static CommandXboxController XboxCommander = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  


  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    // Use event markers as triggers
    new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));


    //old setup:
    // Configure the trigger bindings
    /*new RunCommand(
            () ->swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OperatorConstants.kDriverYAxis),
      () -> -driverJoytick.getRawAxis(OperatorConstants.kDriverXAxis),
      () -> driverJoytick.getRawAxis(OperatorConstants.kDriverRotAxis),
      () -> !driverJoytick.getRawButton(OperatorConstants.kDriverFieldOrientedButtonIdx),
      () -> driverJoytick.getRawButton(OperatorConstants.kDriverStopPath)))
    );*/
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OperatorConstants.kDriverYAxis),
      () -> -driverJoytick.getRawAxis(OperatorConstants.kDriverXAxis),
      () -> -driverJoytick.getRawAxis(OperatorConstants.kDriverRotAxis),
      () -> !driverJoytick.getRawButton(OperatorConstants.kDriverFieldOrientedButtonIdx)));

    configureBindings();

    //New setup
    // Configure default commands
    /*swerveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
            swerveSubsystem.drive(
                    // Multiply by max speed to map the joystick unitless inputs to actual units.
                    // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                    // converting them to actual units.
                    m_driverController.getLeftY() * DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                    m_driverController.getLeftX() * DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                    m_driverController.getRightX()
                        * ModuleConstants.kTurningEncoderRPM2RadPerSec,
                    false),
                    swerveSubsystem));*/

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
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
  private void configureBindings() {
    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
    SmartDashboard.putData("Pathfind zero zero", AutoBuilder.pathfindToPose(
      //new Pose2d(8.3, 4.1, Rotation2d.fromDegrees(90)), 
      new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(90)),
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
      new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));

    //print buttom pressed
    // ...

    //m_driverController.getXButton().whenPressed(() -> System.out.println("X button pressed"));

    if (Constants.OperatorConstants.kZeroHeadingButtonIdx != 0){
      XboxCommander.button(Constants.OperatorConstants.kZeroHeadingButtonIdx).onTrue(new InstantCommand(() -> {
        swerveSubsystem.zeroHeading();
      }));
    }

    XboxCommander.button(Constants.OperatorConstants.kDriverSlow).onTrue(new InstantCommand(() -> swerveSubsystem.SlowerSpeed(),swerveSubsystem));
    XboxCommander.button(Constants.OperatorConstants.kDriverTurbo).onTrue(new InstantCommand(() -> swerveSubsystem.MaxSpeed(),swerveSubsystem));
    //If both buttons are false, run the normal speed command
    XboxCommander.button(Constants.OperatorConstants.kDriverSlow).or(XboxCommander.button(Constants.OperatorConstants.kDriverTurbo)).whileFalse(new InstantCommand(() -> swerveSubsystem.NormalSpeed(),swerveSubsystem));
    Trigger xButton = XboxCommander.x();

    xButton.whileTrue(AutoBuilder.pathfindToPose(
            new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(90)), 
            new PathConstraints(
              4.0, 4.0, 
              Units.degreesToRadians(360), Units.degreesToRadians(540)
            ), 
            0
        ));
    Translation2d corner1 = new Translation2d(8.0, 6.0);
    Translation2d corner2 = new Translation2d(9.4, 4.5);
    List<Pair<Translation2d, Translation2d>> list = List.of(new Pair<>(corner1, corner2));
    Pathfinding.setDynamicObstacles(list, swerveSubsystem.getPose().getTranslation());

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = swerveSubsystem.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
