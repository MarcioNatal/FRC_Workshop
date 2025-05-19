// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



//Java Imports
import com.pathplanner.lib.auto.AutoBuilder;

//WPILIB Imports

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;

//Robot Imports
import frc.robot.Constants.OIConstants.JoystickDriverConstants;
import frc.robot.Constants.OIConstants.MesinhaJoy1Constants;
import frc.robot.Constants.OIConstants.MesinhaJoy2Constants;

import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Utils.OperatorHub;
import frc.robot.Utils.TouchScreenInterface;
import frc.robot.commands.swerve.AimAndRangeCmd;
import frc.robot.commands.swerve.LeaveCmd;
import frc.robot.commands.swerve.TrackAlgae;
import frc.robot.subsystems.LedControl;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  //Operator Controller
  public static TouchScreenInterface touchScreenInterface = new TouchScreenInterface();
  
  public static CommandXboxController driverJoystick = 
                                      new CommandXboxController(JoystickDriverConstants.kDriverControllerPort);
  
  public static Joystick mesinhaJoy1 = 
                          new Joystick(MesinhaJoy1Constants.kJoy1Port);
  public static Joystick mesinhaJoy2 = 
                          new Joystick(MesinhaJoy2Constants.kJoy2Port);

  public static final OperatorHub operatorHub = 
                                  new OperatorHub(mesinhaJoy1, mesinhaJoy2);
  /**
   * SUBSYSTEMS
   */
  //Put all subsystems here
  public static LimelightHelpers limelightHelpers = new LimelightHelpers();
  public static LedControl ledControl = new LedControl();
  
  //Swerve Drive
  public static SwerveSubsystem swerveDrive = new SwerveSubsystem();
  
  

  //Swerve commands
  public static TrackAlgae trackAlgaeCmd = new TrackAlgae(); 
  public static AimAndRangeCmd aimAndRangeCmd = new AimAndRangeCmd( );
  public static LeaveCmd leaveCmd = new LeaveCmd();
  
    
  /**
  * Declare class SendableChoser()  
  *  Pop up selection of option on the SmartDashBoard   
  */
  SendableChooser<Command> chooserAuto;
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
       
    // Add commands to the subsystem
    
    //Swerve Commands
    trackAlgaeCmd.addRequirements(swerveDrive);
    aimAndRangeCmd.addRequirements(swerveDrive);
    leaveCmd.addRequirements(swerveDrive);
    
    //algaSubsystem.setDefaultCommand(algaeCoralManual);


    // Build an auto chooser. This will use Commands.none() as the default option.
    chooserAuto = AutoBuilder.buildAutoChooser();
    
    
    //Instantiate subsystems
    //swerveDrive 
    // Add the path to the auto chooser
   
    
    // Instantiate Xbox Controller
    
    
     //Add to choices to smart dashboard
    SmartDashboard.putData("Autonomous", chooserAuto);

    //Default option
    /*chooserAuto.setDefaultOption("Autonomous 1", autoMundoSenai);
    chooserAuto.addOption("Autonomous 2", autoQuadrado);*/
   

    // Configure the button bindings
    configureBindings();

    //*******set default command to drive with joystick************/
     //The left stick controls translation of the robot.
     // Turning is controlled by the X axis of the right stick.
    
  
    /**
    * DEFAULT COMMANDS
    */
    //liftSubsystem.setDefaultCommand(liftCmd);

    //Runnable command
    swerveDrive.setDefaultCommand
                                  (new RunCommand
                                    (()-> swerveDrive.driveRobotOriented
                                      ( ()->driverJoystick.getLeftY(),
                                        ()->driverJoystick.getLeftX(),
                                        ()->-driverJoystick.getRightX(),
                                        ()->true
                                      ), 
                                      swerveDrive
                                    )//.onlyIf(()-> !driverJoystick.getHID().getXButton())
                          )
                                  ; 
    
    
    
                                                 
    
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
     /**
      * XBOX CONTROLLER BUTTONS
      */

     //Changes robot speed - speeding up
      driverJoystick.leftBumper().whileTrue(
                                            new InstantCommand(() -> {swerveDrive.robotSlower();
                                                                      ledControl.setWhiteLed();
                                                                      })); 
     driverJoystick.rightBumper().whileTrue(
                                            new InstantCommand(() -> {swerveDrive.robotFast();
                                                                      ledControl.setYellowLed();
                                                                      })); 

     driverJoystick.start().onTrue(new InstantCommand(swerveDrive::zeroHeading));


     driverJoystick.leftBumper().and(driverJoystick.rightBumper())  
                                .whileFalse(new InstantCommand(()->{swerveDrive.robotMaxSpeed();
                                                                    ledControl.setRedLed();
                                                                    })); 
     
    
    driverJoystick.povDown().whileTrue(leaveCmd);

    //Prepare to Pick Coral
     
   
     /**
      * OPERATOR CONTROLS
      */

    //JoystickButton leaveAlgaeReefHigh = new JoystickButton (mesinhaJoy2, operatorHub.kEleven);
    //leaveAlgaeReefHigh.whileTrue(freeAlgaeReefHighCmd);

    /*Trigger prepareClimb = new Trigger(()->(escaladorSubsystem.isDistanceValid() && !escaladorSubsystem.getSensorClimber()));
    prepareClimb.onTrue(new InstantCommand(ledControl::setGreenLed));
    prepareClimb.onTrue(new InstantCommand(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 0.75)));
    prepareClimb.onFalse(new InstantCommand(ledControl::setRedLed));
    prepareClimb.onFalse(new InstantCommand(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 0)));*/

   
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // An example command will be run in autonomous
    return chooserAuto.getSelected();
  }

}
