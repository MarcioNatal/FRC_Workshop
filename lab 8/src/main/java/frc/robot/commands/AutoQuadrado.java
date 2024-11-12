// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.List;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoQuadrado extends SequentialCommandGroup 
{
 public SwerveSubsystem drive = RobotContainer.swerveDrive;

  /** Creates a new AutoMundoSenai. */
  public AutoQuadrado( ) 
  {

    /******************************************************************************
    * The config contains information about special constraints, the max velocity,*
    * the max acceleration in addition to the start velocity and end velocity     *
    ******************************************************************************/

    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setReversed(true)
                .setKinematics(Constants.DriveConstants.kDriveKinematics);


    /**********************************************************************************
    * A trajectory is a smooth curve, with velocities and accelerations at each point *
    * along the curve, connecting two endpoints on the field.                         *
    * A spline refers to a set of curves that interpolate between points. Think of it *
    * as connecting dots, except with curves.                                         *
    ***********************************************************************************/
    // An example trajectory to follow.  All units in meters.

    /****************************
    * For clamped cubic splines *
    *****************************/

    // An example trajectory to follow.  All units in meters.
    Trajectory GoStraight =
        TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                //List.of(new Translation2d(-Units.feetToMeters(2), -.05)), 
                  List.of(new Translation2d(-0.5, -.05)), 
                // End 3 meters straight ahead of where we started, facing forward
                //new Pose2d(-Units.feetToMeters(4), 0, new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(-1.0, 0, new Rotation2d(Units.degreesToRadians(0))),
                config);

    Trajectory StrafeCurve =
        TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              //new Pose2d(-Units.feetToMeters(4), 0, new Rotation2d(Units.degreesToRadians(180))),
              new Pose2d(-1.0, -1.0, new Rotation2d(Units.degreesToRadians(0))),         
              // Pass through these two interior waypoints, making an 's' curve path
              //List.of(new Translation2d(-Units.feetToMeters(2), .05)), 
              List.of(new Translation2d(-0.5, -1.05)), 
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(0, -1.0, new Rotation2d(0)),
              config);
              
    // An example trajectory to follow.  All units in meters.
    Trajectory GoStraight2 =
        TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(-1.0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1.05, -.5)), 
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-1.0, -1, new Rotation2d(Units.degreesToRadians(0))),
                config);

    Trajectory StrafeCurve2 =
    TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-0.0, -1.0, new Rotation2d(Units.degreesToRadians(0))),

            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-0.05, -0.5)), 
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

    var thetaController =
    new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
  
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand goStraight =
    new SwerveControllerCommand(
        GoStraight,
        drive::getPose,
        Constants.DriveConstants.kDriveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    SwerveControllerCommand goStraight2 =
    new SwerveControllerCommand(
        GoStraight2,
        drive::getPose,
        Constants.DriveConstants.kDriveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    SwerveControllerCommand strafeCurve =
    new SwerveControllerCommand(
        StrafeCurve,
        drive::getPose,
        Constants.DriveConstants.kDriveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    SwerveControllerCommand strafeCurve2 =
    new SwerveControllerCommand(
        StrafeCurve2,
        drive::getPose,
        Constants.DriveConstants.kDriveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    

    

    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(


            new InstantCommand(() -> drive.resetOdometry(GoStraight.getInitialPose())),
            
            goStraight, 
           
            new WaitCommand(0.1),
            new InstantCommand(drive::stopModules),
            goStraight2,  
            
        
        
            new WaitCommand(0.1),
            new InstantCommand(drive::stopModules),
            strafeCurve,
            //new InstantCommand(() -> drive.resetOdometry(GoStraight2.getInitialPose())),
            
            
           new WaitCommand(0.1),
            new InstantCommand(drive::stopModules),
           
            strafeCurve2

    );
  }

}
