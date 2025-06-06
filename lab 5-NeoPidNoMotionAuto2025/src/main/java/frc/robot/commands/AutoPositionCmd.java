// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPositionCmd extends SequentialCommandGroup 
{
  double position1Degrees;// = SmartDashboard.getNumber("Position1", 0.0);
  double position2Degrees;// = SmartDashboard.getNumber("Position2", 0.0);   
  double position3Degrees;// = 270;//SmartDashboard.getNumber("Position3", 0.0);
  private static DriveSubSystem drive = RobotContainer.drive;
 
  /** Creates a new AutoPositionCmd. */
  public AutoPositionCmd( ) 
  {
   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands
    (
      new InstantCommand(() -> position1Degrees = SmartDashboard.getNumber("Position1", 0.0)),
      new InstantCommand(() -> position2Degrees = SmartDashboard.getNumber("Position2", 0.0)),
      new InstantCommand(() -> position3Degrees = SmartDashboard.getNumber("Position3", 0.0)),
      
      new InstantCommand(drive::resetEncoder),
      
      new RunCommand (() -> drive.driveToPosition(position1Degrees, false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()>=position1Degrees),

      new WaitCommand(0.5),

      new RunCommand (() -> drive.driveToPosition(position2Degrees, false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()>=position2Degrees),

      new WaitCommand(1.0),

      new RunCommand (() -> drive.driveToPosition(position3Degrees, false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()>=position3Degrees),

      new WaitCommand(0.5),

      new RunCommand (() -> drive.driveToPosition(0.0, false), drive)//mode = true for Motion Control otherwise false for PID
                                  .until(()->drive.getMotorPosition()<=0.0)
      
      
    );
  }
}
