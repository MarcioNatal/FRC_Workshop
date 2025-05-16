// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//revrobotics imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

//wpilib imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//local imports
import frc.robot.Constants;

public class DriveSubSystem extends SubsystemBase 
{

  private final SparkMax motorSpark; 
  private final RelativeEncoder motorEncoder;
  private SparkMaxConfig globalConfig = new SparkMaxConfig();
  
  /** Creates a new DriveSubSystem. */
  public DriveSubSystem() 
  {
    motorSpark = new SparkMax(Constants.motorSparkId,MotorType.kBrushless);

    motorEncoder = motorSpark.getEncoder();


    //motorSpark settings

    globalConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Configure the lift motor
     globalConfig
    .idleMode(Constants.kMotorIdleMode)
    .smartCurrentLimit(Constants.kMotorCurrentLimit)
    .inverted(false);
    //motorRelativeEncoder.setPositionConversionFactor(1);
    //motorRelativeEncoder.setVelocityConversionFactor(1);

    // Configure the lift encoder 
    globalConfig.encoder
    .velocityConversionFactor(1);

        // Configure the lift motor - like burn flash
    motorSpark.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);//
    
    
  }

  
  /**
   * 
   * @param speed
   */
  public void  driveMotor(double speed)
  {
    motorSpark.set(speed);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Encoder Position", motorEncoder.getPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Encoder Velocity", motorEncoder.getVelocity());
  }


}
