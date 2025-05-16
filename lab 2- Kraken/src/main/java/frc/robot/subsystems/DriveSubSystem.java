// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//CTRE IMPORTS
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

//WPILIB IMPORTS
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//ROBOT IMPORTS
import frc.robot.Constants.DriveConstants;

public class DriveSubSystem extends SubsystemBase 
{

  // Create a CANSparkMax object
  private final TalonFX motorKraken;
  
  /** Creates a new DriveSubSystem. */
  public DriveSubSystem() 
  {
    motorKraken = new TalonFX(DriveConstants.kMotorId, "rio");

    initializeMotor();
  }
  /**
   * @ motors initialization
   */
  public void initializeMotor()
  {
    TalonFXConfiguration globalConfig = new TalonFXConfiguration();
    motorKraken.clearStickyFaults();
    //Factory Default
    motorKraken.getConfigurator().apply(new TalonFXConfiguration());      
    
        
    /* Configure a stator limit of 20 amps */
   
    CurrentLimitsConfigs currentLimitConfigs = globalConfig.CurrentLimits;
    currentLimitConfigs.StatorCurrentLimit = DriveConstants.kCurrentThreshold;
    currentLimitConfigs.StatorCurrentLimitEnable = DriveConstants.kEnableCurrentLimit; 
    
    /* Gear Ratio Config */
    globalConfig.Feedback.SensorToMechanismRatio= 1;
    globalConfig.MotorOutput.NeutralMode = DriveConstants.kNeutralMode;
    //globalConfig.ClosedLoopGeneral.ContinuousWrap = false;
    /* Closed Loop Ramping */
    globalConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveConstants.kClosedLoopRamp;
    
    /* User can optionally change the configs or leave it alone to perform a factory default */
    globalConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motorKraken.getConfigurator().apply(globalConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
     
    //Enable safety
    //escaladorMotor.setSafetyEnabled(true);

    /* And initialize encoder position to 0 */
    motorKraken.getConfigurator().setPosition(0);
    
    

  }
  /**
   * Metodo para movimentar o motor
   * @param speed
   */
  public void  driveMotor(double speed)
  {
    motorKraken.set(speed);
  }

  public void  resetEncoder( )
  {
    motorKraken.setPosition(0);
  
  }
  
  public void  stopMotor( )
  {
    motorKraken.set(0);
  }

  

  public double getCurrent()
  {

    return motorKraken.getStatorCurrent().getValueAsDouble();
  }
  
   public double getVoltage()
  {

    return motorKraken.getMotorVoltage().getValueAsDouble();
  }
  /**
   * 
   * @return
   */
  public double getTemperature()
  {

    return motorKraken.getDeviceTemp().getValueAsDouble();
  }

  /**
   * 
   * @return valor do duty cycle
   */
  public double getDutyCycle()
  {

    return motorKraken.getDutyCycle().getValueAsDouble();
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
    SmartDashboard.putNumber("Encoder Position", motorKraken.getPosition().getValueAsDouble());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Encoder Velocity", motorKraken.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Motor Current", getCurrent());
    SmartDashboard.putNumber("Motor Voltage", getVoltage());
    SmartDashboard.putNumber("Motor Temperature", getTemperature());
    SmartDashboard.putNumber("Motor Duty", getDutyCycle());

  }


}
