// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubSystem extends SubsystemBase 
{

  private final CANSparkMax motorSpark; 
 
  private final RelativeEncoder motorEncoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private final SparkPIDController m_pidController;
  int slot0;
  
  /** Creates a new DriveSubSystem. */
  public DriveSubSystem() 
  {
   motorSpark = new CANSparkMax(Constants.motorSparkId,MotorType.kBrushless);
  

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    motorSpark.restoreFactoryDefaults();

    // Encoder object created to display position values
    motorEncoder = motorSpark.getEncoder();

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = motorSpark.getPIDController();

    // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP,slot0);
    m_pidController.setI(kI,slot0);
    m_pidController.setD(kD,slot0);
    m_pidController.setIZone(kIz,slot0);
    m_pidController.setFF(kFF,slot0);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput,slot0);

    motorSpark.setIdleMode(Constants.kMotorIdleMode);
    motorSpark.setSmartCurrentLimit(Constants.kMotorCurrentLimit);
    motorEncoder.setPositionConversionFactor(Constants.motorSparkIdPositionFactor);
    motorEncoder.setVelocityConversionFactor(1);
    
    motorSpark.burnFlash();

    
  
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("PID/P Gain", kP);
    SmartDashboard.putNumber("PID/I Gain", kI);
    SmartDashboard.putNumber("PID/D Gain", kD);
    SmartDashboard.putNumber("PID/I Zone", kIz);
    SmartDashboard.putNumber("PID/Feed Forward", kFF);
    SmartDashboard.putNumber("PID/Max Output", kMaxOutput);
    SmartDashboard.putNumber("PID/Min Output", kMinOutput);
    
    SmartDashboard.putNumber("Motor/Position", 0);
    SmartDashboard.putNumber("Motor/Velocity", 0);




  }
  /**
   * Metodo para movimentar o motor
   * @param speed
   */
  public void  driveToVelocity(double mSpeed)
  {
     /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    
    double setPoint = mSpeed*maxRPM;
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    
  }

  /**
   * Metodo para movimentar o motor
   * @param position
   */
  public void  driveToPosition( double mRotation)
  {
     /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    
    
    m_pidController.setReference(mRotation, CANSparkMax.ControlType.kPosition);
    
  }

  public void  resetEncoder( )
  {
    motorEncoder.setPosition(0);
  
  }
  
  public void  stopMotor( )
  {
    motorSpark.set(0);
  
  }

   public double  getMotorVelocity( )
  {
    return motorEncoder.getVelocity();
  }

  public double  getMotorPosition( )
  {
    return motorEncoder.getPosition();
  }

  

  public double getCurrent()
  {

    return motorSpark.getOutputCurrent();
  }
  
   public double getVoltage()
  {

    return motorSpark.getBusVoltage();
  }
  /**
   * 
   * @return
   */
  public double getTemperature()
  {

    return motorSpark.getMotorTemperature();
  }

  /**
   * 
   * @return valor do duty cycle
   */
  public double getDutyCycle()
  {

    return motorSpark.getAppliedOutput();
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

    SmartDashboard.putNumber("Motor Current", getCurrent());
    SmartDashboard.putNumber("Motor Voltage", getVoltage());
    SmartDashboard.putNumber("Motor Temperature", getTemperature());
    SmartDashboard.putNumber("Motor Duty", getDutyCycle());
    SmartDashboard.putNumber("ProcessVariable", motorEncoder.getPosition());
   

     // read PID coefficients from SmartDashboard
     double p = SmartDashboard.getNumber("PID/P Gain", 0);
     double i = SmartDashboard.getNumber("PID/I Gain", 0);
     double d = SmartDashboard.getNumber("PID/D Gain", 0);
     double iz = SmartDashboard.getNumber("PID/I Zone", 0);
     double ff = SmartDashboard.getNumber("PID/Feed Forward", 0);
     double max = SmartDashboard.getNumber("PID/Max Output", 0);
     double min = SmartDashboard.getNumber("PID/Min Output", 0);
    
      
 
     // if PID coefficients on SmartDashboard have changed, write new values to controller
     if((p != kP)) { m_pidController.setP(p); kP = p; }
     if((i != kI)) { m_pidController.setI(i); kI = i; }
     if((d != kD)) { m_pidController.setD(d); kD = d; }
     if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
     if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
     if((max != kMaxOutput) || (min != kMinOutput)) { 
       m_pidController.setOutputRange(min, max); 
       kMinOutput = min; kMaxOutput = max; 
      }
  }
}
