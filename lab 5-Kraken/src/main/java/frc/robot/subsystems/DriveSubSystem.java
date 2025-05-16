// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//CTRE IMPORTS
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


//WPILIB IMPORTS
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//LOCAL IMPORTS
import frc.robot.Constants.*;

public class DriveSubSystem extends SubsystemBase 
{

  // Create a CANSparkMax object
  private final TalonFX motorKraken;
  private final TalonFXConfiguration globalConfig;// = new TalonFXConfiguration();
  
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake;// = new NeutralOut();

  //
  private final PositionVoltage m_positionVoltage;// = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage m_velocityVoltage;// = new VelocityVoltage(0).withSlot(1);


  //Create constants for PID coefficients
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  
  
  
  /** Creates a new DriveSubSystem. */
  public DriveSubSystem() 
  {

    //Instatiate objects
    motorKraken = new TalonFX(DriveConstants.kMotorId, "rio");
    globalConfig = new TalonFXConfiguration();
    m_brake = new NeutralOut();
    m_positionVoltage = new PositionVoltage(0).withSlot(0);
    m_velocityVoltage = new VelocityVoltage(0).withSlot(1);

    
    initializeMotor();

     
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
   * @ motors initialization
   */
  public void initializeMotor()
  {
    
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
    
    // Voltage-based VELOCITY requires a velocity feed forward to account for the back-emf of the motor 
    globalConfig.Slot0.kS = DriveConstants.KS; //0.1; // Add 0.1 V output to overcome static friction
    globalConfig.Slot0.kV = DriveConstants.KV; //0.12; // A velocity target of 1 rps results in 0.12 V output
    
    // PIDF coefficients
    globalConfig.Slot0.kP = DriveConstants.kP; //0.11; // An error of 1 rps results in 0.11 V output
    globalConfig.Slot0.kI = DriveConstants.kI; //0; // no output for integrated error
    globalConfig.Slot0.kD = DriveConstants.kD; //0; // no output for error derivative
   
    
    // Peak output of 10 volts
    //globalConfig.Voltage.withPeakForwardVoltage(Volts.of(12))

    
    
    
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
     * parameters.
     *  
     */
    
    
     motorKraken.setControl(m_velocityVoltage.withVelocity(mSpeed));
    
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
     * 
     */
    
    
     motorKraken.setControl(m_positionVoltage.withPosition(mRotation));
    
  }

  public void  resetEncoder( )
  {
    motorKraken.setPosition(0);
  
  }
  
  public void  stopMotor( )
  {
    motorKraken.setControl(m_brake);
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
    SmartDashboard.putNumber("ProcessVariable", motorKraken.getPosition().getValueAsDouble());
   

     // read PID coefficients from SmartDashboard
     double p = SmartDashboard.getNumber("PID/P Gain", 0);
     double i = SmartDashboard.getNumber("PID/I Gain", 0);
     double d = SmartDashboard.getNumber("PID/D Gain", 0);
     //double iz = SmartDashboard.getNumber("PID/I Zone", 0);
     //double ff = SmartDashboard.getNumber("PID/Feed Forward", 0);
     //double max = SmartDashboard.getNumber("PID/Max Output", 0);
     //double min = SmartDashboard.getNumber("PID/Min Output", 0);
    
 
     // if PID coefficients on SmartDashboard have changed, write new values to controller
     if((p != kP)) { globalConfig.Slot0.kP=p; kP = p; }
     if((i != kI)) { globalConfig.Slot0.kI=i; kI = i; }
     if((d != kD)) { globalConfig.Slot0.kD=d; kD = d; }
     
  }
}
