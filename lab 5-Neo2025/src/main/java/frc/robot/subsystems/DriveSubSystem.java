// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//REV IMPORTS
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


//WPILIB IMPORTS
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//LOCAL IMPORTS
import frc.robot.Constants.*;

public class DriveSubSystem extends SubsystemBase 
{

  //Create a new CANSparkMax object
  private final SparkMax motorSpark  = new SparkMax(DriveConstants.motorSparkId,MotorType.kBrushless);

  //Create a new SparkMaxConfig object
  private SparkMaxConfig globalConfig = new SparkMaxConfig();

  //Create a new SparkClosedLoopConfig object
  private final SparkClosedLoopController m_pidController = motorSpark.getClosedLoopController();
  //Create a new RelativeEncoder object
  private final RelativeEncoder motorEncoder = motorSpark.getEncoder();
  

  

  


  //Create constants for PID coefficients
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  
  
  
  /** Creates a new DriveSubSystem. */
  public DriveSubSystem() 
  {

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("PID/P Gain", kP);
    SmartDashboard.putNumber("PID/I Gain", kI);
    SmartDashboard.putNumber("PID/D Gain", kD);
    SmartDashboard.putNumber("PID/I Zone", kIz);
    SmartDashboard.putNumber("PID/Feed Forward", kFF);
    SmartDashboard.putNumber("PID/Max Output", kMaxOutput);
    SmartDashboard.putNumber("PID/Min Output", kMinOutput);

    //Instantiate the CANSparkMax object
    //motorSpark = new SparkMax(DriveConstants.motorSparkId,MotorType.kBrushless);
    
    //Instantiate the SparkClosedLoopConfig object
    //m_pidController = motorSpark.getClosedLoopController();

      /**
       * The RestoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
    

      // Encoder object created to display position values
     // motorEncoder = motorSpark.getEncoder();
      

      /**
       * In order to use PID functionality for a controller, a SparkPIDController object
       * is constructed by calling the getPIDController() method on an existing
       * CANSparkMax object
       */
      

      // PID coefficients
      kP =0.01;
      kI = 0.0;
      kD = 0.0; 
      kIz = 0.0; 
      kFF = 0.0000; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;

      // Configure PID controller
     //pIdLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
     
     initializeMotors();

      // Configure the PID controller with the global configuration
      
     // Set the PID gains for the turning motor. Note these are example gains, and you may need to tune them for your own robot!
     // slot, as it will default to slot 0.
    /*  pIdLoopConfig.pidf(kP, 
                        kI, 
                        kD, 
                        kFF,ClosedLoopSlot.kSlot0) ;*/

      //Instantiate the SparkClosedLoopConfig object
      // Configure PID controller
          
     
      

      // Set the PID gains for the turning motor. Note these are example gains, and you may need to tune them for your own robot!
     // slot, as it will default to slot 0.
     
          
      
      //Instantiate the SparkClosedLoopConfig object
      
      
      
      SmartDashboard.putNumber("Position Degrees", 0);
      //SmartDashboard.putNumber("Motor/Velocity", 0);




  }

  /**
   * Metodo para inicializar o motor
   */
  public void initializeMotors()
  {
 
    // Configure motor
     globalConfig
                .idleMode(DriveConstants.kMotorIdleMode) 
                .smartCurrentLimit(DriveConstants.kMotorCurrentLimit)
                .inverted(false);

    // Configure encoder 
    globalConfig.encoder
                .positionConversionFactor(360/7.25);

    globalConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(kP)
                .i(kI)
                .d(kD)
                .velocityFF(kFF)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0,360);

    

    

    // Configure  like burn flash
    motorSpark.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);//

  
    
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
     *  com.revrobotics.SparkMax.ControlType.kDutyCycle
     *  com.revrobotics.SparkMax.ControlType.kPosition
     *  com.revrobotics.SparkMax.ControlType.kVelocity
     *  com.revrobotics.SparkMax.ControlType.kVoltage
     */
    
    double setPoint = mSpeed*maxRPM;
    m_pidController.setReference(setPoint, SparkMax.ControlType.kVelocity);
    
  }

  /**
   * Metodo para movimentar o motor
   * @param position
   */
  public void  driveToPosition( double position)
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
     *  com.revrobotics.SparkMax.ControlType.kDutyCycle
     *  com.revrobotics.SparkMax.ControlType.kPosition
     *  com.revrobotics.SparkMax.ControlType.kVelocity
     *  com.revrobotics.SparkMax.ControlType.kVoltage
     */
    System.out.println("position setpoint"+position);
    
    m_pidController.setReference(position, SparkMax.ControlType.kPosition);
    
  }

  public void  resetEncoder( )
  {
    motorEncoder.setPosition(0);
  
  }
  
  public void  stopMotor( )
  {
    motorSpark.set(0);
  
  }

  public void  stopPidMotor( )
  {
    m_pidController.setReference(0.0, SparkMax.ControlType.kVelocity);
  
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
    SmartDashboard.putNumber("Encoder Position", this.getMotorPosition());

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    //SmartDashboard.putNumber("Encoder Velocity", motorEncoder.getVelocity());
    SmartDashboard.putNumber("Motor Current", getCurrent());
    SmartDashboard.putNumber("Motor Voltage", getVoltage());
    SmartDashboard.putNumber("Motor Temperature", getTemperature());
    SmartDashboard.putNumber("Motor Duty", getDutyCycle());
    //SmartDashboard.putNumber("ProcessVariable", motorEncoder.getPosition());
   

     // read PID coefficients from SmartDashboard
     double p = SmartDashboard.getNumber("PID/P Gain", 0);
     double i = SmartDashboard.getNumber("PID/I Gain", 0);
     double d = SmartDashboard.getNumber("PID/D Gain", 0);
     double iz = SmartDashboard.getNumber("PID/I Zone", 0);
     double ff = SmartDashboard.getNumber("PID/Feed Forward", 0);
     double max = SmartDashboard.getNumber("PID/Max Output", 0);
     double min = SmartDashboard.getNumber("PID/Min Output", 0);
     
  
    
 
     // if PID coefficients on SmartDashboard have changed, write new values to controller
     if((p != kP)) { globalConfig.closedLoop.p(p); kP = p;}//
     if((i != kI)) { globalConfig.closedLoop.i(i); kI = i; }
     if((d != kD)) { globalConfig.closedLoop.d(d); kD = d; }
     if((iz != kIz)) { globalConfig.closedLoop.iZone(iz); kIz = iz; }
     if((ff != kFF)) { globalConfig.closedLoop.velocityFF(ff); kFF = ff; }
     if((max != kMaxOutput) || (min != kMinOutput)) { globalConfig.closedLoop.outputRange(min, max);
                                                      kMinOutput = min; kMaxOutput = max;}

    
  }
}
