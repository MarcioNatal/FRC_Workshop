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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


//WPILIB IMPORTS
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//LOCAL IMPORTS
import frc.robot.Constants.*;

public class DriveSubSystem extends SubsystemBase 
{

  //Create a new CANSparkMax object
  private final SparkMax motorSpark;//;//  = new SparkMax(DriveConstants.motorSparkId,MotorType.kBrushless);

  //Create a new SparkMaxConfig object
  private SparkMaxConfig globalConfig;// = new SparkMaxConfig();

  //Create a new SparkClosedLoopConfig object
  private final SparkClosedLoopController m_pidController;// = motorSpark.getClosedLoopController();
  //Create a new RelativeEncoder object
  private final RelativeEncoder motorEncoder;// = motorSpark.getEncoder();
  

  

  


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

     //Instantiate a new CANSparkMax object
    motorSpark  = new SparkMax(DriveConstants.motorSparkId,MotorType.kBrushless);

    //Instantiate a new SparkMaxConfig object
    globalConfig = new SparkMaxConfig();

    //Instantiate a new SparkClosedLoopConfig object
    m_pidController = motorSpark.getClosedLoopController();
    
    //Instantiate  a new RelativeEncoder object
    motorEncoder = motorSpark.getEncoder();

    

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

           
     initializeMotors();

          
      SmartDashboard.putNumber("Position Degrees", 0);
      




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
                .velocityConversionFactor(6) // Velocity in Degrees/sec = 360degrees/60sec = 6                                                     
                .positionConversionFactor(360.0/7.25);  //360/7.25 is the conversion factor for degrees to encoder units

    // Configure PID coefficients for slot 0
    globalConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0065)
                .i(kI)
                .d(kD)
                .velocityFF(kFF)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0,360);
    
    // Configure PID coefficients slot 1 for motion control
    globalConfig.closedLoop
                .p(0.0065, ClosedLoopSlot.kSlot1)
                .i(kI, ClosedLoopSlot.kSlot1)
                .d(kD,  ClosedLoopSlot.kSlot1)
                .velocityFF(kFF, ClosedLoopSlot.kSlot1)
                .maxOutput(kMaxOutput,ClosedLoopSlot.kSlot1)
                .minOutput(kMinOutput,ClosedLoopSlot.kSlot1);

    MAXMotionConfig maxMotionConfig = globalConfig.closedLoop.maxMotion;
    // Set MAXMotion parameters for position control. We need to pass
    // a closed loop slot to the maxMotion method to set the parameters for that slot.
    // In this case, we are setting the parameters for slot 0.
    // Set MAXMotion parameters for velocity control in slot 1
    maxMotionConfig
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal,ClosedLoopSlot.kSlot1)
                .maxAcceleration(34200,ClosedLoopSlot.kSlot1)
                .maxVelocity(34200,ClosedLoopSlot.kSlot1) //Degrees/sec = (MotorRPM/60) *360degrees
                                                                      //Degrees/sec = (5700/60) * 360 = 34200 degrees/sec
                .allowedClosedLoopError(0.85,ClosedLoopSlot.kSlot1);//degrees error

    

    // Configure  like burn flash
    motorSpark.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);//

    motorEncoder.setPosition(0);
    
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
   * @param position - position in degrees
   * @param mode choose between kPosition or kMAXMotionPositionControl
   */
  public void  driveToPosition( double position, boolean mode)
  {

    // Set the position setpoint for the PID controller 
    //if mode is true control type is kPosition (regular PID) in case 
    //mode is false control type is kMAXMotionPositionControl (MAXMotion PID)
    
    if(mode)
    {
      m_pidController.setReference(position,SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    
    }
    else
    {
      m_pidController.setReference(position,SparkMax.ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot1);
    
    }
      
      
    
  }


  /**
   * Method to reset the encoder position to zero
   */
  public void  resetEncoder( )
  {
    motorEncoder.setPosition(0);
  
  }

  /**
   * Method to run the motor in open loop mode
   * @param speed
   */
  public void  runMotor( double speed)
  {
    motorSpark.set(speed);
  
  }

  /**
   * Method to stop the motor in open loop mode
   */
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
