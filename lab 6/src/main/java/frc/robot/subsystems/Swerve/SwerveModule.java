// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;




//CTRE imports
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
//REV imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


//WPI imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Local imports
import frc.robot.Constants.ModuleConstants;
import frc.robot.Utils.Conversions;


public class SwerveModule extends SubsystemBase 
{


//Declare motor Dirves
  private final TalonFX driveMotor;
  private final CANSparkMax turningMotor;

  /* drive motor control requests */
  //private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  //private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  
  // create a velocity closed-loop request, voltage output, slot 0 configs
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  //Declare motor encoders
  private final RelativeEncoder turningEncoder;
  
  

 
  //Turning PID
  private final SparkPIDController turningPidController;
  int pidSparkSlot = 0;

  //Declare absolute encoder
  public final CANcoder absoluteCaNcoder;



  //FeedForward 
  //private final SimpleMotorFeedforward driveFeedForward;

                                                                                   
  private SwerveModuleState m_desiredState;// = new SwerveModuleState(0.0, new Rotation2d());

  private Rotation2d angleOffset;
  //private boolean absoluteEncoderReversed=false;
  private double m_chassisAngularOffset = 0;

 

  //private static StatusSignal canCoderSignal = new StatusSignal<>(null, null);
  
  
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, 
   * drive encoder and turning encoder.
   * @param driveMotorId
   * @param turningMotorId
   * @param absoluteEncoderId
   * @param absoluteEncoderOffset
   * @param chassiAngularOffset
   */
  public SwerveModule(int driveMotorId, 
                      int turningMotorId, 
                      int absoluteEncoderId, 
                      Rotation2d absoluteEncoderOffset,
                      double chassisAngularOffset 
                      ) 
  {
    this.angleOffset = absoluteEncoderOffset;
    
    //this.absoluteEncoderReversed = absoluteEncoderReversed;
    
    /**
     * Instantiate objects
     */
    this.driveMotor = new TalonFX(driveMotorId);
    this.turningMotor = new CANSparkMax(turningMotorId,MotorType.kBrushless);
    this.absoluteCaNcoder = new CANcoder(absoluteEncoderId);
    
    

    /*this.driveFeedForward = new SimpleMotorFeedforward( Constants.ModuleConstants.driveKS, 
                                                                                      Constants.ModuleConstants.driveKV, 
                                                                                      Constants.ModuleConstants.driveKA);*/

    

    this.m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    
    /**
   * Config angle motors
   * https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications
   * @category turning motor SparkMax settings 
   */
  
    turningMotor.restoreFactoryDefaults();

    turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    
    //1. Setup encoders and PID controllers for the turning SPARKS MAX.
    turningEncoder = turningMotor.getEncoder();  
    turningPidController = turningMotor.getPIDController(); 
    turningPidController.setFeedbackDevice(turningEncoder); 
    
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);//radian
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);//radian/s
    

    /**
   * Tunning SparkMax PIDController
   * @see https://docs.revrobotics.com/brushless/revlib/closed-loop-control-overview/position-control-mode
   * 
   */
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningPidController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    
    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPidController.setP(ModuleConstants.kPTurning,pidSparkSlot);
    turningPidController.setI(ModuleConstants.kITurning,pidSparkSlot);
    turningPidController.setD(ModuleConstants.kDTurning,pidSparkSlot);
    turningPidController.setFF(ModuleConstants.kTurningFF,pidSparkSlot);//This is a velocity feed-forward. This is unique to each type of motor, 
                                                                        //and can be calculated by taking the reciprocal of the motor's velocity 
                                                                        //constant (Kv), in other words 1/Kv.
    turningPidController.setOutputRange(ModuleConstants.kTurningMinOutput,
                                        ModuleConstants.kTurningMaxOutput,pidSparkSlot);
    turningMotor.burnFlash();
    
    
    canCoderConfigs();
    
    configDriveMotor();
   
    drivingPidControllerGains();

    //5. Initialize chassi offset value   

    m_chassisAngularOffset = chassisAngularOffset;
    
    
    //6.Load absolute CanCoder value to turning encoder
    resetToAbsolute();
    
   
    //7. Create desired angle  
    m_desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    //resetToAbsolute();
    

   // 8.Reset drive encoder

    resetDriveEncoders();
    //resetWheel();
    

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

  public SwerveModulePosition getPosition() 
  {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
                                   this.getDrivePositionMeters(),
                                    new Rotation2d(turningEncoder.getPosition()-m_chassisAngularOffset)
                                    );
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

  public SwerveModuleState getState() 
  {
    return new SwerveModuleState(this.getDriveVelocityMPS(), new Rotation2d(this.getTurningPosition()-m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) 
  {
      
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
    correctedDesiredState.angle = state.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                                              new Rotation2d(turningEncoder.getPosition()));

  
      //It scales the velocity down using the robot's max speed
      // set velocity to 8 rps, add 0.5 V to overcome gravity
      
    double driveVelocityRPS = Conversions.MPSToRPS(optimizedDesiredState.speedMetersPerSecond, 
                                                  ModuleConstants.kWheelCircumferenceMeters,
                                                  ModuleConstants.kDriveMotorGearRatio);
    
    driveMotor.setControl(m_request.withVelocity(driveVelocityRPS)); 
    //driveMotor.setControl(m_request.withVelocity(1.0).withFeedForward(0.5)); 
    // driveMotor.setControl(m_request.withDu(1.0)); 
    

    //pid controller calculate its position
    turningPidController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkBase.ControlType.kPosition, pidSparkSlot);
    
    

    m_desiredState = state;
  }

  
  
  public void configTurningMotor()
  {

    
    

  }

  /**https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
   * Config drive motors
   * @category TalonFx settings for drive motors
   */

  private void configDriveMotor()
  { 
    driveMotor.clearStickyFaults();
    //Factory Default
    this.driveMotor.getConfigurator().apply(new TalonFXConfiguration());      
    //driveMotor.setInverted(driveMotorReversed);
    driveMotor.setNeutralMode(ModuleConstants.driveNeutralMode);
   
    
    /* Configure a stator limit of 20 amps */
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
    currentLimitConfigs.StatorCurrentLimit = ModuleConstants.driveCurrentThreshold;
    currentLimitConfigs.StatorCurrentLimitEnable = ModuleConstants.driveEnableCurrentLimit; 
    
    /*MNL 10/28/2024 precisa testar as 3 configs */
    /* Gear Ratio Config */
    toConfigure.Feedback.SensorToMechanismRatio=1/ModuleConstants.kDriveMotorGearRatio;
    toConfigure.ClosedLoopGeneral.ContinuousWrap = true;
    /* Closed Loop Ramping */
    toConfigure.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ModuleConstants.kDriveClosedLoopRamp;
    
    
        
    driveMotor.getConfigurator().apply(toConfigure);

    /* And initialize encoder position to 0 */
    driveMotor.getConfigurator().setPosition(0);

    

  }

  /**
   * @category CanCoder configutation
   */
  public void canCoderConfigs()
  {
    absoluteCaNcoder.clearStickyFaults();
     //Factory Default
    absoluteCaNcoder.getConfigurator().apply(new CANcoderConfiguration());    

    var configs = new CANcoderConfiguration();

    configs.withMagnetSensor(new MagnetSensorConfigs().
            withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
      
    /* Speed up signals to an appropriate rate */
    absoluteCaNcoder.getPosition().setUpdateFrequency(100);
    absoluteCaNcoder.getVelocity().setUpdateFrequency(100);
    
    /*MNL acrescentado 17/10/2024 */
    configs.MagnetSensor.withMagnetOffset(0);

    /* User can change the configs if they want, or leave it empty for factory-default */
    absoluteCaNcoder.getConfigurator().apply(configs);
    
  
 
  }

  

  public void turningPidControllerGains()
  {
    

  }

  /**
   * Tunning TalonFx PIDController
   * @see https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
   * 
   */
  public void drivingPidControllerGains()
  {
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    
    slot0Configs.kS = ModuleConstants.driveKS; //0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = ModuleConstants.driveKV; //0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = ModuleConstants.kPdriving; //0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = ModuleConstants.kIdriving; //0; // no output for integrated error
    slot0Configs.kD = ModuleConstants.kDdriving; //0; // no output for error derivative
    
    driveMotor.getConfigurator().apply(slot0Configs);
  }



  /**
   * 
   * @return 
   */
  public Rotation2d getCANcoder()
  {
    return Rotation2d.fromRotations(absoluteCaNcoder.getAbsolutePosition().getValue());
  }

  /**
   * Checks if cancoder getAbsolutePosition have an OK error code.
   * @return true if all is good otherwise false
   */
  public boolean getCanCoderIsValid()
  {
    return BaseStatusSignal.isAllGood(absoluteCaNcoder.getAbsolutePosition());
  }

  /*Initialize wheels positions */
  public void resetToAbsolute()
  {
   
    double absolutePosition = (2*Math.PI*(getCANcoder().getRotations() - angleOffset.getRotations()));
    
    turningEncoder.setPosition(absolutePosition);
  }
  
  /**
   * 
   * @return canCoder degrees
   */
  public double canCoderDegrees() 
  {
    return getRawHeading() * 360;
  }

  /**
   * 
   * @return canCoder degrees
   */
  public double canCoderRad() 
  {
    return getRawHeading() * 2 * Math.PI;
  }

  /**
   * Get the heading of the canCoder - will also include the offset
   *
   * @return Returns the raw heading of the canCoder (rotations)
   */
  public double getRawHeading() 
  {
      return absoluteCaNcoder.getAbsolutePosition().getValueAsDouble();
      
  }



  /**
   * Get the heading of the swerve module
   * @return Returns the heading of the module in radians as a double
   */
  public double getTurningHeading() 
  {
    //double heading = Units.degreesToRadians(canCoderDegrees() - absoluteOffsetEncoderDegrees());
    double heading = canCoderRad()-absoluteOffsetEncoderRadians();
    heading %= 2 * Math.PI;
    return heading;
  }

  /**
   * 
   * @return offset angle of the module in degrees
   */
  public double absoluteOffsetEncoderDegrees()
  {

    return angleOffset.getDegrees();
  }

  /**
   * 
   * @return offset angle of the module in radians
   */
  public double absoluteOffsetEncoderRadians()
  {

    return m_chassisAngularOffset;
  }

  /**
   * Get the heading of the swerve module
   * @return Returns the heading of the module in degrees as a double
   */
  public double getTurningHeadingDegrees() 
  {
    double heading = (canCoderDegrees() - absoluteOffsetEncoderDegrees()); //* (absoluteEncoderReversed ? -1.0: 1.0);
    heading %= 360;
    return heading;
  }

  

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    // Monitor absolute encoder health
        
  }


  /**
   * 
   * @return driveMotor position as double
   */
  public double getDrivePositionMeters() 
  {
      return Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(),ModuleConstants.kWheelCircumferenceMeters);
  }

  public double getTurningPosition() 
  {
      return turningEncoder.getPosition();
  }

  public double getDriveVelocityMPS() 
  {

    return Conversions.RPSToMPS(driveMotor.getRotorVelocity().getValueAsDouble(),ModuleConstants.kWheelCircumferenceMeters);
  }

  public double getTurningVelocity() {
      return turningEncoder.getVelocity();
  }

  public double getDriveCurrent()
  {
    return driveMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getDriveTemperature()
  {
    return driveMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getDriveVoltage()
  {
    return driveMotor.getMotorVoltage().getValueAsDouble();
  }
  
  
  public void stopMotors() 
  {
      driveMotor.set(0);
      turningMotor.set(0);
      
  }

  /**
     * Resets the drive encoder
     */
  public void resetDriveEncoders()
  {
    driveMotor.setPosition(0);
    
  }



}
