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
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
//REV imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;



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
  private final SparkMax turningMotor;
  SparkMaxConfig config = new SparkMaxConfig();


  /* drive motor control requests */
  //private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  //private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  
  // create a velocity closed-loop request, voltage output, slot 0 configs
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  //Declare motor encoders
  private final RelativeEncoder turningEncoder;
  
  
  

 
  //Turning PID
  private final SparkClosedLoopController turningPidController;


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
    this.turningMotor = new SparkMax(turningMotorId,MotorType.kBrushless);
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
  
    
    //1. Setup encoders and PID controllers for the turning SPARKS MAX.
    turningEncoder = turningMotor.getEncoder();  
    turningPidController = turningMotor.getClosedLoopController(); 
    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    /**
   * Tunning SparkMaxConfig
   * @see https://docs.revrobotics.com/brushless/revlib/revlib-overview/migrating-to-revlib-2025
   * 
   */
   canCoderConfigs();

   
    config
    .idleMode(ModuleConstants.kTurningMotorIdleMode)
    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    initializeDriveMotor();

    config.encoder
    .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
    .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    //.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    
    config.closedLoop
    // Set the PID gains for the turning motor. Note these are example gains, and you may need to tune them for your own robot!
    .pidf(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning, ModuleConstants.kTurningFF)
      //This is a velocity feed-forward. This is unique to each type of motor, 
      //and can be calculated by taking the reciprocal of the motor's velocity 
      //constant (Kv), in other words 1/Kv.


                            //WRAPING

    .positionWrappingEnabled(true)                                          // Enable PID wrap around for the turning motor. This will allow the PID     
    .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)   // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)   // to 10 degrees will go through 0 rather than the other direction which is a
    .outputRange(ModuleConstants.kTurningMinOutput,                                 // longer route.
    ModuleConstants.kTurningMaxOutput,ModuleConstants.pidSparkSlot)
    ;
    
    //Works like the old BurnFlash
    turningMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Metodos geters
    /*boolean isInverted = turningMotor.configAccessor.getInverted();
    double positionFactor = turningMotor.configAccessor.encoder.getPositionConversionFactor();
    double velocityFactor = turningMotor.configAccessor.encoder.getVelocityConversionFactor();*/



    

   
    //drivingPidControllerGains();MNL 01/28/2025

    //5. Initialize chassi offset value   

    m_chassisAngularOffset = chassisAngularOffset;
    
    
    //6.Load absolute CanCoder value to turning encoder
    resetToAbsolute();
    
   
    //7. Create desired angle  
    m_desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    //Testar
    //m_desiredState.angle = new Rotation2d(FeedbackSensor.kPrimaryEncoder.value);
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

    /**SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                                              new Rotation2d(turningEncoder.getPosition()));*/
    

  //01/23/2025 MNL precisa testar a correção
   correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));
    
  // set velocity to 8 rps, add 0.5 V to overcome gravity
  /*  
  double driveVelocityRPS = Conversions.MPSToRPS(optimizedDesiredState.speedMetersPerSecond, 
                                                ModuleConstants.kWheelCircumferenceMeters,
                                                ModuleConstants.kDriveMotorGearRatio);*/
  
  //
  double driveVelocityRPS = Conversions.MPSToRPS(correctedDesiredState.speedMetersPerSecond, 
                                                ModuleConstants.kWheelCircumferenceMeters,
                                                ModuleConstants.kDriveMotorGearRatio);

  driveMotor.setControl(m_request.withVelocity(driveVelocityRPS)); 
  turningPidController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition, ModuleConstants.pidSparkSlot);  
     
  //turningPidController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition, ModuleConstants.pidSparkSlot);
  
   m_desiredState = state;
  }

  
  

  /**https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
   * Config drive motors
   * @category TalonFx settings for drive motors
   */

  private void initializeDriveMotor()
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
    toConfigure.MotorOutput.NeutralMode = ModuleConstants.driveNeutralMode;
    /* Closed Loop Ramping */
    toConfigure.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ModuleConstants.kDriveClosedLoopRamp;
    //MNL 01/28/2025
    toConfigure.Slot0.kS = ModuleConstants.driveKS; //0.1; // Add 0.1 V output to overcome static friction
    toConfigure.Slot0.kV = ModuleConstants.driveKV; //0.12; // A velocity target of 1 rps results in 0.12 V output
    toConfigure.Slot0.kP = ModuleConstants.kPdriving; //0.11; // An error of 1 rps results in 0.11 V output
    toConfigure.Slot0.kI = ModuleConstants.kIdriving; //0; // no output for integrated error
    toConfigure.Slot0.kD = ModuleConstants.kDdriving; //0; // no output for error derivative
    
    //MNL 01/28/2025
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) 
    {
      status = driveMotor.getConfigurator().apply(toConfigure);
      if (status.isOK()) break;
    }
    if (!status.isOK()) 
    {
      System.out.println("Could not apply swerve module configs, error code: " + status.toString());
    }
    

    /* And initialize encoder position to 0 */
    driveMotor.setPosition(0);

    

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

  



  /**
   * Tunning TalonFx PIDController
   * @see https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
   * MNL 01/28/2025 comentado para testar
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
    return Rotation2d.fromRotations(absoluteCaNcoder.getAbsolutePosition().getValueAsDouble());
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

  public double getTurnCurrent()
  {
    return turningMotor.getOutputCurrent();
  }
  public double getDriveTemperature()
  {
    return driveMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getDriveVoltage()
  {
    return driveMotor.getMotorVoltage().getValueAsDouble();
  }
  
  /**
   * 
   */
  public void stopMotors() 
  {
      driveMotor.setControl(m_brake);
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
