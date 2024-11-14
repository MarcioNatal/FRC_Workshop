package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    //private final CANSparkMax driveMotor;
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder turningEncoder;

    public SwerveModuleState targetState;

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.targetState = new SwerveModuleState();
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);//new AnalogInput(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        driveMotor.clearStickyFaults();
        
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.setNeutralMode(NeutralModeValue.Brake);//brake mode


        TalonFXConfiguration TalonSetup = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitConfigs = TalonSetup.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimit = ModuleConstants.driveCurrentThreshold;
        currentLimitConfigs.StatorCurrentLimitEnable = ModuleConstants.driveEnableCurrentLimit; 
        //kDriveEncoderRot2Meter = 1 count = 1 meter
        //kDriveMotorGearRatio = 1 count = revolution
        TalonSetup.Feedback.SensorToMechanismRatio=1/ModuleConstants.kDriveEncoderRot2Meter;//ModuleConstants.kDriveMotorGearRatio;
        TalonSetup.ClosedLoopGeneral.ContinuousWrap = true;
        TalonSetup.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ModuleConstants.kDriveClosedLoopRamp;
        driveMotor.getConfigurator().apply(TalonSetup);
        driveMotor.getConfigurator().setPosition(0);

        turningMotor = new CANSparkMax(turningMotorId, CANSparkLowLevel.MotorType.kBrushless);

        turningMotor.setInverted(turningMotorReversed);

        turningEncoder = turningMotor.getEncoder();

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();//driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();//driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue();//.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);//driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition() {
      //SwerveModulePosition val = new SwerveModulePosition();
      return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setTargetState(SwerveModuleState targetState) {
        if (Math.abs(targetState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        targetState = SwerveModuleState.optimize(targetState, getState().angle);
        driveMotor.set(targetState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //driveMotor.
        turningMotor.set(turningPidController.calculate(getTurningPosition(), targetState.angle.getRadians()));
        this.targetState = targetState;
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", targetState.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
