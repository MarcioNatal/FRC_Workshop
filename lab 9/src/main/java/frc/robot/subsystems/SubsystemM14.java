// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemM14 extends SubsystemBase 
{
  private final CANSparkMax motoSparkMax ;
  private final RelativeEncoder mEncoder;
  //Constantes motor
  private final int kMotorID=14;
  private final IdleMode kIdleMode = IdleMode.kBrake;
  private final int kSparkMaxCurrent = 20;

  /** Creates a new DriveSubsystem. */
  public SubsystemM14() 
  {
    motoSparkMax = new CANSparkMax(kMotorID, MotorType.kBrushless);
    mEncoder = motoSparkMax.getEncoder();

    //motor settings
    motoSparkMax.restoreFactoryDefaults();
    motoSparkMax.setIdleMode(kIdleMode);
    motoSparkMax.setSmartCurrentLimit(kSparkMaxCurrent);
    motoSparkMax.burnFlash();

    SmartDashboard.putNumber("Parametro Movimento/ SetPoint Voltas M14",0);

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Corrente do Motor M14", this.getMotorCurrent());

    //retorna unidades de rotaçao 
    SmartDashboard.putNumber("Posiçao do encoder M14", mEncoder.getPosition());

    //mostra a velocidade em rpm
    SmartDashboard.putNumber("Velocidade do encoder M14", mEncoder.getVelocity());



  }


  /**
   * Método para acionar o motor
   * @param speed valor entre -1 e 1
   */
  public void driveMotor(double speed)
  {
    motoSparkMax.set(speed);

  }

  
  public void resetEncoder()
  {
    mEncoder.setPosition(0);
  }

  public double getPosition()
  {
    return mEncoder.getPosition();

  }

  
  
  /**
   * 
   * @return a corrente do motorSpark
   */
  public double getMotorCurrent()
  {
    return motoSparkMax.getOutputCurrent();
  }




}
