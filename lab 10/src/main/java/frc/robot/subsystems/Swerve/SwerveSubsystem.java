// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

//JAVA Imports
import java.util.function.Supplier;

//NavX imports
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

//WPI Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

//LOCAL Imports
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;


public class SwerveSubsystem extends SubsystemBase 

{
  /**************************
   * Create 4 Swerve Modules*
   **************************/
  public final SwerveModule frontLeftModule = 
  new SwerveModule
                  (
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetFLTurning,
                    DriveConstants.kFrontLeftChassisAngularOffset
                  );

  private final SwerveModule frontRightModule = 
  new SwerveModule(
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetFRTurning,
                    DriveConstants.kFrontRightChassisAngularOffset
                  );

  private final SwerveModule backLeftModule = 
  new SwerveModule(
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftTurningMotorPort,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetBLTurning,
                    DriveConstants.kBackLeftChassisAngularOffset
                  );

  private final SwerveModule backRightModule = 
  new SwerveModule(
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightTurningMotorPort,
                    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetBRTurning,
                    DriveConstants.kBackRightChassisAngularOffset
                  );

  
  
  /*******************
   * Create gyroscope*
   * ******************/
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  //public double startAngle; 
 
  private final SwerveDriveOdometry odometer;
               
 
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter= new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  ChassisSpeeds chassisSpeeds;

  SwerveModule[] modules = {frontLeftModule,frontRightModule,backLeftModule,backRightModule};

  /** Creates a new SwerveSubsystem. */
  private boolean throttleSlow=false;
  private boolean throttleFast=false;
  private boolean throttleMax=false;

  public SwerveSubsystem() 
  {
    
     odometer = new SwerveDriveOdometry(
                                        DriveConstants.kDriveKinematics,
                                        gyro.getRotation2d(), 
                                        new SwerveModulePosition[] 
                                        {
                                          frontLeftModule.getPosition(),
                                          frontRightModule.getPosition(),
                                          backLeftModule.getPosition(),
                                          backRightModule.getPosition()
                                        }
                                      );
    
    
    /* delay 1 s and then reset gyro */
    new Thread(() -> {
                      try 
                      {
                          Thread.sleep(1000); //avoid to block the rest of our code from running
                          zeroHeading();
                      } catch (Exception e) 
                      {
                      }
                    	}).start();

   
  
   AutoBuilder.configureHolonomic(
       this::getPose, // Robot pose supplier
       this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
       this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
       this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
       AutoConstants.pathAutoConfig,
        () -> {
          ///*
          var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Blue){
              return false;
            }else if(alliance.get() == DriverStation.Alliance.Red){
              return true;
            }
          }
          //*/
          return false;
        },
       this // Reference to this subsystem to set requirements
      );

  }

  public Rotation2d getGyroYaw() 
  {
    return Rotation2d.fromDegrees(gyro.getYaw());
    //return Rotation2d.fromDegrees(gyro.getYaw()).unaryMinus();
    //return gyro.getRotation2d();
     
  }

 /*
  public Rotation2d getYaw() 
  {
    
    double yaw = gyro.getYaw();
    return (Constants.DriveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
  }*/

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
        
    odometer.update(gyro.getRotation2d(), //(getGyroYaw()) , gyro.getYaw()
                    new SwerveModulePosition[] 
                    {
                      frontLeftModule.getPosition(),
                      frontRightModule.getPosition(),
                      backLeftModule.getPosition(),
                      backRightModule.getPosition()
                    }
                    );

  
    SmartDashboard.putNumber("Chassis /Pose X (meters)", this.getPose().getX());
    SmartDashboard.putNumber("Chassis /Pose Y (meters)", this.getPose().getY());
    SmartDashboard.putNumber("Chassis /Rotation (degrees)", this.getPose().getRotation().getDegrees());
    

    for (int i = 0; i < modules.length; i++) 
    {
      SmartDashboard.putNumber("Module " + i + " /Drive Motor Current (Amp)", modules[i].getDriveCurrent());
      SmartDashboard.putNumber("Module " + i + " /Drive Motor Temperature (℃)", modules[i].getDriveTemperature());
      SmartDashboard.putNumber("Module " + i + " /Turning Motor Heading (Degrees)", modules[i].getTurningHeadingDegrees());
      SmartDashboard.putNumber("Module " + i + " /CanCoder Heading (rad)", modules[i].canCoderRad());
      SmartDashboard.putNumber("Module " + i + " /CanCoder Heading (Degrees)", modules[i].canCoderDegrees());
      SmartDashboard.putNumber("Module " + i + " /CanCoder Angle value (rad)", modules[i].getTurningPosition());
      
  
       if (!modules[i].getCanCoderIsValid())
       {
        System.err.println("Absolute Encoder Error: Returning default angle.");

       }

    }
    

    // Output Module States to SmartDashboard
    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < moduleStates.length; i++) 
    {
      SmartDashboard.putNumber("Module " + i + " /Speed (m/s)", moduleStates[i].speedMetersPerSecond);
      SmartDashboard.putNumber("Module " + i + " /Angle (rad)", moduleStates[i].angle.getRadians());

      
    }


  }

  

  /** Zeroes the heading of the robot. */
  public void zeroHeading() 
  {
    gyro.reset();
    
    
    
    
  }   
  
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() 
  {
    frontLeftModule.resetDriveEncoders();
    frontRightModule.resetDriveEncoders();
    backLeftModule.resetDriveEncoders();
    backRightModule.resetDriveEncoders();
  }

  
  /**
   * Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) 
  {
    SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  //Method to get the ROBOT RELATIVE ChassisSpeeds
  public ChassisSpeeds getChassisSpeeds()
  {
    // Relative to robot
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose of the robot (x and y are in meters).
   */

  public Pose2d getPose() 
  {
    return odometer.getPoseMeters();
  }
  

   //reset the odomoter to a new location
   /**MNL10/23/2024
    * 
    * @param pose The pose  to set the odometry.
    */
   public void resetOdometry(Pose2d pose) 
   {

    odometer.resetPosition(
                  gyro.getRotation2d(), //arrow usa gyro.getAngle() ///getGyroYaw()
                  new SwerveModulePosition[] 
                                          {
                                              frontLeftModule.getPosition(),
                                              frontRightModule.getPosition(),
                                              backLeftModule.getPosition(),
                                              backRightModule.getPosition()
                                          },
                                          pose);
}

  /**
   * Get yaw value
   * @return The current Yaw values in degrees (-180 to 180)
   */
  public double getDoubleYaw()
  {
    return gyro.getYaw();
  }

  /**
   * 
   * @return the pose of the robot
   */
  public Rotation2d getRotation() 
  {
    return odometer.getPoseMeters().getRotation();
  }
 

  /**
   * Stop all modules
   */

  public void stopModules ()
  {
    
    for (int i = 0; i < modules.length; i++) 
    {
       modules[i].stopMotors();
     
    }
    

  }

  /**
   * 
   * @return throttle slow down the robot speed
   */
  public void robotSlower()
  {
    
    throttleSlow = true;
    throttleFast = false;
    throttleMax=false;
  }

  /**
   * 
   * @return throttle slow down the robot speed
   */
  public void robotFast()
  {
    throttleSlow = false;
    throttleFast = true;
    throttleMax=false;
  }

  /**
   * 
   * @return throttle slow down the robot speed
   */
  public void robotMaxSpeed()
  {
    
    throttleSlow=false;
    throttleFast=false;
    throttleMax=true;
  }
  

  // Improved "setX()" to use `setModuleStates()`
  public void setX() 
  {
    SwerveModuleState[] xStates = new SwerveModuleState[4];
    xStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    xStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    xStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    xStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    setModuleStates(xStates);
  }

   /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
   public void setModuleStates (SwerveModuleState [] desiredStates)
   { 

    //normalize the wheel speeds ;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond); 
    
    // Output Module States to each one
    for (int i = 0; i < modules.length; i++) 
    {
       modules[i].setDesiredState(desiredStates[i]);
     
    }
    

    // Output Module States to SmartDashboard
    
    /*for (int i = 0; i < desiredStates.length; i++) 
    {
      
      System.out.println("Module " + i + " Speed (m/s)"+ desiredStates[i].speedMetersPerSecond);
      System.out.println("Module " + i + " Angulo (rad)"+ desiredStates[i].angle);
      
    }*/
    
  }

  /**
   * 
   * @return states 
   */
  public SwerveModuleState[] getModuleStates() 
  {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) 
    {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Method drive with joystick
   * The use of these parameters as suppliers to dynamically provide speed and field orientation values.
   * Suppliers are especially useful in cases where you need to calculate values dynamically based on factors that can change over time.
   * The Supplier interface gives you a powerful mechanism to make your FRC robot code more flexible and adaptable. 
   * Suppliers are incredibly useful in command-based FRC programming because they allow you to:
   * Decouple Logic: You can separate the logic for calculating drive parameters (speeds, orientation) 
   * from the actual drive command. This makes your code cleaner and more maintainable.
   * Dynamic Values: You can easily update the speed and orientation values on-the-fly based on real-time conditions, 
   * such as sensor feedback or joystick input.
   * 
   * @param xSpdFunction Speed of the robot in the x direction (forward).
   * @param ySpdFunction Speed of the robot in the y direction (sideways).
   * @param turningSpdFunction Angular rate of the robot. rad/s
   * @param fieldOrientedFunction Boolean indicating if speeds are relative to the field or to therobot. 
   *                              
   **/

  public void driveRobotOriented( Supplier<Double> xSpdFunction, 
                                  Supplier<Double> ySpdFunction,
                                  Supplier<Double> turningSpdFunction, 
                                  Supplier<Boolean> fieldOrientedFunction)
  {
    
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();  
    double turningSpeed = turningSpdFunction.get();
    boolean fieldOriented = fieldOrientedFunction.get();
    double throttle = 1.0;
       
    
    //Selects speed
    if (throttleSlow)
    {
      
      throttle =0.2;
      System.out.println("Slow down velocity - 20%");
      
    }
    else if (throttleFast)
    {
      
      throttle =0.5;
      System.out.println("Speed up velocity - 60%");

    }
    else if (throttleMax) 
    { 
      throttle = 1.0;
      System.out.println("The Flash Mode ");
    }

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond*throttle;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond*throttle;
    turningSpeed = turningLimiter.calculate(turningSpeed)* DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond*throttle;

    // 4. Construct desired chassis speeds

    var swerveModuleStates =  DriveConstants.
                              kDriveKinematics.
                              toSwerveModuleStates
                              (fieldOriented ? 
                                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 
                                                                      ySpeed, 
                                                                      turningSpeed, 
                                                                      gyro.getRotation2d()) 
                                : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed)                               
                              );//Do this if fielOrientation is false 

    
    // 5. Convert chassis speeds to individual module states
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    // 6. Output each module states to wheels
    this.setModuleStates(swerveModuleStates);//*/  

    
    

  }

  

    







}
