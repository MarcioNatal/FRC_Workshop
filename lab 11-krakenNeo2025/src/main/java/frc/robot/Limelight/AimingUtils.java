
package frc.robot.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AimingUtils 
{

    // Constants for aiming and ranging to the target
    private static final double kP = 1.0; // Proportional control constant
    private static final double kMaxAngularSpeed = 1.0; // Maximum angular speed in radians per second
    private static final double kMaxSpeed = 1.0; // Maximum forward speed in meters per second

    
    /**
     * Method to aim at the target using Limelight's tx value
     * @param limelightName
     * @return targetingAngularVelocity
     */
    public static double aimAtTarget(String limelightName) 
    {
        // Get the horizontal offset from crosshair to target (tx)
        double targetingAngularVelocity = LimelightHelpers.getTX(limelightName) * kP;

        // Convert to radians per second for our drive method
        targetingAngularVelocity *= kMaxAngularSpeed;

        // Invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= 1.0;

        return targetingAngularVelocity;
    }

   
    /**
     * Method to range to the target using Limelight's ty value
     * @param limelightName
     * @return targetingForwardSpeed
     */
    public static double rangeToTarget(String limelightName) 
    {
        // Get the vertical offset from crosshair to target (ty)
        double targetingForwardSpeed = LimelightHelpers.getTY(limelightName) * kP;

        // Convert to meters per second for our drive method
        targetingForwardSpeed *= kMaxSpeed;

        // Invert since ty is positive when the target is above the crosshair
        targetingForwardSpeed *= 1.0;

        return targetingForwardSpeed;
    }

    
    /**
     * Method to estimate the distance to the target using Limelight's ty value
     * @param limeligthName
     * @return distanceToTarget
     */
    public static double estimateDistanceToTarget(String limeligthName) 
    {
        // Constants for distance calculation
        double targetHeight = 0.15; // Height of the target in meters
        double limelightHeight = 0.1; // Height of the Limelight in meters
        double limelightAngle = 0.0; // Mounting angle of the Limelight in degrees

        // Get the vertical offset from crosshair to target (ty)
        double ty = LimelightHelpers.getTY(limeligthName);

        // Calculate the angle to the target in radians
        double angleToTarget = Math.toRadians(limelightAngle + ty);

        // Calculate the distance to the target using trigonometry
        double distanceToTarget = (targetHeight - limelightHeight) / Math.tan(angleToTarget);

        return distanceToTarget;
    }

    
    /**
     * Method to estimate the distance to the target from the robot's pose
     * @param swerveSubsystem
     * @param targetPosition
     * @return
     */
    public static Pose2d estimateDistanceToTargetFromPose(SwerveSubsystem swerveSubsystem, Pose2d targetPosition) 
    {
        // Get the current robot pose from the pose estimator
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();

        // Get the robot's current position

        // Calculate the distance to the target
        double distanceToX = Math.abs(targetPosition.getX() - currentPose.getX());
        double distanceToY = Math.abs(targetPosition.getY() - currentPose.getY());
        double distanceToRot = Math.abs(targetPosition.getRotation().getDegrees() - currentPose.getRotation().getDegrees());

        return new Pose2d(distanceToX,distanceToY,Rotation2d.fromDegrees(distanceToRot));
        //return distanceToX, distanceToY, distanceToRot;
    }

    /**
     * Method to track the Coral Station using Limelight's AprilTag detection
     * @param limelightName
     */
    public static void  trackCoralStation(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{1, 2, 12, 13}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range

    }

    
    /**
     * // Method to track the processor using Limelight's AprilTag detection
     * @param limelightName
     * 
     */
    public static void  trackProcessor(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{3, 16}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range

    }

      /**
     * Method to track the Reef Position using Limelight's AprilTag detection
     * @param limelightName
     * @param Id1
     * @param Id2
     */
    public static void  trackReefPosition(String limelightName, int Id1, int Id2)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{Id1, Id2}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range

    }

    /**
     * Method to track the Reef A using Limelight's AprilTag detection
     * @param limelightName
     */
    public static void  trackReefAB(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{7, 18}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range

    }
    
   /**
    * Method to track the Reef C using Limelight's AprilTag detection
    * @param limelightName
    */
    public static void trackReefCD(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{8, 17}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range

    }
    
    /**
     * Method to track the Reef E using Limelight's AprilTag detection
     * @param limelightName
     */
    public static void  trackReefEF(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{9, 22}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range

    }

    /**
     * Method to track the Reef G using Limelight's AprilTag detection
     * @param limelightName
     */
    public static void  trackReefGH(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{10, 21}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range
                

    }

   /**
    * Method to track the Reef IJ using Limelight's AprilTag detection
    * @param limelightName
    */
    public static void  trackReefIJ(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{11, 20}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range
                

    }
    
    /**
     * Method to track the Reef KL using Limelight's AprilTag detection
     * @param limelightName
     */
    public static void  trackReefKL(String limelightName)
    {
        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{6, 19}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range
                

    }
    



}