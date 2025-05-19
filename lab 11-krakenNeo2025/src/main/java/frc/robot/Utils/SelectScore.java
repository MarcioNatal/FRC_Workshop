// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utils.TouchScreenInterface.VirtualButton;

/** Add your docs here. */
public class SelectScore 
{

    public static Pose2d poseToScore(TouchScreenInterface touch){
        Pose2d poseToScore;
        var alliance2 = DriverStation.getAlliance();
        if (alliance2.get() == DriverStation.Alliance.Red) 
        {
            // Red Alliance Values
            if(touch.getButtonValue(VirtualButton.kA)){
                poseToScore = new Pose2d(14.47,3.87,Rotation2d.fromDegrees(180));
            }
            else if (touch.getButtonValue(VirtualButton.kB)){
                poseToScore = new Pose2d(14.47,4.190,Rotation2d.fromDegrees(180));
            }
            else if (touch.getButtonValue(VirtualButton.kC)){
                poseToScore = new Pose2d(13.89,5.18,Rotation2d.fromDegrees(-120));
            }
            else if (touch.getButtonValue(VirtualButton.kD)){
                poseToScore = new Pose2d(13.65,5.31,Rotation2d.fromDegrees(-120));
            }
            else if (touch.getButtonValue(VirtualButton.kE)){
                poseToScore = new Pose2d(12.50,5.35,Rotation2d.fromDegrees(-60));
            }
            else if (touch.getButtonValue(VirtualButton.kF)){
                poseToScore = new Pose2d(12.11,5.43,Rotation2d.fromDegrees(-60));
            }
            else if (touch.getButtonValue(VirtualButton.kG)){
                poseToScore = new Pose2d(11.62,4.190,Rotation2d.fromDegrees(0));
            }
            else if (touch.getButtonValue(VirtualButton.kH)){
                poseToScore = new Pose2d(11.62,3.861,Rotation2d.fromDegrees(0));
            }//testar com um metro a mais no x
            else if (touch.getButtonValue(VirtualButton.kI)){
                poseToScore = new Pose2d(12.22,2.86,Rotation2d.fromDegrees(60));
            }
            else if (touch.getButtonValue(VirtualButton.kJ)){
                poseToScore = new Pose2d(12.50,2.70,Rotation2d.fromDegrees(60));
            }
            else if (touch.getButtonValue(VirtualButton.kK)){
                poseToScore = new Pose2d(13.60,2.71,Rotation2d.fromDegrees(120));
            }
            else if (touch.getButtonValue(VirtualButton.kL)){
                poseToScore = new Pose2d(13.88,2.87,Rotation2d.fromDegrees(120));
            }
            else{
                poseToScore = new Pose2d();
            }
        }else{
            // Blue Alliance Values

            if(touch.getButtonValue(VirtualButton.kA)){
                poseToScore = new Pose2d(3.05,4.18,Rotation2d.fromDegrees(0));
            }
            else if (touch.getButtonValue(VirtualButton.kB)){
                poseToScore = new Pose2d(3.05,3.87,Rotation2d.fromDegrees(0));
            }
            else if (touch.getButtonValue(VirtualButton.kC)){
                poseToScore = new Pose2d(3.57,2.92,Rotation2d.fromDegrees(52));
            }
            else if (touch.getButtonValue(VirtualButton.kD)){
                poseToScore = new Pose2d(3.96,2.69,Rotation2d.fromDegrees(60));
            }
            else if (touch.getButtonValue(VirtualButton.kE)){
                poseToScore = new Pose2d(5.0,2.67,Rotation2d.fromDegrees(122));
            }
            else if (touch.getButtonValue(VirtualButton.kF)){
                poseToScore = new Pose2d(5.27,2.87,Rotation2d.fromDegrees(120));
            }
            else if (touch.getButtonValue(VirtualButton.kG)){
                poseToScore = new Pose2d(5.90,3.84,Rotation2d.fromDegrees(180));
            }
            else if (touch.getButtonValue(VirtualButton.kH)){
                poseToScore = new Pose2d(5.90,4.19,Rotation2d.fromDegrees(180));
            }
            else if (touch.getButtonValue(VirtualButton.kI)){
                poseToScore = new Pose2d(5.31,5.17,Rotation2d.fromDegrees(-115));
            }
            else if (touch.getButtonValue(VirtualButton.kJ)){
                poseToScore = new Pose2d(5.02,5.35,Rotation2d.fromDegrees(-120));
            }
            else if (touch.getButtonValue(VirtualButton.kK)){
                poseToScore = new Pose2d(3.92,5.34,Rotation2d.fromDegrees(-60));
            }
            else if (touch.getButtonValue(VirtualButton.kL)){
                poseToScore = new Pose2d(3.64,5.18,Rotation2d.fromDegrees(-60));
            }
            else{
                poseToScore = new Pose2d();
            }
        }

        return poseToScore;
    }
}
