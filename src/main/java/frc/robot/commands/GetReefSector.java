package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class GetReefSector{
    public Double getReefSector(Pose2d currentPose){
        Double centerXRed = 514.13;
        Double centerXBlue = 176.75;
        Double centerY = 158.5;

        Translation2d reefRelativePose = null;

        var alliance = DriverStation.getAlliance();
        if(!alliance.isPresent()){
            return null;

        } else if(alliance.get() == DriverStation.Alliance.Red){
            reefRelativePose = new Translation2d(currentPose.getX() - centerXRed, currentPose.getY() - centerY);
        
        } else {
            reefRelativePose = new Translation2d(currentPose.getX() - centerXBlue, currentPose.getY() - centerY);
        }

        double reefRelativeAngle = Math.toDegrees(Math.atan2(reefRelativePose.getY(), reefRelativePose.getX()));
        
        if(0 <= reefRelativeAngle || reefRelativeAngle <= 30){
            return 0.0;
        } else if(30 < reefRelativeAngle || reefRelativeAngle <= 90){
            return 60.0;
        } else if(90 < reefRelativeAngle || reefRelativeAngle <= 150){
            return 120.0;
        } else if(150 < reefRelativeAngle || reefRelativeAngle <= 210){
            return 180.0;
        } else if(210 < reefRelativeAngle || reefRelativeAngle <= 270){
            return 240.0;
        } else if(270 < reefRelativeAngle || reefRelativeAngle <= 330){
            return 300.0;
        } else if(330 < reefRelativeAngle || reefRelativeAngle <= 360){
            return 0.0;
        } else {
            return null;
        }
    }
}