package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GetReefSector{
    public String getRedReefSector(Pose2d currentPose){
        Translation2d reefRelativePose = new Translation2d(currentPose.getX() - 514.13, currentPose.getY() - 158.5);

        double reefRelativeAngle = Math.toDegrees(Math.atan2(reefRelativePose.getY(), reefRelativePose.getX()));
        
        if(0 <= reefRelativeAngle || reefRelativeAngle <= 30){
            return "7,R";
        } else if(30 < reefRelativeAngle || reefRelativeAngle <= 60){
            return "8,L";
        } else if(60 < reefRelativeAngle || reefRelativeAngle <= 90){
            return "8,R";
        } else if(90 < reefRelativeAngle || reefRelativeAngle <= 120){
            return "9,L";
        } else if(120 < reefRelativeAngle || reefRelativeAngle <= 150){
            return "9,R";
        } else if(150 < reefRelativeAngle || reefRelativeAngle <= 180){
            return "10,L";
        } else if(180 < reefRelativeAngle || reefRelativeAngle <= 210){
            return "10,R";
        } else if(210 < reefRelativeAngle || reefRelativeAngle <= 240){
            return "11,L";
        } else if(240 < reefRelativeAngle || reefRelativeAngle <= 270){
            return "11,R";
        } else if(270 < reefRelativeAngle || reefRelativeAngle <= 300){
            return "6,L";
        } else if(300 < reefRelativeAngle || reefRelativeAngle <= 330){
            return "6,R";
        } else if(330 < reefRelativeAngle || reefRelativeAngle <= 360){
            return "7,L";
        } else{
            return "0,0";
        }
    }

    public String getBlueReefSector(Pose2d currentPose){
        Translation2d reefRelativePose = new Translation2d(currentPose.getX() - 176.75, currentPose.getY() - 158.5);

        double reefRelativeAngle = Math.toDegrees(Math.atan2(reefRelativePose.getY(), reefRelativePose.getX()));
        
        if(0 <= reefRelativeAngle || reefRelativeAngle <= 30){
            return "21,R";
        } else if(30 < reefRelativeAngle || reefRelativeAngle <= 60){
            return "20,L";
        } else if(60 < reefRelativeAngle || reefRelativeAngle <= 90){
            return "20,R";
        } else if(90 < reefRelativeAngle || reefRelativeAngle <= 120){
            return "19,L";
        } else if(120 < reefRelativeAngle || reefRelativeAngle <= 150){
            return "19,R";
        } else if(150 < reefRelativeAngle || reefRelativeAngle <= 180){
            return "18,L";
        } else if(180 < reefRelativeAngle || reefRelativeAngle <= 210){
            return "18,R";
        } else if(210 < reefRelativeAngle || reefRelativeAngle <= 240){
            return "17,L";
        } else if(240 < reefRelativeAngle || reefRelativeAngle <= 270){
            return "17,R";
        } else if(270 < reefRelativeAngle || reefRelativeAngle <= 300){
            return "22,L";
        } else if(300 < reefRelativeAngle || reefRelativeAngle <= 330){
            return "22,R";
        } else if(330 < reefRelativeAngle || reefRelativeAngle <= 360){
            return "21,L";
        } else{
            return "0,0";
        }
    }
}
