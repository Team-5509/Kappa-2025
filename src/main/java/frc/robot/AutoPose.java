package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoPose {

    private static final Pose2d[] poses = { //TODO add poses
        new Pose2d(0, 0, new Rotation2d(0)), //Red coral stations
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)), //Red processor
        new Pose2d(0, 0, new Rotation2d(0)), //Red barge
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)), //Red reef
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)), //Blue coral stations
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)), //Blue processor
        new Pose2d(0, 0, new Rotation2d(0)), //Blue barge
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)), //Blue reef
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(0, 0, new Rotation2d(0))
    };

    public static Command drive(SwerveSubsystem drivebase) {
        Pose2d pos = NotCodedYet.magicallyGetBestPose();
        drivebase.driveToPose(pos);
        return Commands.none();
    }
}
