package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
/**
 * Drives to reef:
 * - first a coarse 3-ft approach,
 * - then a precise nudge to final, while lifting to L3,
 * - then outtakes.
 */
public class ReefScoreCommand extends SequentialCommandGroup {

  public enum ReefSide {
    LEFT,
    RIGHT
  }

  private final SwerveSubsystem m_drive;
  private final ElevatorSubsystem m_elevator;
  private final Command m_outtakeWithSensor2;
  private final ReefSide m_sector;
  private final Setpoint m_elevatorSetpoint;

  private static final double LONGSET_IN = 50.72;

  public void printPose(String key, Pose2d pose) {
    SmartDashboard.putString(key,
        "x:" + prettyDouble(pose.getX()) +
            " y:" + prettyDouble(pose.getY()) +
            " R:" + prettyDouble(pose.getRotation().getDegrees()));
  }

  public ReefScoreCommand(
      SwerveSubsystem drive,
      ElevatorSubsystem elevator,
      Command outtakeWithSensor2,
      ReefSide sectorChoice,
      Setpoint elevatorSetpoint) {

    this.m_drive = drive;
    this.m_elevator = elevator;
    this.m_outtakeWithSensor2 = outtakeWithSensor2;
    this.m_sector = sectorChoice;
    this.m_elevatorSetpoint = elevatorSetpoint;

    addRequirements(drive);

    Command shouldIShoot = m_outtakeWithSensor2;
    shouldIShoot = Setpoint.kLevel4 == elevatorSetpoint ? Commands.none() : m_outtakeWithSensor2;

  ((OuttakeWithSensor) m_outtakeWithSensor2).setSpeed(0.35);

    
    addCommands(
        driveParent()
        ,
        Commands.parallel(driveToFinal(), m_elevator.setSetpointCommand(m_elevatorSetpoint)),
        Commands.waitSeconds(0.25),
        shouldIShoot
        );
    ((OuttakeWithSensor) m_outtakeWithSensor2).resetSpeed();
        
  }

  /**
   * Step 1: coarse approach ~3 ft beyond LONGSET, then transition into the fine
   */
  private Command driveParent() {
    return Commands.defer(() -> {
      double totalLongsetMeters = Units.inchesToMeters(LONGSET_IN + 12 * 3);
      Pose2d current = m_drive.getPose();
      printPose("Current_Pose_from_3ft_away", current);

      double sectorDeg = getReefSector(current);
      Pose2d target = altComputeLocation(totalLongsetMeters, /* m= */0.0, sectorDeg, current);

      return m_drive.driveToPose(target).until(() -> {
        Pose2d pose = m_drive.getPose();
        double dist = pose.getTranslation().getDistance(target.getTranslation());
        double angleErr = Math.abs(pose.getRotation().minus(target.getRotation()).getRadians());
        return dist < 0.5 && angleErr < Math.toRadians(10);
      });
    }, Set.of(m_drive));
  }

  /** Step 2: precise nudge to final pose (two nudges). */
  private Command driveToFinal() {
    return Commands.defer(() -> {

      double totalLongsetMeters = Units.inchesToMeters(LONGSET_IN + 6);
      double mOffset = 0.164338;
      if (m_sector == ReefSide.LEFT) {
        mOffset *= -1;
      }
      
      // changes which side is left/right based on alliance
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
        mOffset *= -1;
      }

      Pose2d current = m_drive.getPose();

      double sectorDeg = getReefSector(current);

      Pose2d target = altComputeLocation(totalLongsetMeters, mOffset, sectorDeg, m_drive.getPose());

      return m_drive.nudgeToPose(target).andThen(m_drive.nudgeToPose(target));
    }, Set.of(m_drive));
  }

  /**
   * Computes the target pose given a radial distance (length), a lateral offset
   */
  private static Pose2d altComputeLocation(double lengthMeters, double mMeters, double degrees,
      Pose2d fallbackPose) {
    final double centerXRed = 13.059;
    final double centerXBlue = 4.489;

    final double yc = 4.0259;

    double xc;
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      xc = fallbackPose.getX();
    } else if (alliance.get() == DriverStation.Alliance.Red) {
      xc = centerXRed;
    } else {
      xc = centerXBlue;
    }

    double theta = Math.toRadians(degrees);
    double xr = xc + lengthMeters * Math.cos(theta) - mMeters * Math.sin(theta);
    double yr = yc + lengthMeters * Math.sin(theta) + mMeters * Math.cos(theta);
    return new Pose2d(xr, yr, Rotation2d.fromDegrees(degrees - 180.0));
  }

  public static String prettyDouble(double value, int precision) {
    return Math.round(value * Math.pow(10, precision)) / Math.pow(10, precision) + "";
  }

  public static String prettyDouble(double value) {
    return prettyDouble(value, 2);
  }

  public Double getReefSector(Pose2d currentPose) {
    Double centerXRed = Units.inchesToMeters(514.13);
    Double centerXBlue = Units.inchesToMeters(176.75);
    Double centerY = Units.inchesToMeters(158.5);

    Translation2d reefRelativePose = null;

    var alliance = DriverStation.getAlliance();
    // return currentPose.getRotation().getDegrees();
    if (!alliance.isPresent()) {
      return null;

    } else if (alliance.get() == DriverStation.Alliance.Red) {
      reefRelativePose = new Translation2d(currentPose.getX() - centerXRed, currentPose.getY() - centerY);

    } else {
      reefRelativePose = new Translation2d(currentPose.getX() - centerXBlue, currentPose.getY() - centerY);
    }

    double reefRelativeAngle = Math.toDegrees(Math.atan2(reefRelativePose.getY(), reefRelativePose.getX()));
    reefRelativeAngle = ((reefRelativeAngle % 360) + 360) % 360;
    SmartDashboard.putNumber("reefRelativeAngle:", reefRelativeAngle);
    if (0 <= reefRelativeAngle && reefRelativeAngle <= 30) {
      return 0.0;
    } else if (30 < reefRelativeAngle && reefRelativeAngle <= 90) {
      return 60.0;
    } else if (90 < reefRelativeAngle && reefRelativeAngle <= 150) {
      return 120.0;
    } else if (150 < reefRelativeAngle && reefRelativeAngle <= 210) {
      return 180.0;
    } else if (210 < reefRelativeAngle && reefRelativeAngle <= 270) {
      return 240.0;
    } else if (270 < reefRelativeAngle && reefRelativeAngle <= 330) {
      return 300.0;
    } else if (330 < reefRelativeAngle && reefRelativeAngle <= 360) {
      return 0.0;
    } else {
      return null;
    }
  }

  public static Command score(
      SwerveSubsystem drive,
      ElevatorSubsystem elevator,
      Command outtakeWithSensor2,
      Setpoint setpoint,
      ReefSide sector) {
    return new ReefScoreCommand(
        drive, elevator, outtakeWithSensor2, sector, setpoint);
  }

  // L2
  public static Command scoreL2Left(
      SwerveSubsystem drive, ElevatorSubsystem elevator, Command outtakeWithSensor2) {
    return score(drive, elevator, outtakeWithSensor2,
        Setpoint.kLevel2, ReefSide.LEFT);
  }

  public static Command scoreL2Right(
      SwerveSubsystem drive, ElevatorSubsystem elevator, Command outtakeWithSensor2) {
    return score(drive, elevator, outtakeWithSensor2,
        Setpoint.kLevel2, ReefSide.RIGHT);
  }

  // L3
  public static Command scoreL3Left(
      SwerveSubsystem drive, ElevatorSubsystem elevator, Command outtakeWithSensor2) {
    return score(drive, elevator, outtakeWithSensor2,
        Setpoint.kLevel3, ReefSide.LEFT);
  }

  public static Command scoreL3Right(
      SwerveSubsystem drive, ElevatorSubsystem elevator, Command outtakeWithSensor2) {
    return score(drive, elevator, outtakeWithSensor2,
        Setpoint.kLevel3, ReefSide.RIGHT);
  }

  // L4
  public static Command scoreL4Left(
      SwerveSubsystem drive, ElevatorSubsystem elevator, Command outtakeWithSensor2) {
    return score(drive, elevator, outtakeWithSensor2,
        Setpoint.kLevel4, ReefSide.LEFT);
  }

  public static Command scoreL4Right(
      SwerveSubsystem drive, ElevatorSubsystem elevator, Command outtakeWithSensor2) {
    return score(drive, elevator, outtakeWithSensor2,
        Setpoint.kLevel4, ReefSide.RIGHT);
  }
}
