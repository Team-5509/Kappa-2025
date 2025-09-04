package frc.robot.commands;

import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CoralPickupCommand extends SequentialCommandGroup {

  private final SwerveSubsystem m_drive;
  private final ElevatorSubsystem m_elevator;
  private final Setpoint m_elevatorSetpoint;
  private final int m_id;
  private final Double m_lInches;
  private final Double m_mInches;

  
  private static final class TagAnchor {
    final double xc_in, yc_in, rotDeg;
    final Alliance alliance;

    TagAnchor(double xc_in, double yc_in, double rotDeg,
        Alliance alliance) {
      this.xc_in = xc_in;
      this.yc_in = yc_in;
      this.rotDeg = rotDeg;
      this.alliance = alliance;
    }
  }

  private static final Map<Integer, TagAnchor> TAGS = Map.of(
      1, new TagAnchor(657.37, 25.80, 126, Alliance.Red),
      2, new TagAnchor(657.37, 291.20, 234, Alliance.Red),
      12, new TagAnchor(33.51, 25.80, 54, Alliance.Blue),
      13, new TagAnchor(33.51, 291.20, 306, Alliance.Blue));

  public CoralPickupCommand(
      SwerveSubsystem drive,
      ElevatorSubsystem elevator,
      int id,
      Setpoint elevatorSetpoint,
      Double lInches,
      Double mInches) {

    this.m_drive = drive;
    this.m_elevator = elevator;
    this.m_elevatorSetpoint = elevatorSetpoint;
    this.m_id = id;
    this.m_lInches = lInches;
    this.m_mInches = mInches;

    addRequirements(drive);

    addCommands(
        drive3FtAway(), // coarse to l + 36 in
        // Commands.parallel(driveToFinal(),
        // m_elevator.setSetpointCommand(m_elevatorSetpoint)),
        m_elevator.setSetpointCommand(m_elevatorSetpoint),
        driveToFinal());
  }


  private Command drive3FtAway() {
    return Commands.defer(() -> {
      TagAnchor tag = getCheckedAnchor(m_id);
      if (tag == null)
        return Commands.none();

      double lCoarse_in = m_lInches;
      Pose2d target = altComputeLocation(tag, lCoarse_in, m_mInches);

      return m_drive.driveToPose(target).until(() -> {
        Pose2d pose = m_drive.getPose();
        double dist = pose.getTranslation().getDistance(target.getTranslation());
        double angleErr = Math.abs(pose.getRotation().minus(target.getRotation()).getRadians());
        return dist < 0.5 && angleErr < Math.toRadians(50);
      });
    }, Set.of(m_drive));
  }

  private Command driveToFinal() {
    return Commands.defer(() -> {
      TagAnchor tag = getCheckedAnchor(m_id);

      Pose2d target = altComputeLocation(tag, m_lInches, m_mInches);
      return m_drive.nudgeToPose(target).andThen(m_drive.nudgeToPose(target));
    }, Set.of(m_drive));
  }

  private static TagAnchor getCheckedAnchor(int id) {
    TagAnchor tag = TAGS.get(id);
    if (tag == null) {
      DriverStation.reportError("No AprilTag anchor for id " + id, false);
      return null;
    }
    // Optional<Alliance> a = DriverStation.getAlliance();
    // if (a.isEmpty()) {
    //   DriverStation.reportError("Alliance not available for id " + id, false);
    //   return null;
    // }
    // if (tag.alliance != a.get()) {
    //   DriverStation.reportError(
    //       "Tag " + id + " is for " + tag.alliance + ", not " + a.get(), false);
    //   return null;
    // }
    return tag;
  }

  private static Pose2d altComputeLocation(TagAnchor tag, double lInches, double mInches) {
    double thetaRad = Math.toRadians(tag.rotDeg);
    double xc_m = Units.inchesToMeters(tag.xc_in);
    double yc_m = Units.inchesToMeters(tag.yc_in);
    double l_m = Units.inchesToMeters(lInches);
    double m_m = Units.inchesToMeters(mInches);

    double xr = xc_m + l_m * Math.cos(thetaRad) - m_m * Math.sin(thetaRad);
    double yr = yc_m + l_m * Math.sin(thetaRad) + m_m * Math.cos(thetaRad);
    // Face the tag center:
    return new Pose2d(xr, yr, Rotation2d.fromDegrees(tag.rotDeg));
  }

  public static CoralPickupCommand create(
      SwerveSubsystem drive,
      ElevatorSubsystem elevator,
      int id,
      Setpoint elevatorSetpoint,
      Double lInches,
      Double mInches) {
    return new CoralPickupCommand(drive, elevator, id, elevatorSetpoint, lInches, mInches);
  }

  public static CoralPickupCommand pickupLeft(
      SwerveSubsystem drive,
      ElevatorSubsystem elevator) {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    int id = (alliance.orElse(Alliance.Red) == Alliance.Red) ? 1 : 13;
  
    return new CoralPickupCommand(drive, elevator,
        id, ElevatorSubsystem.Setpoint.kFeederStation, 30.0, 0.0);
  }

  /**
   * FIXME 
   * @param drive
   * @param elevator
   * @return
   */
  public static CoralPickupCommand pickupRight(
      SwerveSubsystem drive,
      ElevatorSubsystem elevator) {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        int id = (alliance.orElse(Alliance.Red) == Alliance.Red) ? 2 : 12;


        return new CoralPickupCommand(drive, elevator, 
        id, ElevatorSubsystem.Setpoint.kFeederStation , 30.0, 0.0);
  }

}
