// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.swervedrive.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LeftAlignReef extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem m_drive;
  public GetReefSector getReefSector = new GetReefSector();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LeftAlignReef(SwerveSubsystem subsystem) {
    m_drive = subsystem;

    SmartDashboard.putNumber("Coral/Elevator/BiggestVisibleIdLeft", -1);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  private double inchesToMeters(double inches) {
    return inches * 0.0254;
  }
 public Pose2d altComputeLocation(double length, double m, double degrees){
    Double centerXRed = 13.059;
    Double centerXBlue = 4.489;
    double xc = 13.059;
    double yc = 4.0259;

    var alliance = DriverStation.getAlliance();
      if(!alliance.isPresent()){
          return null;
      } else if(alliance.get() == DriverStation.Alliance.Red){
        xc = centerXRed;
      } else {
        xc = centerXBlue;
      }
  
  double xr = xc + length * Math.cos(Math.toRadians(degrees))- m * Math.sin(Math.toRadians(degrees));
  double yr = yc + length * Math.sin(Math.toRadians(degrees))+ m * Math.cos(Math.toRadians(degrees));
  return new Pose2d(xr , yr , Rotation2d.fromDegrees(degrees-180));
 }
  public Command drive3FtAway() {
    //Pose2d customPose = computeLocation(11);
    double totalLongset = (LONGSET + 12*3) * 0.0254;
    Pose2d t = m_drive.getPose();
    SmartDashboard.putString("I am here", "x:" + t.getX() + "y:" + t.getY() + "R:"+ t.getRotation().getDegrees());

    Pose2d customPose = altComputeLocation(totalLongset, 0, getReefSector.getReefSector(m_drive.getPose()));
    //Pose2d customPose = new Pose2d(inchesToMeters(494.38),inchesToMeters(111.36), Rotation2d.fromDegrees(60));
    //Pose2d customPose = new Pose2d(12.527, 2.763, Rotation2d.fromDegrees(60));
    SmartDashboard.putString("I am going here", "x:" + customPose.getX() + "y:" + customPose.getY() + "R:"+ customPose.getRotation().getDegrees());
    return m_drive.driveToPose(customPose).until(() -> {
      Pose2d currentPose = m_drive.getPose();
      double distance = currentPose.getTranslation().getDistance(customPose.getTranslation());
      double angleError = Math.abs(currentPose.getRotation().minus(customPose.getRotation()).getRadians());
      return distance < 0.5 && angleError < Math.toRadians(10);
    });
  }
private final double LONGSET = 50.72; //sum of center of coral to tag + half of length of robot(in inches)

  public Command driveToFinal() {
    double totalLongset = (LONGSET + 6) * 0.0254;

    //Pose2d customPose = computeLocation(11);
    //Pose2d customPose = new Pose2d(inchesToMeters(494.38),inchesToMeters(111.36), Rotation2d.fromDegrees(60));
    // Pose2d customPose = new Pose2d(12.5, 2.723, Rotation2d.fromDegrees(60));
    Pose2d customPose = altComputeLocation(totalLongset, 0.164338, getReefSector.getReefSector(m_drive.getPose()));

    return m_drive.nudgeToPose(customPose).andThen(m_drive.nudgeToPose(customPose));
  }

  public static String prettyDouble(double value, int precision) {
    return Math.round(value * Math.pow(10, precision)) / Math.pow(10, precision) + "";
  }

  public static String prettyDouble(double value) {
    return prettyDouble(value, 2);
  }

  public Pose2d computeLocation(int id) {
    final double CENTER_X = 514.13;
    final double CENTER_Y = 158.5;
    final double OFFSET = 32.714 + 12 * 2 + 36 / 2;
    // for (int i = 0; i < 6; i++){
    int i = id - 7;
    if (id == 6) {
      i = 5;
    }
    double angle = Math.toRadians(i * 60);
    double x = CENTER_X + OFFSET * Math.cos(angle);

    double y = CENTER_Y + OFFSET * Math.sin(angle);
    double finalAngle = Math.toRadians(-180) + angle;
    // }
    x = x * 0.0254;
    y = y * 0.0254;
    SmartDashboard.putString("driveToPose",
        "x" + prettyDouble(x) + "y" + prettyDouble(y) + "r" + prettyDouble(Math.toDegrees(finalAngle)));

    Rotation2d finalRotation = new Rotation2d(finalAngle);
    return new Pose2d(x, y, finalRotation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.vision.scanLeft();
    if (m_drive.isRedAlliance()) {
      int detectedId = 0;
      if (detectedId >= 6 && detectedId <= 11) {

      }
    } else {
      int detectedId = 0;
      if (detectedId >= 17 && detectedId <= 22) {

      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
