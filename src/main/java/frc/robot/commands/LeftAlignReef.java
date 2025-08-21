// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.swervedrive.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Math.*;

/** An example command that uses an example subsystem. */
public class LeftAlignReef extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem m_drive;

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

  public Command drive3FtAway() {
    return m_drive.driveToPose(computeLocation(11
    ));
  }
public static String prettyDouble(double value, int precision){
  return Math.round(value*Math.pow(10, precision))/Math.pow(10, precision)+"";
} 
public static String prettyDouble(double value){
  return prettyDouble(value,2);
}
  public Pose2d computeLocation(int id) {
    final double CENTER_X = 514.13;
    final double CENTER_Y = 158.5;
    final double OFFSET = 32.714 + 12*2 + 36/2;
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
    x = in2m(x);
    y = in2m(y);
    SmartDashboard.putString("driveToPose", "x"+ prettyDouble(x) + "y" + prettyDouble(y) + "r" + prettyDouble(Math.toDegrees(finalAngle)));

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
