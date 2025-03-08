// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.Queue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants;
// import frc.robot.BlinkinLEDController;
// import frc.robot.BlinkinLEDController.BlinkinPattern;
/** An example command that uses an example subsystem. */
public class IntakeWithSensor extends Command {
  private static Spark m_blinkin;
  // private static BlinkinPattern m_currentPattern;
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_intakeSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeWithSensor(IntakeSubsystem subsystem) {
    m_intakeSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakePower(IntakeSetpoints.kForward);

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
    if (m_intakeSubsystem.getIntakeInput()) {
      m_intakeSubsystem.setIntakePower(0);
      // BlinkinLEDController.setPattern(BlinkinPattern.GOLD);
      return true;
    } else {
      return false;
    }
  }
}
