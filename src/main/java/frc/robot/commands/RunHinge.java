// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunHinge extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final HingeSubsystem m_hingeSubsystem;
  private final DoubleSupplier m_controllerInput;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunHinge(HingeSubsystem subsystem, DoubleSupplier controllerInput) {
    m_hingeSubsystem = subsystem;
    m_controllerInput = controllerInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hingeSubsystem.setHingePower(m_controllerInput.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hingeSubsystem.syncHingeControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
