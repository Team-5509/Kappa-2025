// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.ElevatorSubsystemConstants.ElevatorSetpoints;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import edu.wpi.first.wpilibj2.command.Command;

/** An AutoElevatorCoral command that uses an Elevator subsystem. */
public class AutoElevatorCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevatorSubSystem;
 private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoElevatorCoral(ElevatorSubsystem subsystem) {
    m_elevatorSubSystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubSystem.setElevatorCurrentTarget(elevatorCurrentTarget); 
    m_elevatorSubSystem.moveToSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_elevatorSubSystem.getElevatorEncoder() >= elevatorCurrentTarget - 1 
    || m_elevatorSubSystem.getElevatorEncoder() <= elevatorCurrentTarget + 1){
      return true;
    }
    return false;
  }
}
