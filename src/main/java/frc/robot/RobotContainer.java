// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.HangSubsystemConstants;
import frc.robot.Constants.HangSubsystemConstants.HangSetpoints;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.commands.AutoElevator;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunHinge;
import frc.robot.Constants.IntakeSubsystemConstants;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final ElevatorSubsystem m_elevatorSubSystem = new ElevatorSubsystem();
  private final HingeSubsystem m_hingeSubSystem = new HingeSubsystem();
  private final HangSubsystem m_hangSubSystem = new HangSubsystem();
  private final IntakeSubsystem m_intakeSubSystem = new IntakeSubsystem();
  
  private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController auxXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MAXIMUM_FACTOR)
      .cubeRotationControllerAxis(true)
      .cubeTranslationControllerAxis(true)
      .allianceRelativeControl(true);

  // private final static double speedFactorCalculation(Double TriggerAxis){
  // double speedFactorCalculation =
  // -(Constants.OperatorConstants.SPEED_MAXIMUM_FACTOR -
  // Constants.OperatorConstants.SPEED_MINIMUM_FACTOR)
  // *(TriggerAxis) + Constants.OperatorConstants.SPEED_MAXIMUM_FACTOR;
  // return speedFactorCalculation(TriggerAxis);
  // }
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity with a trigger axis based slowdown.
   */
  SwerveInputStream driveAngularVelocityFinesse = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MINIMUM_FACTOR)
      .cubeRotationControllerAxis(true)
      .cubeTranslationControllerAxis(true)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MAXIMUM_FACTOR)
      .cubeRotationControllerAxis(true)
      .cubeTranslationControllerAxis(true)
      .robotRelative(true)
      .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("HingeL4", m_hingeSubSystem.setSetpointCommand(HingeSubsystem.Setpoint.kLevel4));
    // NamedCommands.registerCommand("AutoElevator", m_elevatorSubSystem.AutoElevator);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityFinnese = drivebase.driveFieldOriented(driveAngularVelocityFinesse);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleSim);
        Command runHinge = new RunHinge(m_hingeSubSystem, () -> auxXbox.getRightY() );
        Command runElevator = new RunElevator(m_elevatorSubSystem, () -> auxXbox.getLeftY() );

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else {

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.y().onTrue(m_hangSubSystem.setSetpointCommand(HangSubsystem.Setpoint.kLevel2));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().whileTrue(driveRobotOrientedAngularVelocity.repeatedly());
      driverXbox.x().whileTrue(driveFieldOrientedAnglularVelocityFinnese);

      // Auxilary Controller 
      auxXbox.b().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel1));
      auxXbox.a().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel2));
      auxXbox.x().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel3));
      auxXbox.y().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel4));
      auxXbox.rightBumper().onTrue(m_intakeSubSystem.reverseIntakeCommand());
      auxXbox.leftBumper().onTrue(m_intakeSubSystem.runIntakeCommand());
      auxXbox.axisMagnitudeGreaterThan(5, 0.2).whileTrue(runHinge);
      auxXbox.start().onTrue(m_hingeSubSystem.setSetpointCommand(HingeSubsystem.Setpoint.kLevel4));
      auxXbox.axisMagnitudeGreaterThan(1, 0.2).whileTrue(runElevator);

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("Test");
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
