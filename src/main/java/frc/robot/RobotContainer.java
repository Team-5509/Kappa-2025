// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Pair;
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
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.HangSubsystemConstants;
import frc.robot.Constants.HangSubsystemConstants.HangSetpoints;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.commands.AutoElevatorCoral;
import frc.robot.commands.AutoElevatorTrough;
import frc.robot.commands.AutoElevatorkLevel1AndAHalf;
import frc.robot.commands.AutoElevatorkLevel2;
import frc.robot.commands.AutoElevatorkLevel3;
import frc.robot.commands.AutoElevatorkLevel4;
import frc.robot.commands.CoralPickupCommand;
import frc.robot.commands.ReefScoreCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeWithSensor;
import frc.robot.commands.OuttakeWithSensor;
import frc.robot.commands.RunOuttake;
import frc.robot.commands.ReefScoreCommand.ReefSide;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunHinge;
import frc.robot.commands.RunHang;
import frc.robot.commands.RunHangReverse;
import frc.robot.Constants.IntakeSubsystemConstants;
import java.io.File;
import java.util.HashMap;
import java.util.Set;

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


  private final ScoreSelection sel = new ScoreSelection();

  private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController auxXbox = new CommandXboxController(1);
  final CommandXboxController selXbox = new CommandXboxController(2);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
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
      .robotRelative(false)
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
      .robotRelative(false)
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
      .scaleTranslation(OperatorConstants.SPEED_MINIMUM_FACTOR)
      .cubeRotationControllerAxis(true)
      .cubeTranslationControllerAxis(true)
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveRobotOrientedStrafeLeftFinesse = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 0,
      () -> -0.6)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * 0)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MINIMUM_FACTOR)
      .cubeRotationControllerAxis(true)
      .cubeTranslationControllerAxis(true)
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveRobotOrientedStrafeRightFinesse = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 0,
      () -> 0.6)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * 0)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MINIMUM_FACTOR)
      .cubeRotationControllerAxis(true)
      .cubeTranslationControllerAxis(true)
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveRobotOrientedStrafeUpFinesse = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> 0.6,
      () -> driverXbox.getRightY() * 0)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * 0)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MINIMUM_FACTOR)
      .cubeRotationControllerAxis(true)
      .cubeTranslationControllerAxis(true)
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveRobotOrientedStrafeDownFinesse = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -0.6,
      () -> driverXbox.getRightY() * 0)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * 0)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.SPEED_MINIMUM_FACTOR)
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
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) * Math.PI)
          * (Math.PI * 2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) * Math.PI)
              *
              (Math.PI * 2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("HingeL4", m_hingeSubSystem.setSetpointCommand(HingeSubsystem.Setpoint.kLevel4));
    NamedCommands.registerCommand("AutoElevatorCoral", new AutoElevatorCoral(m_elevatorSubSystem));
    NamedCommands.registerCommand("AutoElevatorTrough", new AutoElevatorTrough(m_elevatorSubSystem));
    NamedCommands.registerCommand("AutoElevatorkLevel1AndAHalf", new AutoElevatorkLevel1AndAHalf(m_elevatorSubSystem));
    NamedCommands.registerCommand("AutoElevatorkLevel2", new AutoElevatorkLevel2(m_elevatorSubSystem));
    NamedCommands.registerCommand("AutoElevatorkLevel3", new AutoElevatorkLevel3(m_elevatorSubSystem));
    NamedCommands.registerCommand("AutoElevatorkLevel4", new AutoElevatorkLevel4(m_elevatorSubSystem));
    NamedCommands.registerCommand("AutoIntake", new IntakeWithSensor(m_intakeSubSystem));
    NamedCommands.registerCommand("AutoOuttake", new OuttakeWithSensor(m_intakeSubSystem , 0.20));
    NamedCommands.registerCommand("CLeftDriveToPose",
        drivebase.driveToPose(new Pose2d(new Translation2d(5.311, 5.098), Rotation2d.fromDegrees(-156.631))));
    NamedCommands.registerCommand("BLeftDriveToPose",
        drivebase.driveToPose(new Pose2d(new Translation2d(3.937, 5.203), Rotation2d.fromDegrees(-35.272))));
    NamedCommands.registerCommand("BRightDriveToPose",
        drivebase.driveToPose(new Pose2d(new Translation2d(3.623, 5.135), Rotation2d.fromDegrees(-41.186))));
    NamedCommands.registerCommand("ELeftDriveToPose",
        drivebase.driveToPose(new Pose2d(new Translation2d(5.031, 2.826), Rotation2d.fromDegrees(156.864))));
    NamedCommands.registerCommand("FLeftDriveToPose",
        drivebase.driveToPose(new Pose2d(new Translation2d(3.665, 2.894), Rotation2d.fromDegrees(33.889))));
    NamedCommands.registerCommand("FRightDriveToPose",
        drivebase.driveToPose(new Pose2d(new Translation2d(3.895, 2.685), Rotation2d.fromDegrees(33.889))));
    NamedCommands.registerCommand("DLeftDriveToPose",
        drivebase.driveToPose(new Pose2d(new Translation2d(5.864, 4.151), Rotation2d.fromDegrees(166.675))));

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
    Command driveRobotOrientedStrafeLeftFinneseCommand = drivebase
        .driveFieldOriented(driveRobotOrientedStrafeLeftFinesse);
    Command driveRobotOrientedStrafeRightFinneseCommand = drivebase
        .driveFieldOriented(driveRobotOrientedStrafeRightFinesse);
    Command driveRobotOrientedStrafeUpFinneseCommand = drivebase.driveFieldOriented(driveRobotOrientedStrafeUpFinesse);
    Command driveRobotOrientedStrafeDownFinneseCommand = drivebase
        .driveFieldOriented(driveRobotOrientedStrafeDownFinesse);

    // Command snapToReef = new LeftAlignReef(drivebase);
    // LeftAlignReef snapToReef_1 = new LeftAlignReef(drivebase);

    Command runHingeReverse = new RunHinge(m_hingeSubSystem, () -> 0.25);
    Command runHingeForward = new RunHinge(m_hingeSubSystem, () -> -0.25);
    Command runHang = new RunHang(m_hangSubSystem, () -> driverXbox.getRightTriggerAxis() * -1);
    Command runHangReverse = new RunHangReverse(m_hangSubSystem, () -> driverXbox.getLeftTriggerAxis() * -1);
    Command runOuttake = new RunOuttake(m_intakeSubSystem, () -> auxXbox.getRightY() * -1);
    Command runElevator = new RunElevator(m_elevatorSubSystem, () -> auxXbox.getLeftY() * -0.25);
    Command outtakeWithSensor = new OuttakeWithSensor(m_intakeSubSystem);
    Command intakeWithSensor = new IntakeWithSensor(m_intakeSubSystem);

    // sel.generateAllCommands(drivebase, m_elevatorSubSystem, outtakeWithSensor3);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else {

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(10, 8, new Rotation2d()))));
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
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

      driverXbox.leftBumper().whileTrue(driveRobotOrientedAngularVelocity.repeatedly());
      driverXbox.rightBumper().whileTrue(driveFieldOrientedAnglularVelocityFinnese);
      driverXbox.axisMagnitudeGreaterThan(3, 0.2).whileTrue(runHang);
      driverXbox.axisMagnitudeGreaterThan(2, 0.2).whileTrue(runHangReverse);
      driverXbox.povRight().whileTrue(driveRobotOrientedStrafeLeftFinneseCommand);
      driverXbox.povLeft().whileTrue(driveRobotOrientedStrafeRightFinneseCommand);
      driverXbox.povUp().whileTrue(driveRobotOrientedStrafeUpFinneseCommand);
      driverXbox.povDown().whileTrue(driveRobotOrientedStrafeDownFinneseCommand);

      // driverXbox.y().whileTrue(ReefScoreCommand.scoreL3Right(drivebase, m_elevatorSubSystem, outtakeWithSensor));

      // driverXbox.b().onTrue(Commands.runOnce(() -> {
      //   sel.nextCycleLevel();
      // }));
     
      // driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().onTrue(Commands.runOnce(() -> {
      //   sel.toggleSide();
      // }));

    //   driverXbox.a().whileTrue(
    //     Commands.defer(
    //         () -> {
    //             var level = sel.level();
    //             var side  = sel.side();
    //             return ReefScoreCommand.score(
    //                 drivebase, m_elevatorSubSystem, outtakeWithSensor, level, side
    //             );
    //         },
    //         Set.of(drivebase, m_elevatorSubSystem)
    //     )
    // );

      driverXbox.x().whileTrue(
        Commands.defer(
            () -> CoralPickupCommand.pickupLeft(drivebase, m_elevatorSubSystem),
            Set.of(drivebase, m_elevatorSubSystem)
        )
      );

      driverXbox.y().whileTrue(
        Commands.defer(
            () -> CoralPickupCommand.pickupRight(drivebase, m_elevatorSubSystem),
            Set.of(drivebase, m_elevatorSubSystem)
        )
      );
      driverXbox.back().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      // //selectXbox
      // selXbox.a().onTrue(Commands.runOnce(() -> {
      //   SmartDashboard.putString("testingAsel","pressed");
      //   sel.selectSide(ReefScoreCommand.ReefSide.LEFT);
      // }));
      // driverXbox.a().whileTrue(ReefScoreCommand.scoreL3Right(drivebase, m_elevatorSubSystem, outtakeWithSensor));
      driverXbox.a().whileTrue(
            Commands.defer(
                () -> {
                    var level = sel.level();
                    var side  = ReefSide.LEFT;
                    return ReefScoreCommand.score(
                        drivebase, m_elevatorSubSystem, outtakeWithSensor.asProxy(), level, side
                    );
                },
                Set.of(drivebase, m_elevatorSubSystem)
            )
        );

      driverXbox.b().whileTrue(
            Commands.defer(
                () -> {
                    var level = sel.level();
                    var side  = ReefSide.RIGHT;
                    return ReefScoreCommand.score(
                        drivebase, m_elevatorSubSystem, outtakeWithSensor.asProxy(), level, side
                    );
                },
                Set.of(drivebase, m_elevatorSubSystem)
            )
        );
    
      // selectXbox
      // selXbox.a().onTrue(Commands.runOnce(() -> {
      //   sel.selectSide(ReefScoreCommand.ReefSide.LEFT);
      // }));
      
      // selXbox.b().onTrue(Commands.runOnce(() -> {
      //   sel.selectSide(ReefScoreCommand.ReefSide.RIGHT);
      // }));

      selXbox.povUp().onTrue(Commands.runOnce(() -> {
        sel.selectLevel(Setpoint.kLevel4);
      }));
      selXbox.povLeft().onTrue(Commands.runOnce(() -> {
        sel.selectLevel(Setpoint.kLevel2);
      }));
      selXbox.povDown().onTrue(Commands.runOnce(() -> {
        sel.selectLevel(Setpoint.kLevel3);
      }));



      // Auxillary Controller
      auxXbox.a().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel1));
      auxXbox.b().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel2));
      auxXbox.y().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel3));
      auxXbox.x().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel4));
      auxXbox.rightBumper().onTrue(outtakeWithSensor.asProxy());
      auxXbox.leftBumper().onTrue(intakeWithSensor);
      auxXbox.povDown().whileTrue(runHingeReverse);
      auxXbox.povUp().whileTrue(runHingeForward);
      // auxXbox.start().onTrue(m_hingeSubSystem.setSetpointCommand(HingeSubsystem.Setpoint.kLevel4));
      auxXbox.axisMagnitudeGreaterThan(1, 0.2).whileTrue(runElevator);
      auxXbox.axisMagnitudeGreaterThan(5, 0.2).whileTrue(runOuttake);
      auxXbox.axisMagnitudeGreaterThan(3, 0.2).onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kFeederStation));
      auxXbox.axisMagnitudeGreaterThan(2, 0.2).onTrue(Commands.runOnce(() -> {
        sel.nextCycleLevel();}));

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

  public static class ScoreSelection {
    private final ElevatorSubsystem.Setpoint[] levels = {
        ElevatorSubsystem.Setpoint.kLevel2,
        ElevatorSubsystem.Setpoint.kLevel3,
        ElevatorSubsystem.Setpoint.kLevel4
    };
    private int idx = 1;
    private ReefScoreCommand.ReefSide side = ReefScoreCommand.ReefSide.RIGHT;
    // how to create a hashmap of levels to commands
    
    // HashMap<Pair<ElevatorSubsystem.Setpoint, ReefScoreCommand.ReefSide>, Command> commandMap = new HashMap<>();


    // public void generateAllCommands(SwerveSubsystem drivebase, ElevatorSubsystem elevator,
    //     Command outtakeWithSensor) {
    //   for (ElevatorSubsystem.Setpoint level : levels) {
    //     for (ReefScoreCommand.ReefSide side : ReefScoreCommand.ReefSide.values()) {
    //       commandMap.put(new Pair<>(level, side),
    //           ReefScoreCommand.score(drivebase, elevator, outtakeWithSensor, level, side));
    //     }
    //   }
    // }

    // public Command getCommand(ElevatorSubsystem.Setpoint level, ReefScoreCommand.ReefSide side) {
    //   return commandMap.get(new Pair<>(level, side));
    // }

    public synchronized void cycleLevel(int dir) {
      idx = Math.floorMod(idx + dir, levels.length);
      SmartDashboard.putString("Score/Level", levels[idx].name());
    }

    public synchronized void nextCycleLevel() {
      cycleLevel(+1);
    }

    public synchronized void selectLevelandSide(ElevatorSubsystem.Setpoint level,
        ReefScoreCommand.ReefSide side) {
      for (int i = 0; i < levels.length; i++) {
        if (levels[i] == level) {
          idx = i;
          this.side = side;
          SmartDashboard.putString("Score/Level", levels[idx].name());
          SmartDashboard.putString("Score/Side", side.name());
          return;
        }
      }
      throw new IllegalArgumentException("Unknown level " + level);
    }

    public synchronized void toggleSide() {
      side = (side == ReefScoreCommand.ReefSide.LEFT)
          ? ReefScoreCommand.ReefSide.RIGHT
          : ReefScoreCommand.ReefSide.LEFT;
      SmartDashboard.putString("Score/Side", side.name());
    }

    public synchronized void selectSide(ReefScoreCommand.ReefSide side){
      this.side = side; 
    }

    public synchronized void selectLevel(ElevatorSubsystem.Setpoint level){
      for (int i = 0; i < levels.length; i++) {
        if (levels[i] == level) {
          idx = i;
          SmartDashboard.putString("Score/Level", levels[idx].name());
          return;
        }
      }
      throw new IllegalArgumentException("Unknown level " + level);
    }

    public synchronized ElevatorSubsystem.Setpoint level() {
      return levels[idx];
    }

    public synchronized ReefScoreCommand.ReefSide side() {
      return side;
    }

    //build a generate function that returns 
  }

}
