package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.HangSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants;


public class IntakeSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.

  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkMax intakeMotor =
      new SparkMax(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  private DigitalInput intakeInput = 
      new DigitalInput(IntakeSubsystemConstants.KIntakeInputDigitalIO);

      private DigitalInput outtakeInput = 
      new DigitalInput(IntakeSubsystemConstants.KOuttakeInputDigitalIO);

            private SparkMax intakeFollowerMotor =
      new SparkMax(IntakeSubsystemConstants.kIntakeFollowerMotorCanId, MotorType.kBrushless);

  private SparkClosedLoopController intakeClosedLoopController =
      intakeMotor.getClosedLoopController();

      private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

      private RelativeEncoder intakeFollowerEncoder = intakeFollowerMotor.getEncoder();


  
      public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();



  public IntakeSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
 

    intakeMotor.configure(
        Configs.ElevatorSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        intakeFollowerMotor.configure(
          Configs.ElevatorSubsystem.intakeConfig.follow(IntakeSubsystemConstants.kIntakeMotorCanId, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        
 

    // Display mechanism2d



  }






  /** Set the intake motor power in the range of [-1, 1]. */
  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));

  }

  public Boolean getIntakeInput() {
    return intakeInput.get();
  }
  
  public Boolean getOuttakeInput() {
    return outtakeInput.get();
  }
  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
  }

  @Override
  public void periodic() {
    

    // Display subsystem values

  
    SmartDashboard.putBoolean("Coral/Intake/intake input", getIntakeInput());
    SmartDashboard.putBoolean("Coral/Intake/outtake input", getOuttakeInput());
    SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());

  }
  /** Get the current drawn by each simulation physics model */

}