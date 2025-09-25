package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants.ElevatorSetpoints;
import org.littletonrobotics.junction.Logger;


public class ElevatorSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel1AndAHalf,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorFollowerMotor =
  new SparkMax(ElevatorSubsystemConstants.kElevatorFollowerMotorCanId, MotorType.kBrushless);
  private SparkMax elevatorMotor = 
      new SparkMax(ElevatorSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  // private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.



  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;

 


  public ElevatorSubsystem() {
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
          
    elevatorMotor.configure(
        Configs.ElevatorSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorFollowerMotor.configure(
        Configs.ElevatorSubsystem.elevatorConfig.follow(ElevatorSubsystemConstants.kElevatorMotorCanId,true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Display mechanism2d


    // Zero arm and elevator encoders on initialization
    elevatorEncoder.setPosition(0);

  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  public void moveToSetpoint() {
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ElevatorSubsystemConstants.kArbFF);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }



  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
             
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case kLevel1:
            
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel1AndAHalf:
            
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1AndAHalf;
              break;
            case kLevel2:
              
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:

              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
          
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
        });
  }

 
  @Override
  public void periodic() {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();
    Logger.recordOutput("Odometry/Elevator/Target", elevatorCurrentTarget);
    Logger.recordOutput("Odometry/Elevator/Actual", elevatorEncoder.getPosition());



    // Display subsystem values

    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Elevator/Current", getElevatorCurrent());


  }
  /** Get the current drawn by each simulation physics model */
  public double getElevatorCurrent(){
    return elevatorMotor.getOutputCurrent();
  }
  public double getElevatorEncoder() {
    return elevatorEncoder.getPosition();
    //return elevatorMotor.getEncoder().getPosition();
  }

  public void setElevatorPower(double speed){
    elevatorMotor.set(speed);
  }

  public void syncElevatorControl(){
    elevatorCurrentTarget = elevatorEncoder.getPosition();
    moveToSetpoint();
  }
  public void setElevatorCurrentTarget(double currentTarget){
    elevatorCurrentTarget = currentTarget;
  }
}