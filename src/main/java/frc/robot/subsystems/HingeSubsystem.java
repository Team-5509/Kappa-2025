package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.HingeSubsystemConstants;
import frc.robot.Constants.HingeSubsystemConstants.HingeSetpoints;


public class HingeSubsystem extends SubsystemBase {
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
  
  private SparkMax hingeMotor =
      new SparkMax(HingeSubsystemConstants.kHingeMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController hingeClosedLoopController =
      hingeMotor.getClosedLoopController();
  private RelativeEncoder hingeEncoder = hingeMotor.getEncoder();



  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double hingeCurrentTarget = HingeSetpoints.kFeederStation;

 


  public HingeSubsystem() {
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
    hingeMotor.configure(
        Configs.HingeSubsystem.hingeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
   

    // Display mechanism2d


    // Zero arm and elevator encoders on initialization
    hingeEncoder.setPosition(0);

  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    hingeClosedLoopController.setReference(
       hingeCurrentTarget, ControlType.kMAXMotionPositionControl);
  }
public void syncHingeControl(){
  hingeCurrentTarget = hingeEncoder.getPosition();
  moveToSetpoint();
}
  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroHingeOnLimitSwitch() {
    if (!wasResetByLimit && hingeMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      hingeEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!hingeMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      hingeEncoder.setPosition(0);
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
             
              hingeCurrentTarget = HingeSetpoints.kFeederStation;
              break;
            case kLevel1:
            
              hingeCurrentTarget = HingeSetpoints.kLevel1;
              break;
            case kLevel2:
              
              hingeCurrentTarget = HingeSetpoints.kLevel2;
              break;
            case kLevel3:

              hingeCurrentTarget = HingeSetpoints.kLevel3;
              break;
            case kLevel4:
          
              hingeCurrentTarget = HingeSetpoints.kLevel4;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    zeroHingeOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values

    SmartDashboard.putNumber("Hinge/Target Position", hingeCurrentTarget);
    SmartDashboard.putNumber("Hinge/Actual Position", hingeEncoder.getPosition());

  }
  /** Get the current drawn by each simulation physics model */
  public void setHingePower(double speed){
    hingeMotor.set(speed);
  }
}