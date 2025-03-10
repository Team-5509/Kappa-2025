package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.HangSubsystemConstants;
import frc.robot.Constants.HangSubsystemConstants.HangSetpoints;


public class HangSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  
  private SparkMax hangMotor =
      new SparkMax(HangSubsystemConstants.kHangMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController hangClosedLoopController =
      hangMotor.getClosedLoopController();
  private RelativeEncoder hangEncoder = hangMotor.getEncoder();



  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double hangCurrentTarget = HangSetpoints.kFeederStation;

 


  public HangSubsystem() {
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
    hangMotor.configure(
        Configs.HangSubsystem.hangConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

 
   

    // Display mechanism2d


    // Zero arm and elevator encoders on initialization
    hangEncoder.setPosition(0);

  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    hangClosedLoopController.setReference(
       hangCurrentTarget, ControlType.kMAXMotionPositionControl);
  }
  public void syncHangControl(){
    hangCurrentTarget = hangEncoder.getPosition();
    moveToSetpoint();
  }
  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroHangOnLimitSwitch() {
    if (!wasResetByLimit && hangMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      hangEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!hangMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      hangEncoder.setPosition(0);
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
             
              hangCurrentTarget = HangSetpoints.kFeederStation;
              break;
            case kLevel1:
            
              hangCurrentTarget = HangSetpoints.kLevel1;
              break;
            case kLevel2:
              
              hangCurrentTarget = HangSetpoints.kLevel2;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    zeroHangOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values

    SmartDashboard.putNumber("Hang/Target Position", hangCurrentTarget);
    SmartDashboard.putNumber("Hang/Actual Position", hangEncoder.getPosition());
    SmartDashboard.putNumber("Hang/Current", getHangCurrent());

  }
  /** Get the current drawn by each simulation physics model */
  public void setHangPower(double speed){
    hangMotor.set(speed);
  }

  public double getHangCurrent(){
    return hangMotor.getOutputCurrent();
  }
}