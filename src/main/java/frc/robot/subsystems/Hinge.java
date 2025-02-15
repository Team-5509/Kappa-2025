// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.relation.Relation;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HingeConstants;

public class Hinge extends SubsystemBase {

private SparkMax spark;
private SparkMaxConfig config;
SparkClosedLoopController m_controller;

  /** Creates a new ExampleSubsystem. */
  public Hinge() {

    spark = new SparkMax(16, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    spark.configure(config,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
    
    // Set PID gains
    config.closedLoop
    .p(HingeConstants.kP)
    .i(HingeConstants.kI)
    .d(HingeConstants.kD)
    .outputRange(HingeConstants.kMinOutput,HingeConstants.kMaxOutput);

    // Set kFF
    //config.closedLoop.velocityFF(1/HingeConstants.Kv);

    // Set MAXMotion parameters
    config.closedLoop.maxMotion
      .maxVelocity(HingeConstants.maxVel)
      .maxAcceleration(HingeConstants.maxAccel)
      .allowedClosedLoopError(HingeConstants.allowedErr);

    // Initialize the closed loop controller
    m_controller = spark.getClosedLoopController();

  }

void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    
    // Don't persist parameters since it takes time and this change is temporary
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command hingeSetPointMethodCommand(double setPoint) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          // Set the setpoint of the PID controller in raw position mode
          m_controller.setReference(setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
