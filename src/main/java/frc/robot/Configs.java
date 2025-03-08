package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;




public final class Configs {
 


  public static final class ElevatorSubsystem {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();


    static {

  
      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12).inverted(true);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      elevatorConfig
          .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          //TODO: Set P value empirically
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);

      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    }
  }



  public static final class HingeSubsystem {
    public static final SparkMaxConfig hingeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the arm motor
    hingeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).inverted(true);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      hingeConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

  
    }
  }
  public static final class HangSubsystem {
    public static final SparkMaxConfig hangConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the arm motor
    hangConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).inverted(true
    );

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      hangConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

  
    }
  }
}