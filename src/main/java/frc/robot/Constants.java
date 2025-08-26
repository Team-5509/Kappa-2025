// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int BLINKIN_LED_CONTROLLER_PORT = 0;
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ElevatorSubsystemConstants {
    
    public static final int kElevatorMotorCanId = 13;
    public static final int kElevatorFollowerMotorCanId = 14;
    public static final double kArbFF = 0.4;
    
    public static final class ElevatorSetpoints {
      // TODO fix these values to the actual Reef levels
      public static final double kFeederStation = 4.7;
      public static final double kLevel1 = 3;
      public static final double kLevel1AndAHalf = 10;
      public static final double kLevel2 = 14.5;
      public static final double kLevel3 = 32;
      public static final int kLevel4 = 61;
    
      }
  }
  public static final class IntakeSubsystemConstants {

    public static final int kIntakeMotorCanId = 15;
    public static final int kIntakeFollowerMotorCanId = 16;
    public static final int KIntakeInputDigitalIO = 9; 
    public static final int KOuttakeInputDigitalIO = 8; 

    public static final class IntakeSetpoints {
      public static final double kForward = 1;
      public static final double kReverse = -0.5;
    }
  }

  public static final class HangSubsystemConstants {
    
    public static final int kHangMotorCanId = 18;
  
  

    public static final class HangSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 30;
    }
  }
  public static final class HingeSubsystemConstants {

    
    public static final int kHingeMotorCanId = 17;
  

    public static final class HingeSetpoints {
      // TODO fix these values to the actual Reef levels
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 10;
      public static final int kLevel3 = 10;
      public static final int kLevel4 = 35;
      }

      
  }
 

  public static final double ROBOT_MASS = (130) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(40);


  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass = 4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads = Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads = Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }


  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final double SPEED_MAXIMUM_FACTOR = .8;
    public static final double SPEED_MINIMUM_FACTOR = .45;


  }
}
