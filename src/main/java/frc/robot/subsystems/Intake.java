// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OperatorConstants;

// public class Intake extends SubsystemBase {
//   /** Creates a new ExampleSubsystem. */

//   private Rev2mDistanceSensor distSens;

//   private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(OperatorConstants.ID_INTAKE);
//   public Intake() {
//         intakeMotor.setInverted(true);
//         intakeMotor.setNeutralMode(NeutralMode.Brake);

//         //distSens = new Rev2mDistanceSensor(Port.kOnboard);
//         distSens = new Rev2mDistanceSensor(Port.kOnboard);

//         distSens.setAutomaticMode(true);
//         distSens.setEnabled(true);

//         distSens.setRangeProfile(RangeProfile.kDefault);
        
//   }

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command getDistanceCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
     
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     boolean isValid = distSens.isRangeValid();
//     SmartDashboard.putBoolean("Valid", isValid);
//     if(isValid) {
//       SmartDashboard.putNumber("Range", distSens.getRange());
//       SmartDashboard.putNumber("Timestamp", distSens.getTimestamp());
//     }
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }

//   public void runIntake(double speed){
//     intakeMotor.set(speed);
// }
// public void stopIntake(){
//     intakeMotor.set(0);
// }
// }
