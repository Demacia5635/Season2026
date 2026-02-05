// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFXMotor motorIntake;
  // private TalonSRXMotor motorRoller;
  // private TalonSRXMotor motorToShooter;

  public IntakeSubsystem() {
    motorIntake = new TalonFXMotor(intakeConstants.INTAKE_CONFIG);
    // motorRoller = new TalonSRXMotor(intakeConstans.ROLLER_CONFIG);
    // motorToShooter = new TalonSRXMotor(intakeConstans.TO_SHOOTER_CONFIG);
  }

  // public void setDutyConveyorBelt(double pow){
  // motorRoller.setDuty(pow);
  // motorToShooter.setDuty(pow);
  // }

  public void setDutyIntake(double pow) {
    motorIntake.setDuty(pow);
  }

  public void stopIntake() {
    motorIntake.stop();
  }

  // public void stopRoller(double pow){
  // motorRoller.stop();
  // }

  // public void stopToShooter(){
  // motorToShooter.stop();
  // }
}
