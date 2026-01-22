// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.subsystem;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.robot.intake.intakeConstans;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intake. */
  TalonFXMotor motorIntake;

  TalonSRXMotor motorRoller;

  TalonSRXMotor motorToShooter;

  public IntakeSubsystem() {
    motorIntake = new TalonFXMotor(intakeConstans.INTAKE_CONFIG);
    motorRoller = new TalonSRXMotor(intakeConstans.ROLLER_CONFIG);
    motorToShooter = new TalonSRXMotor(intakeConstans.TO_SHOOTER_CONFIG);
  }


  public void setDuteIntake(double pow){
    motorIntake.setDuty(pow);
  }

  public void setduteRoller(double pow){
    motorRoller.setDuty(pow);
  }

  public void setDuteToShooter(double pow){
    motorToShooter.setDuty(pow);
  }

  public void stopIntake(){
    motorIntake.stop();
  }

  public void stopRoller(double pow){
    motorRoller.stop();
  }

  public void stopToShooter(){
    motorToShooter.stop();
  }
}
