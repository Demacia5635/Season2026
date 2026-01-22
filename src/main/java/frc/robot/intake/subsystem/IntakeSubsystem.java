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
  TalonFXMotor motor;

  TalonSRXMotor motorShinoa;

  TalonSRXMotor motorShit;

  public IntakeSubsystem() {
    motor = new TalonFXMotor(intakeConstans.INTAKE_CONFIG);
    motorShinoa = new TalonSRXMotor(intakeConstans.SHINA_CONFIG);
    motorShit = new TalonSRXMotor(intakeConstans.SHIT_CONFIG);
  }

  public void start() {
    motor.setDuty(intakeConstans.MAX_POWER);
  }

  public void setDute(double pow){
    motorShinoa.setDuty(pow);
  }

  public void setDuteShit(double pow){
    motorShit.setDuty(pow);
  }

  public void stop(){
    motor.setDuty(0);
  }
}
