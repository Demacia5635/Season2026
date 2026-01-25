// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.subsystem;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFXMotor motorIntake;

  public IntakeSubsystem() {
    motorIntake = new TalonFXMotor(new TalonFXConfig(20, Canbus.Rio, "intake motor"));
  }

  public void setDutyIntake(double pow){
    motorIntake.setDuty(pow);
  }
}