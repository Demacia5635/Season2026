// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;

public class BatteryLid extends SubsystemBase {
  /** Creates a new BatteryLid. */
  TalonFXMotor motor;
  double MAX_POSITION = 4.2;

  public BatteryLid() {
    this.motor = new TalonFXMotor(new TalonFXConfig(24, Canbus.Rio, "Battery Motor").withBrake(true).withInvert(false).withPID(20, 0, 0, 0, 0, 0, 0).withRadiansMotor(64));
  
    motor.configPidFf(0);

  }

  public void setEncoderPosition(double position){
    motor.setEncoderPosition(position);
  }
  public void setPosition(double position){
    motor.setPositionVoltage(position);
  }
  public void setPower(double power){
    motor.set(power);
  }
  public boolean isAtMin(double angleError){
    return Math.abs(motor.getCurrentPosition()) < angleError;
  }
  public boolean isAtMax(double angleError){
    return Math.abs(MAX_POSITION - motor.getCurrentPosition()) < angleError;
  }
  @Override
  public void periodic() {
  }
}
