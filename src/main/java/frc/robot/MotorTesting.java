// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;

public class MotorTesting extends SubsystemBase {
  /** Creates a new MotorTesting. */
  TalonFXMotor motor;
  public MotorTesting() {
    motor = new TalonFXMotor(motorConstants.frontLeftMotorConfig);
    SmartDashboard.putData("Motor Testing", this);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("Motor Current", this::getMotorCurrent, null);
      builder.addBooleanProperty("Motor Stall", this::getMotorStall, null);

  }
  public double getMotorCurrent() {
    return motor.getCurrentCurrent();
  }
  public void setMotorStall() {
    motor.updateStallDetection();
  }
  public boolean getMotorStall() {
    return motor.getStallDetection();
  } 
  public void setMotorPower(double power) {
    motor.set(power);
  }
  public void stopMotor() {
    motor.stopMotor();
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motor.updateStallDetection();
  }
}
