// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;

public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  TalonFXMotor shooterMotor;

  public Shooter() {
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    
  }

  public void setSpeed(double speed){
    shooterMotor.setVelocity(speed);
  }
  public void setPower(double power){
    shooterMotor.set(power);
  }

  
  public void stop(){
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
