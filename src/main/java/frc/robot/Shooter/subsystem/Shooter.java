// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.Shooter.ShooterConstans;


public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  TalonFXMotor shooterMotor;
  TalonFXMotor indexerMotor;
  public double VelocityInFucer;
  
  public Shooter() {
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    indexerMotor = new TalonFXMotor(ShooterConstans.INDEXER_CONFIG);
  }


  public void setSpeed(double speed){
    shooterMotor.setVelocity(speed);
  }
  public double getShooterVelocity(){
    return shooterMotor.getVelocity().getValueAsDouble();
  }
  
  public void setPower(double power){
    shooterMotor.set(power);
  }

  
  public void setVelocitiesAndAngle(double[] VelocityAndAngle){
    setSpeed(VelocityAndAngle[0]);
  }

  public void setShppterAngle(){

  }

  public void setVelocityInTheFucer(double vel){
    VelocityInFucer = vel;
  }

  public double getVelocityInFucer(){
    return VelocityInFucer;
  }
  

  public void setIndexerPower(double pow){
    indexerMotor.set(pow);
  }

  public boolean isShooterReady(){
    return Math.abs(shooterMotor.getClosedLoopError().getValueAsDouble()) < 0.2;
  }
  
  public void stop(){
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
