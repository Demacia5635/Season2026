// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.utils.ShooterUtils;


public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  TalonFXMotor shooterMotor;
  TalonFXMotor indexerMotor;
  public double VelocityInFucer;
  Chassis chassis;

  public Shooter(Chassis chassis) {
    this.chassis = chassis;
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

  //hub pose (i finde it with april tag)
  public Translation3d hubPose(){
    return new Translation3d(449.5/100, 370.84000000000003/100, 142.24/2);
  }

  //shooter pose on the robot
  public Translation3d ShooterPoseOnRobot(){
    return new Translation3d();
  }
  public Translation3d getShooterPosOnField(){
    return new Translation3d();
  }

  //get the distins from the shooter to the target
  public Translation3d getVectorToTargetShoter(){
    return hubPose().minus(getShooterPosOnField());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
