// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import java.security.PublicKey;

import edu.wpi.first.math.MathUtil;
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
  TalonFXMotor hoodMotor;
  public double VelocityInFucer;
  Chassis chassis;

  public Shooter(Chassis chassis) {
    hoodMotor = new TalonFXMotor(ShooterConstans.HOOD);
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


  public void setAngleHood(double angle){
    MathUtil.clamp(angle, ShooterConstans.MIN_ANGLE_HOOD, ShooterConstans.MAX_ANGLE_HOOD);
    hoodMotor.setPositionVoltage(angle);
  }

  public double getAngleHood(){
    return hoodMotor.getCurrentAngle();
  }
  
  public void setVelocitiesAndAngle(double vel, double angle){
    setSpeed(vel);
    setAngleHood(angle);
  }


  public void setVelocityInTheFucer(double vel){
    VelocityInFucer = vel;
  }

  public double getLookUpTableVel(double distance){
    return ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance)[0];
  }

  public double getLookUpTableAngle(double distance){
    return ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance)[1];
  }

  

  public double getVelocityInFucer(){
    return VelocityInFucer;
  }
  
  public Translation3d getVelInVector(double vel){
    return new Translation3d(vel, new Rotation3d(chassis.getGyroAngle().getDegrees(), getAngleHood(), 0));
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
  public Translation3d getVectorToHubShoter(){
    return ShooterConstans.hubPose.minus(getShooterPosOnField());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
