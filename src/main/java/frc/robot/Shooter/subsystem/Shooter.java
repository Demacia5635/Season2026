// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.utils.ShooterUtils;


public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  //motors
  TalonFXMotor shooterMotor;
  TalonFXMotor indexerMotor;

  //variables
  public double VelocityInFucer;

  //abicted i use
  Chassis chassis;

  //cunstroctor 
  public Shooter(Chassis chassis) {
    this.chassis = chassis;
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    indexerMotor = new TalonFXMotor(ShooterConstans.INDEXER_CONFIG);
  }


  public double getAngle(){
    //TODO: cange to the angle
    return 0;
  }

  public double getTurretAngle(){
    //TODO:CANGE TO THE TURRET ANGLE
    return chassis.getGyroAngle().getDegrees(); // for the tsting the tow angle is the chassis angle
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

  public double getVelFromLookUpTable(double destins){
    double angle =ShooterConstans.SHOOTER_LOOKUP_TABLE.get(destins)[0];
    return angle;
  }

  public double getAnglFromLookUpTable(double destins){
      double velocity = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(destins)[1];
      return velocity;
  }

  public Translation3d getShooterFucerMoveng() {
  return new  Translation3d(VelocityInFucer, 
      new Rotation3d(0, getAngle(), getTurretAngle())
    );
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

  public Pose2d targetPose(){
    //TODEO: FINALE THE DISTINS TO THE TARGET
    return new Pose2d();
  }

  public Pose2d ShooterPoseOnRobot(){
    //TODEO: FINALE THE SHOOTER POSE ON THE ROBOT
    return new Pose2d();
  }

  public double getDistToTargetShoter(){
    Translation3d robotToTarget = new Translation3d(
      ShooterUtils.distensFromToPose2dPoint(chassis.getPose(), targetPose())
      , ShooterUtils.angle_betuenTowPose2d(chassis.getPose(), targetPose()), 0);

      Translation3d shooterToTarget = robotToTarget.minus(
        new Translation3d(
          ShooterPoseOnRobot().getX(),
          ShooterPoseOnRobot().getY(),
          0
        )
      );
    return shooterToTarget.getNorm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
