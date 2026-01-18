// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.vision.subsystem.Tag;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.utils.ShooterUtils;


public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  //april tag field layout
  AprilTagFieldLayout aprilTagFieldLayout;

  //motors
  TalonFXMotor shooterMotor;
  TalonFXMotor indexerMotor;

  //variables
  public double VelocityInFucer;

  //abicted i use
  Chassis chassis;
  Tag tag;

  //cunstroctor 
  public Shooter(Chassis chassis, Tag tag) {
    this.chassis = chassis;
    this.tag = tag;
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    indexerMotor = new TalonFXMotor(ShooterConstans.INDEXER_CONFIG);
  }

  //get the pitch angle
  public double getAngle(){
    //TODO: cange to the angle
    return 0;
  }

  //get the yow angle
  public double getTurretAngle(){
    return chassis.getGyroAngle().getDegrees(); // for the tsting the tow angle is the chassis angle
  }

  public void setSpeed(double speed){
    shooterMotor.setVelocity(speed); //set the shooter motor speed in RPS
  }


  //get the shooter velocity in RPS
  public double getShooterVelocity(){
    return shooterMotor.getVelocity().getValueAsDouble();
  }
  
  //set the shooter motor power from 0 to 1
  public void setPower(double power){
    shooterMotor.set(power);
  }

  //get witch velocity to set from witch distance from the look up table
  public double getVelFromLookUpTable(double destins){
    double angle =ShooterConstans.SHOOTER_LOOKUP_TABLE.get(destins)[0];
    return angle;
  }

  //get witch angle to set from witch distance from the look up table
  public double getAnglFromLookUpTable(double destins){
      double velocity = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(destins)[1];
      return velocity;
  }


  //get vector of the shooter while moveng in the fucer 
  public Translation3d getShooterFucerMoveng() {
  return new  Translation3d(VelocityInFucer, 
      new Rotation3d(0, getAngle(), getTurretAngle())
    );
  }

  //set the value of the velocity in the fucer so you can get it from other classes
  public void setVelocityInTheFucer(double vel){
    VelocityInFucer = vel;
  }

 // get the value of the velocity in the fucer so you can get it from other classes 
  public double getVelocityInFucer(){
    return VelocityInFucer;
  }
  
  //set the indexer motor power from 0 to 1
  public void setIndexerPower(double pow){
    indexerMotor.set(pow);
  }

  //check if the shooter is ready to shoot
  public boolean isShooterReady(){
    return Math.abs(shooterMotor.getClosedLoopError().getValueAsDouble()) < 0.2;
  }
  
  //stop the shooter motor
  public void stop(){
    shooterMotor.stopMotor();
  }

  //hub pose (i finde it with april tag)
  public Pose2d hubPose(){
    Optional<Pose3d> getTagPose3d = aprilTagFieldLayout.getTagPose(2);
    Pose2d getTagPose2d = getTagPose3d.get().toPose2d();
    double hubMideldestinseFromeTag;
    Rotation2d hubAngleFromeTag;
    Pose2d calclateHubPose = ShooterUtils.calculatePoseWithTransform(getTagPose2d, hubMideldestinseFromeTag, hubAngleFromeTag, new Rotation2d());
    return calclateHubPose;
  }

  //shooter pose on the robot
  public Pose2d ShooterPoseOnRobot(){
    return new Pose2d();
  }

  //get the distins from the shooter to the target
  public Translation3d getVectorToTargetShoter(){
    Translation3d robotToTarget = new Translation3d(
     ShooterUtils.distensFromToPose2dPoint(chassis.getPose(), hubPose())
      , ShooterUtils.angle_betuenTowPose2d(chassis.getPose(), hubPose()), 0);

      Translation3d shooterToTarget = robotToTarget.minus(
        new Translation3d(
          ShooterPoseOnRobot().getX(),
          ShooterPoseOnRobot().getY(),
          0
        )
      );
    return shooterToTarget;
  }



  public double getTheAngleToTheHubFromTurret(){
    Translation2d tagToRobot = tag.getRobotToTagRR();
    Optional<Pose3d> TagPoseOptionalPose3d = aprilTagFieldLayout.getTagPose(2).get();
    Pose2d TagPoseOptionalPose2d = TagPoseOptionalPose3d.get();
    Translation2d TagTohub = new Translation2d(ShooterUtils.distensFromToPose2dPoint(TagPose, hubPose()), ShooterUtils.angle_betuenTowPose2d(TagPose, hubPose()));
    Translation2d ShooterToHub = getVectorToTargetShoter().toTranslation2d();
    Translation2d shooterToMidelOFHub = TagTohub.plus(ShooterToHub);
    return shooterToMidelOFHub.getAngle().getDegrees();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
