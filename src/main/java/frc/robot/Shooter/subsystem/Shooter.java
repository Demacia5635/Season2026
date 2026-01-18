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

<<<<<<< HEAD
=======
  //hub pose (i finde it with april tag)
  public Pose2d hubPose(){
    Optional<Pose3d> getTagPose3d = apriTagfFieldLayout.getTagPose(2);
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



  public double getAngleFromeShottrToHub(){
    Translation2d tagToRobot = tag.getRobotToTagRR();
    Optional<Pose3d> TagPoseOptionalPose3d = apriTagfFieldLayout.getTagPose(2);
    Pose2d TagPosePose2d = TagPoseOptionalPose3d.get().toPose2d();
    Translation2d TagTohub = new Translation2d(ShooterUtils.distensFromToPose2dPoint(TagPosePose2d, hubPose()), ShooterUtils.angle_betuenTowPose2d(TagPosePose2d, hubPose()));
    Translation2d ShooterToHub = getVectorToTargetShoter().toTranslation2d();
    Translation2d shooterToMidelOFHub = TagTohub.plus(ShooterToHub);
    return shooterToMidelOFHub.getAngle().getDegrees();

  }

>>>>>>> parent of 916f506 (shit)
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
