// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.vision.subsystem.ObjectPose;
import frc.robot.intake.subsystem.IntakeSubsystem;



public class IntakeAutonamusVelocities extends Command {
  private Chassis chassis;
  private IntakeSubsystem intake;
  private ObjectPose objectPose;
  private ChassisSpeeds speeds ;
  private Rotation2d angleError;
  private double omega;
  private Translation2d objectCurrTarget;
  private Translation2d currTransltion;
  private double speedx;
  private double speedy;
  
    // Fuel position from vision

  public IntakeAutonamusVelocities(Chassis chassis, IntakeSubsystem intake, ObjectPose objectPose) {
    this.chassis = chassis;
    this.intake = intake;
    this.objectPose = objectPose;
    addRequirements(chassis, intake);
  }

  @Override
  public void initialize() {
    objectCurrTarget = objectPose.getRobotToObject();
    angleError = chassis.getPose().getRotation().minus(objectCurrTarget.getAngle());
    
    speedx = Math.cos(objectCurrTarget.getAngle().getRadians());
    speedy = Math.sin(objectCurrTarget.getAngle().getRadians());
    
    omega = angleError.getRadians() / (objectPose.getDistcameraToObject() / Math.sqrt(speedx * speedx + speedy * speedy));

    speeds = new ChassisSpeeds(speedx, speedy, omega);
    chassis.setVelocities(speeds);
  //   toTarget = objectPose.getRobotToObject();
  //   distance = objectPose.getDistcameraToObject();
  //   System.out.println("Distance: " + distance);
  //   targetPose = new Pose2d(toTarget.rotateBy(chassis.getGyroAngle()).plus(chassis.getPose().getTranslation()), new Rotation2d(0));
  //   omega = toTarget.getAngle().getRadians() / (distance / toTarget.getY());
  //   speeds = new ChassisSpeeds(toTarget.getX(), toTarget.getY(), 0);
    

  //  chassis.setVelocitiesRotateToTarget(speeds,targetPose);
  }

  @Override
  public void execute() {
    intake.setDutyIntake(0.8);
    // intake.setDutyConveyorBelt(0.8);
    
    // if (Math.abs(omega) < 0.06) omega = 0;
    // omega *= -2;

    // speeds = new ChassisSpeeds(-1, 0, omega);
    // chassis.setRobotRelVelocities(speeds);

    currTransltion = chassis.getPose().getTranslation();
    if (objectCurrTarget.minus(objectPose.getRobotToObject().plus(currTransltion)).getNorm() < 0.3){

      objectCurrTarget = objectPose.getRobotToObject();
      angleError = chassis.getPose().getRotation().minus(objectCurrTarget.getAngle());
      omega = angleError.getRadians() / (objectPose.getDistcameraToObject() / objectCurrTarget.getY());

      speedx = Math.cos(objectCurrTarget.getAngle().getRadians());
      speedy = -Math.sin(objectCurrTarget.getAngle().getRadians());

      speeds = new ChassisSpeeds(speedy, speedx, omega);
      chassis.setVelocities(speeds);
      System.out.println("Changed target position");
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("IntakeAutonamusVelocities.end()");
    chassis.stop();
    intake.setDutyIntake(0);
  }

  @Override
  public boolean isFinished() {
    return (objectPose.getDistcameraToObject() < 0.3 );
  }
}