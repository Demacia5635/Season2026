// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;
import frc.demacia.vision.subsystem.ObjectPose;
import frc.robot.intake.subsystem.IntakeSubsystem;



public class IntakeAutonamusVelocities extends Command {
  private Chassis chassis;
  private IntakeSubsystem intake;
  private ObjectPose objectPose;
  private ChassisSpeeds speeds ;
  private double omega;
  
    // Fuel position from vision



  public IntakeAutonamusVelocities(Chassis chassis, IntakeSubsystem intake, ObjectPose objectPose) {
    this.chassis = chassis;
    this.intake = intake;
    this.objectPose = objectPose;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setDutyIntake(0.8);
    omega = objectPose.getRobotToObject().getAngle().getRadians();
    if (Math.abs(omega) < 0.06) omega = 0;
    omega *= 2;
    speeds = new ChassisSpeeds(-objectPose.getY(), objectPose.getX(), omega);
    LogManager.log("Omega: " + omega + "/nAngle to Object: " + objectPose.getRobotToObject().getAngle().getRadians() + "/n Distance: " + objectPose.getDistcameraToObject());
    chassis.setRobotRelVelocities(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    LogManager.log("IntakeAutonamusVelocities.end()");
    chassis.stop();
    intake.setDutyIntake(0);
  }

  @Override
  public boolean isFinished() {
    return (objectPose.getDistcameraToObject() < 0.3 );
  }
}