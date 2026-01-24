// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.vision.subsystem.ObjectPose;


public class IntakeAutonamusVelocities extends Command {

  private final Chassis chassis;
  private final ObjectPose objectPose;  // Fuel position from vision


  private int lostFrames = 0;
  private double distance = 0;

  public IntakeAutonamusVelocities(Chassis chassis, ObjectPose objectPose) {
    this.chassis = chassis;
    this.objectPose = objectPose;
    addRequirements(chassis);
  }


  @Override
  public void initialize() {
    lostFrames = 0;
  }

  @Override
  public void execute() {

    Pose2d targetPose = objectPose.getPose2d();  // Get fuel position
    



    Pose2d robotPose = chassis.getPose();

    Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());

      distance = toTarget.getNorm();
     

   
   
   ChassisSpeeds speeds = new ChassisSpeeds(toTarget.getX(), toTarget.getY(), toTarget.getAngle().getRadians());
   chassis.setVelocitiesRotateToTarget(speeds,targetPose);

  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    if (distance < 10 ) {
      return true;
    }
    return false;
  }
}