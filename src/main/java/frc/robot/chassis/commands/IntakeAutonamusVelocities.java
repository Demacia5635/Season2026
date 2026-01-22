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


  private static final double MAX_DRIVE_SPEED = 2.5;     // m/s
  private static final double MIN_DRIVE_SPEED = 0.4;     // m/s
  private static final double APPROACH_DISTANCE = 0.6;   // meters - distance to start slowing down for fuel
  private static final double FINISH_DISTANCE = 0.15;    // meters - distance to consider fuel collected

  private static final double MAX_OMEGA = 3.0;           // rad/s
  private static final double ANGLE_KP = 2.0;

  private static final int MAX_LOST_FRAMES = 10;         // max frames before stopping if fuel is lost


  private final Chassis chassis;
  private final ObjectPose objectPose;  // Fuel position from vision


  private int lostFrames = 0;


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

    if (targetPose.equals(Pose2d.kZero)) {
      lostFrames++;
      if (lostFrames > MAX_LOST_FRAMES) {
        chassis.stop();  // Stop if fuel not visible
      }
      return;
    }

    lostFrames = 0;

    Pose2d robotPose = chassis.getPose();

    Translation2d toTarget =
        targetPose.getTranslation().minus(robotPose.getTranslation());

    double distance = toTarget.getNorm();

    // Calculate linear speed based on distance to fuel
    double linearSpeed;
    if (distance > APPROACH_DISTANCE) {
      linearSpeed = MAX_DRIVE_SPEED;
    } else {
      // Slow down as we approach the fuel
      linearSpeed = MathUtil.clamp(
          (distance / APPROACH_DISTANCE) * MAX_DRIVE_SPEED,
          MIN_DRIVE_SPEED,
          MAX_DRIVE_SPEED);
    }

    Translation2d velocity =
        toTarget.div(distance).times(linearSpeed);

    // Calculate rotation to face the fuel
    Rotation2d desiredAngle = toTarget.getAngle();
    double angleError =
        desiredAngle.minus(robotPose.getRotation()).getRadians();

    double omega;
    if (distance > 0.05 && linearSpeed > 0.05) {
      double timeToTarget = distance / linearSpeed;
      omega = angleError / timeToTarget;
    } else {
      omega = ANGLE_KP * angleError;
    }

    omega = MathUtil.clamp(omega, -MAX_OMEGA, MAX_OMEGA);

    chassis.setVelocities(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(),
            velocity.getY(),
            omega,
            robotPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    Pose2d targetPose = objectPose.getPose2d();  // Get fuel position
    if (targetPose.equals(Pose2d.kZero)) {
      return false;  // Don't finish if fuel not visible
    }

    double distance =
        targetPose.getTranslation()
            .getDistance(chassis.getPose().getTranslation());

    return distance < FINISH_DISTANCE;  // Finish when close enough to fuel
  }
}