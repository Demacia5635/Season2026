// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Shooter.utils.ShooterUtils;

public class ShootOnTheFlyCommand extends Command {

  public static final double WHEEL_TO_BALL_VELOCITY_RATIO = 0.45;
  public static final double HOOD_OFFSET = Math.toRadians(2);
  public static final double MAGNUS_CORRECTION = 0.2;
  
  Shooter shooter;
  Chassis chassis;

  public ShootOnTheFlyCommand(Shooter shooter, Chassis chassis) {
    this.chassis = chassis;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // calculate next pose and vector to HUB
    ChassisSpeeds robotSpeeds = chassis.getChassisSpeedsFieldRel();
    Pose2d nextPose = ShooterUtils.computeFuturePosition(robotSpeeds,chassis.getPose(),0.02);
    Translation2d toHub = ShooterConstans.HUB_POSE_Translation2d.minus(nextPose.getTranslation());

    // get the distance, heading and LUT valuse
    double distance =  toHub.getNorm();
    Rotation2d heading = toHub.getAngle();
    double[] lut = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance);
    double lutVel = lut[0] * WHEEL_TO_BALL_VELOCITY_RATIO; // correct to actual ball shooting
    double lutHoodAngle = Math.toRadians(lut[1]) + HOOD_OFFSET; // correct to actual ball pitch

    // set the horizontal (xy) velocity and the vertical (z) velocity
    double xyVel = lutVel * Math.cos(Math.toRadians(lutHoodAngle));
    double zVel = lutVel * Math.sin(Math.toRadians(lutHoodAngle));
    
    // calculate the x/y velocities correected by robot speeds
    double xVel = xyVel*heading.getCos() - robotSpeeds.vxMetersPerSecond;
    double yVel = xyVel*heading.getSin() - robotSpeeds.vyMetersPerSecond;
    xyVel = Math.hypot(xVel, yVel);

    // calculate the new ball velocity
    double ballVelocity = Math.hypot(xyVel, zVel);
    ballVelocity -= MAGNUS_CORRECTION * (ballVelocity - lutVel); // correct for Magnus (back spin) effect
    ballVelocity = ballVelocity / WHEEL_TO_BALL_VELOCITY_RATIO; // translate required ball velocity to flywheel velocity

    // calculate the hood angle
    double hoodAngle = Math.atan(zVel/xyVel) - HOOD_OFFSET; // with hood correction

    // calculate the heading
    Rotation2d ballHeading = new Rotation2d(xVel, yVel);

    // set the values
    chassis.setTargetHeading(ballHeading);
    shooter.setFlywheelVel(ballVelocity);
    shooter.setHoodAngle(hoodAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
