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
import frc.robot.RobotCommon;
import frc.robot.RobotContainer;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Shooter.utils.ShooterUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  Shooter shooter;
  Chassis chassis;
  private double WHEEL_TO_BALL_VELOCITY_RATIO = 0.48;
  private double MAGNUS_CORRECTION = 0.05;

  public ShooterCommand(Shooter shooter, Chassis chassis) {
    this.chassis = chassis;
    this.shooter = shooter;
    addRequirements(shooter);

  }

  private void setShootingAndHood(double hoodAngle, double vel, Rotation2d heading, ChassisSpeeds robotSpeeds) {

    // set the horizontal (xy) velocity and the vertical (z) velocity
    double xyVel = vel * Math.cos(hoodAngle);
    double zVel = vel * Math.sin(hoodAngle);

    // calculate the x/y velocities correected by robot speeds
    double xVel = xyVel * heading.getCos() - robotSpeeds.vxMetersPerSecond;
    double yVel = xyVel * heading.getSin() - robotSpeeds.vyMetersPerSecond;

    xyVel = Math.hypot(xVel, yVel);
    // calculate the new ball velocity
    double ballVelocity = Math.hypot(xyVel, zVel);

    // LogManager.log("Distance from hub: " + distance + " heading to hub: " +
    // heading + " LUT vel: " + lutVel + " LUT hood angle: " + lutHoodAngle + " LUT
    // ball xyVel: " + xyVel
    // + " Z vel: " + zVel + " x Vel: " + xVel + " yVel: " + yVel + " newVel: " +
    // xyVel
    // + " ball vel pre magnus: " + ballVelocity);
    ballVelocity -= MAGNUS_CORRECTION * (ballVelocity - vel); // correct for Magnus (back spin) effect

    // LogManager.log("ball vel after magnus: " + ballVelocity);
    ballVelocity = ballVelocity / WHEEL_TO_BALL_VELOCITY_RATIO; // translate required ball velocity to
                                                                // flywheel
                                                                // velocity

    // LogManager.log("flywheel vel: " + ballVelocity);
    // calculate the hood angle
    hoodAngle = Math.atan(zVel / xyVel); // with hood correction
    // check for max angle
    if (hoodAngle > ShooterConstans.MAX_ANGLE_HOOD) {
      hoodAngle = ShooterConstans.MAX_ANGLE_HOOD;
      ballVelocity = xyVel / Math.cos(hoodAngle);
    }

    // calculate the heading
    Rotation2d ballHeading = new Rotation2d(xVel, yVel);

    // LogManager.log("new hood angle: " + hoodAngle + " ball heading: " +
    // ballHeading);
    chassis.setTargetAngle(ballHeading.getRadians());
    shooter.setFlywheelVel(ballVelocity);
    shooter.setHoodAngle(hoodAngle);

  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vel = 0;
    double hoodAngle = 0;
    Rotation2d heading = Rotation2d.kZero;
    switch (shooter.getCurrentShooterState()) {
      case DELIVERY:
        // Translation2d chassisToDelivery = RobotCommon.deliveryTarget
        //     .minus(RobotContainer.chassis.getPose().getTranslation());
        hoodAngle = Math.toRadians(45);
        // vel = 1.7 * Math.sqrt(chassisToDelivery.getNorm() * 9.81);
        // heading = chassisToDelivery.getAngle();

        setShootingAndHood(hoodAngle, vel, heading, RobotCommon.fieldRelativeSpeeds);

        break;

      case SHOOTING:
        ChassisSpeeds robotSpeeds = RobotCommon.robotRelativeSpeeds;
        Pose2d nextPose = ShooterUtils.computeFuturePosition(robotSpeeds, RobotCommon.currentRobotPose, 0.04);
        Translation2d toHub = ShooterConstans.HUB_POSE_Translation2d.minus(nextPose.getTranslation());

        // get the distance, heading and LUT valuse
        double distance = toHub.getNorm();
        heading = toHub.getAngle();

        double[] lut = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance);
        vel = lut[0] * WHEEL_TO_BALL_VELOCITY_RATIO; // correct to actual ball shooting
        hoodAngle = lut[1];

        setShootingAndHood(hoodAngle, vel, heading, RobotCommon.fieldRelativeSpeeds);

      case TRENCH:
        shooter.setHoodAngle(Math.toRadians(45));
        shooter.setFlywheelVel(10);

      default:
        shooter.setHoodPower(0);
        shooter.setFlywheelPower(0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
