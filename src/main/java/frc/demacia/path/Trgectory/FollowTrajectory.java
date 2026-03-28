// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.Trgectory;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotCommon;
import frc.demacia.path.utils.PathPoint;
import frc.demacia.utils.Utils;
import frc.demacia.utils.chassis.Chassis;

public class FollowTrajectory extends Command {
  private Chassis chassis;
  private DemaciaTrajectory trajectory;
  private ArrayList<PathPoint> points;
  private EventLoop eventLoop;
  private Timer timer;

  public FollowTrajectory(Chassis chassis, ArrayList<PathPoint> points) {
    this.chassis = chassis;
    this.points = points;
    this.eventLoop = new EventLoop();
    this.timer = new Timer();
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    this.trajectory = new DemaciaTrajectory(points, false, RobotCommon.getCurrentRobotPose());
    timer.start();
  }

  public Trigger addTriggerPosition(Pose2d pose, double meterTreshold, double rotationTreshold) {
    return new Trigger(eventLoop, () -> {
      return Math
          .abs(RobotCommon.getCurrentRobotPose().getTranslation().getDistance(pose.getTranslation())) <= meterTreshold
          && Math.abs(RobotCommon.getCurrentRobotPose().getRotation().getRadians()
              - pose.getRotation().getRadians()) <= rotationTreshold
          && timer.get() >= 0.1;
    });
  }

  public Trigger addTriggerTime(double time) {
    return new Trigger(eventLoop, () -> Math.abs(timer.get() - time) <= 0.03);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = chassis.getChassisSpeedsFieldRel();
    chassis.setVelocities(
        trajectory.calculate(RobotCommon.getCurrentRobotPose(),
            Utils.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
    eventLoop.poll();

  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();

  }

  @Override
  public boolean isFinished() {

    return trajectory.isFinishedTrajectory();
  }
}