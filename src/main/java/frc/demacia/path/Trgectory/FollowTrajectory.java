// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.Trgectory;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotCommon;
import frc.demacia.path.utils.PathPoint;
import frc.demacia.utils.Utils;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.geometry.ChassisSpeedsDemacia;
import frc.demacia.utils.geometry.Pose2dDemacia;

public class FollowTrajectory extends Command {
  private Chassis chassis;
  private DemaciaTrajectory trajectory;
  private ArrayList<PathPoint> points;
  private EventLoop eventLoop;

  public FollowTrajectory(Chassis chassis, ArrayList<PathPoint> points) {
    this.chassis = chassis;
    this.points = points;
    this.eventLoop = new EventLoop();
    addRequirements(chassis);
  }

  public Trigger addTrigger(Pose2dDemacia pose2d, double meterTreshold, double rotationTreshold) {
    return new Trigger(eventLoop, () -> {
      return RobotCommon.getCurrentRobotPose().getTranslation().getDistance(pose2d.getTranslation()) <= meterTreshold
          && Math.abs(MathUtil.angleModulus(RobotCommon.getRobotAngle().getRadians() - pose2d.getRotation().getRadians())) <= rotationTreshold;
    });
  }

  @Override
  public void initialize() {
    this.trajectory = new DemaciaTrajectory(points, false, RobotCommon.getCurrentRobotPose());

  }

  @Override
  public void execute() {
    ChassisSpeedsDemacia speeds = chassis.getChassisSpeedsFieldRel();
    chassis.setVelocities(
        trajectory.calculate(RobotCommon.getCurrentRobotPose(), Utils.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
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