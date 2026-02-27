// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.Trgectory;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommon;
import frc.demacia.path.Utils.PathPoint;
import frc.demacia.utils.Utils;
import frc.demacia.utils.chassis.Chassis;

public class FollowTrajectory extends Command {
  private Chassis chassis;
  private DemaciaTrajectory trajectory;
  private ArrayList<PathPoint> points;
  private Rotation2d wantedAngle;

  public FollowTrajectory(Chassis chassis, ArrayList<PathPoint> points, Rotation2d wantedAngle) {
    this.chassis = chassis;
    this.points = points;
    this.wantedAngle = wantedAngle;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    this.trajectory = new DemaciaTrajectory(points, false, wantedAngle, chassis.getPose());

  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = chassis.getChassisSpeedsFieldRel();
    chassis.setVelocities(
        trajectory.calculate(chassis.getPose(), Utils.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));

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